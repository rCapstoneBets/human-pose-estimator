import math

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.qos import qos_profile_system_default

from zed_interfaces.msg import ObjectsStamped, Object
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped, Pose, Vector3, Quaternion, Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

import tf_transformations

LAST_SEEN_TIMEOUT = 2.0
PUBLISH_TIMER_RATE = 0.01
DEFAULT_ORIENTATION = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
LOOKUP_DELAY = 0.30

BASE_FRAME = 'robot_base_link'
SHOOT_FRAME = 'shoot_link'
CAMERA_FRAME = 'camera_link'
PLAYER_FRAME = 'player_link'

# Skeleton joints indices
#        16-14   15-17
#             \ /
#              0
#              |
#       2------1------5
#       |    |   |    |
#       3    |   |    6
#       |    |   |    |
#       4    8   11   7
#            |   |
#            9   12
#            |   |
#           10   13
ARM_SKEL_KPI = [2, 3, 4, 5, 6, 7]
ARM_SKEL_NAMES = ['r_shoulder', 'r_elbow', 'r_hand', 'l_shoulder', 'l_elbow', 'l_hand']

class TranslatorNode(Node):
    def __init__(self):
        super().__init__("zed_translator")

        # set up TF system
        self.buffer = Buffer()
        self.tfListener = TransformListener(self.buffer, self)
        self.tfBroadcaster = TransformBroadcaster(self)

        # setup subscriber to pull data from ZED
        self.ObjectsSub = self.create_subscription(ObjectsStamped, '/zed2/zed_node/obj_det/objects', self.getNewObjectData, qos_profile_system_default)
        self.trackedObjId = -1
        self.lastTrackTime = self.get_clock().now()

        # create a timer to publish the last estimate of the position
        self.publishTimer = self.create_timer(PUBLISH_TIMER_RATE, self.publishData)

        # set up signal publisher
        self.signalPub = self.create_publisher(Bool, '/player/signal', qos_profile_system_default)
        self.signalMsg = Bool(data=False)

        # set up pose publisher
        self.posePublisher = self.create_publisher(Pose, '/player/pose', qos_profile_system_default)
        self.lastPlayerPose = Pose()

        self.get_logger().info("Zed translator init complete")

    def getPlayerPoseInShoot(self):
        now = self.get_clock().now() - Duration(nanoseconds=int(LOOKUP_DELAY * 1000000000))
        playerPoseRelShot = Pose()
        try:
            # lookup transform
            ts = self.buffer.lookup_transform(
                SHOOT_FRAME,
                PLAYER_FRAME, 
                now
            )

            # copy data over from the TF lookup
            playerPoseRelShot.position = Point(x=ts.transform.translation.x, y=ts.transform.translation.y, z=ts.transform.translation.z)
            playerPoseRelShot.orientation = ts.transform.rotation

        except TransformException as ex:
            self.get_logger().error(f'Could not transform {PLAYER_FRAME} to {SHOOT_FRAME} : {ex}')
        
        return playerPoseRelShot

    def sendSkeletonPointTf(self, frameName: str, point: Vector3):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = CAMERA_FRAME
        ts.child_frame_id = frameName

        # set translation and orientation
        ts.transform.translation = point
        ts.transform.rotation = DEFAULT_ORIENTATION

        self.tfBroadcaster.sendTransform(ts)

    def strictlyIncreasing(self, values: list) -> bool:
        for i in range(len(values)):
            if(i == 0): continue # skip first index
            if(values[i] < values[i-1]): return False

        return True

    def determinePlayerSignal(self, object: Object):
        skeleton = object.skeleton_3d.keypoints
        # Skeleton joints indices
        #        16-14   15-17
        #             \ /
        #              0
        #              |
        #       2------1------5
        #       |    |   |    |
        #       3    |   |    6
        #       |    |   |    |
        #       4    8   11   7
        #            |   |
        #            9   12
        #            |   |
        #           10   13

        # create lists for left and right arm z coords
        leftArmZ = []
        rightArmZ = []

        # send the point to TF to see in rviz
        for index in range(len(ARM_SKEL_KPI)):
            skelPtIdx = ARM_SKEL_KPI[index]
            skelPtName = ARM_SKEL_NAMES[index]

            # hand transform to TF
            bodyPoint = Vector3(x=float(skeleton[skelPtIdx].kp[0]), y=float(skeleton[skelPtIdx].kp[1]), z=float(skeleton[skelPtIdx].kp[2]))
            self.sendSkeletonPointTf(skelPtName, bodyPoint)

            # add z to correct list
            if(index < len(ARM_SKEL_KPI) / 2): leftArmZ.append(bodyPoint.z) # left arm
            else: rightArmZ.append(bodyPoint.z)# right arm

        return Bool(data=self.strictlyIncreasing(leftArmZ) or self.strictlyIncreasing(rightArmZ))

    def getNewObjectData(self, msg: ObjectsStamped):
        # for object in msg.objects:
        #     self.get_logger().info(f'Detection with label: {object.label}, label_id {object.label_id} and sublabel {object.sublabel}')

        trackedObj = None
        # iterate all detected objects
        for object in msg.objects:

            # skip if invalid object or no skeleton availiable
            if(object.tracking_state == 0 or not object.skeleton_available):
                continue
        
            # figure out how to lock on to one track ID
            if(self.trackedObjId == -1 or self.get_clock().now() - self.lastTrackTime > Duration(nanoseconds=int(LAST_SEEN_TIMEOUT * 1000000000))):
                self.get_logger().info(f'Lost prior track: {self.trackedObjId }, switching to new track: {object.label_id}')
                self.trackedObjId = object.label_id

            # skip if not the tracked object
            if(object.label_id == self.trackedObjId):
                trackedObj = object
                self.lastTrackTime = self.get_clock().now()
                break

        if(trackedObj is None and self.get_clock().now() - self.lastTrackTime > Duration(nanoseconds=int(LAST_SEEN_TIMEOUT * 1000000000))):
            self.lastTrackTime = self.get_clock().now()
            self.trackedObjId = -1
            self.get_logger().warning(f'No valid detections')

        if(trackedObj is None):
            return
            

        trackedBodyPt = trackedObj.head_position
        self.get_logger().debug(f'Found new valid detection {trackedBodyPt}')

        # process body point
        bodyPoint = Vector3(x=float(trackedBodyPt[0]), y=float(trackedBodyPt[1]), z=float(trackedBodyPt[2]))
        self.sendSkeletonPointTf(PLAYER_FRAME, bodyPoint)

        # get relative pose to shoot
        self.lastPlayerPose = self.getPlayerPoseInShoot()

        # examine skeleton for signal
        self.signalMsg = self.determinePlayerSignal(trackedObj)
        if(self.signalMsg.data):
            self.get_logger().warning(f'Track_id {trackedObj.label_id} is signaling')
            

    def publishData(self):
        self.posePublisher.publish(self.lastPlayerPose)
        self.signalPub.publish(self.signalMsg)


def main(args=None):
    rclpy.init(args=args)

    node = TranslatorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()