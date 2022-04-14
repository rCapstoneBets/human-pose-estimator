from enum import Enum
from copy import copy, deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, Vector3, TransformStamped, Quaternion

from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf_transformations as tfm

class JointType(Enum):
    FIXED = 0,
    PRISMATIC = 1,
    ROTATIONAL = 2

class JointAxis(Enum):
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2

class JointInfo:
    def __init__(self, devName: str, calibration: float, toFrameId: str, fromFrameId: str, jointType=JointType.FIXED, jointAxis=JointAxis.AXIS_X, baseTransform=Transform()):
        self.jointType = jointType           # Joint motion type
        self.jointAxis = jointAxis           # Joint Axis to move on
        self.baseTransform = baseTransform   # base offset for the joint
        self.currentTransform = Transform()  # container for the most recent transform value
        self.devName = devName               # name the device has in joint state
        self.calibration = calibration       # relationship between the transform and joint state units
        self.toFrameId = toFrameId           # the child frame
        self.fromFrameId = fromFrameId       # the parent frame to transform from

JOINT_LIST = [
    # pan joint
    JointInfo(
        devName='pan_motor', calibration=1.0/30.0,
        toFrameId='robot_base_link', fromFrameId='turret_base_link',
        jointType=JointType.ROTATIONAL, jointAxis=JointAxis.AXIS_Z,
        baseTransform=Transform(translation=Vector3(z=0.028575)),
    )
]

class TransformerNode(Node):
    def __init__(self):
        super().__init__("zed_translator")

        # set up TF system
        self.tfBroadcaster = TransformBroadcaster(self)

        self.jointStateSub = self.create_subscription(JointState, '/motor/joint_state', self.recieveJointState, qos_profile_sensor_data)

        self.get_logger().info("Joint Transformer init complete")

    def recieveJointState(self, state: JointState):
        for jointInfo in JOINT_LIST:
            # skip the logic if its fixed and just send the transform
            if(jointInfo.jointType == JointType.FIXED):
                jointInfo.currentTransform = jointInfo.baseTransform
            else: 
                try:
                    stateIdx = state.name.index(jointInfo.devName)
                    if(jointInfo.jointType == JointType.PRISMATIC):
                        #unimplemented type
                        self.get_logger().error(f'Prismatic joint type not implemented for {jointInfo.devName}')
                    else:
                        # come up with joint offset
                        euler = [0.0, 0.0, 0.0]
                        euler[jointInfo.jointAxis.value] = state.position[stateIdx] * jointInfo.calibration

                        # base quaternion
                        baseQuat = [
                            jointInfo.baseTransform.rotation.x, jointInfo.baseTransform.rotation.y,
                            jointInfo.baseTransform.rotation.z, jointInfo.baseTransform.rotation.w
                        ]

                        resultQuatVect = tfm.quaternion_multiply(tfm.quaternion_from_euler(euler[0], euler[1], euler[2]), baseQuat)
                        resultQuat = Quaternion(x=resultQuatVect[0], y=resultQuatVect[1], z=resultQuatVect[2], w=resultQuatVect[3])

                        self.get_logger().debug(f'Got result quat {resultQuat}')
                        self.get_logger().debug(f'Using base transform {jointInfo.baseTransform}')

                        jointInfo.currentTransform = deepcopy(jointInfo.baseTransform) # if this isnt a copy, we spin for the win
                        jointInfo.currentTransform.rotation = resultQuat

                
                except ValueError as ex:
                    self.get_logger().error(f'Missing {jointInfo.devName} in motor joint state message. The message had {state.name}')

            self.sendJointTf(jointInfo)

    def sendJointTf(self, jointInfo: JointInfo):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = jointInfo.toFrameId
        ts.child_frame_id = jointInfo.fromFrameId

        self.get_logger().debug(f'Sending transform for frame {jointInfo.toFrameId} from {jointInfo.fromFrameId}')

        # set translation and orientation
        ts.transform = jointInfo.currentTransform

        self.tfBroadcaster.sendTransform(ts)


def main(args=None):
    rclpy.init(args=args)

    node = TransformerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()