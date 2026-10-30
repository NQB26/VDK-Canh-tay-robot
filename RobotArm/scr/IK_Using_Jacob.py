
import math
import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64


# =========================================
# THONG SO ROBOT
# =========================================

LINK1 = 10.0
LINK2 = 18.0
LINK3 = 18.0
TOOL_LENGTH = 9.0

DT = 0.02
GAIN = 1.5
DAMPING = 3.0
MAX_SPEED = math.radians(25.0)
POSITION_TOLERANCE = 0.3

# Gioi han servo
SERVO_MIN = np.zeros(5)
SERVO_MAX = np.full(5, 180.0)

# Goc servo trung tinh
SERVO_ZERO = np.full(5, 90.0)

# Hieu chuan servo:
# Servo = SERVO_ZERO + direction * (q - q_zero)
# q_zero la goc hinh hoc tai vi tri 90 do.
#
# Vi du nay gia dinh:
# q1 = 0, q2 = 0, q3 = 0, q4 = 0, q5 = 0
# khi tat ca servo o 90 do.
#
# CAN HIEU CHUAN THEO ROBOT THAT.
Q_ZERO = np.zeros(5)

SERVO_DIRECTION = np.array(
    [1, 1, 1, 1, 1],
    dtype=float
)


class InverseKinematic(Node):

    def __init__(self):
        super().__init__('inverse_kinematic_node')

        self.pub = self.create_publisher(
            String,
            '/stm32_cmd',
            10
        )

        self.position_pub = self.create_publisher(
            Point,
            '/robot/current_position',
            10
        )

        self.error_pub = self.create_publisher(
            Float64,
            '/robot/position_error',
            10
        )

        self.sub = self.create_subscription(
            Point,
            '/robot/target_position',
            self.target_callback,
            10
        )

        self.gripper_sub = self.create_subscription(
            Float64,
            '/robot/gripper_angle',
            self.gripper_callback,
            10
        )

        # Goc khop hinh hoc, radian
        self.q = Q_ZERO.copy()

        # Servo kep
        self.gripper = 90.0

        # Chua co muc tieu
        self.target = None

        self.timer = self.create_timer(
            DT,
            self.control_loop
        )

        self.get_logger().info(
            'Jacobian IK 5DOF initialized'
        )

        self.publish_state()

    # =====================================
    # DONG HOC THUAN
    # =====================================

    def forward_kinematics(self, q):

        q1, q2, q3, q4, q5 = q

        a = q2
        b = q2 + q3
        c = q2 + q3 + q5

        # Ban kinh tu truc de den dau gap
        r = (
            LINK2 * math.cos(a)
            + LINK3 * math.cos(b)
            + TOOL_LENGTH * math.cos(c)
        )

        # Chieu cao dau gap
        z = (
            LINK1
            + LINK2 * math.sin(a)
            + LINK3 * math.sin(b)
            + TOOL_LENGTH * math.sin(c)
        )

        x = r * math.cos(q1)
        y = r * math.sin(q1)

        return np.array([x, y, z])

    # =====================================
    # MA TRAN JACOBIAN 3x5
    # =====================================

    def jacobian(self, q):

        q1, q2, q3, q4, q5 = q

        a = q2
        b = q2 + q3
        c = q2 + q3 + q5

        r = (
            LINK2 * math.cos(a)
            + LINK3 * math.cos(b)
            + TOOL_LENGTH * math.cos(c)
        )

        dr2 = (
            -LINK2 * math.sin(a)
            - LINK3 * math.sin(b)
            - TOOL_LENGTH * math.sin(c)
        )

        dr3 = (
            -LINK3 * math.sin(b)
            - TOOL_LENGTH * math.sin(c)
        )

        dr5 = -TOOL_LENGTH * math.sin(c)

        dz2 = (
            LINK2 * math.cos(a)
            + LINK3 * math.cos(b)
            + TOOL_LENGTH * math.cos(c)
        )

        dz3 = (
            LINK3 * math.cos(b)
            + TOOL_LENGTH * math.cos(c)
        )

        dz5 = TOOL_LENGTH * math.cos(c)

        c1 = math.cos(q1)
        s1 = math.sin(q1)

        J = np.array([
            [
                -r * s1,
                dr2 * c1,
                dr3 * c1,
                0.0,
                dr5 * c1
            ],
            [
                r * c1,
                dr2 * s1,
                dr3 * s1,
                0.0,
                dr5 * s1
            ],
            [
                0.0,
                dz2,
                dz3,
                0.0,
                dz5
            ]
        ])

        return J

    # =====================================
    # GOC KHOP -> GOC SERVO
    # =====================================

    def joint_to_servo(self, q):

        angles = (
            SERVO_ZERO
            + SERVO_DIRECTION
            * np.degrees(q - Q_ZERO)
        )

        return angles

    # =====================================
    # NHAN TOA DO DICH
    # =====================================

    def target_callback(self, msg):

        target = np.array([
            msg.x,
            msg.y,
            msg.z
        ], dtype=float)

        if not np.all(np.isfinite(target)):
            self.get_logger().warn(
                'Invalid target coordinates'
            )
            return

        self.target = target

        self.get_logger().info(
            f'New target: {target}'
        )

    # =====================================
    # DIEU KHIEN KEP
    # =====================================

    def gripper_callback(self, msg):

        if not math.isfinite(msg.data):
            return

        self.gripper = float(
            np.clip(msg.data, 0.0, 180.0)
        )

    # =====================================
    # VONG LAP DIEU KHIEN
    # =====================================

    def control_loop(self):

        if self.target is not None:

            position = self.forward_kinematics(
                self.q
            )

            error = self.target - position

            error_norm = np.linalg.norm(error)

            if error_norm > POSITION_TOLERANCE:

                J = self.jacobian(self.q)

                # Damped Least Squares
                A = (
                    J @ J.T
                    + (DAMPING ** 2) * np.eye(3)
                )

                velocity = GAIN * error

                q_dot = (
                    J.T
                    @ np.linalg.solve(A, velocity)
                )

                # Gioi han van toc
                q_dot = np.clip(
                    q_dot,
                    -MAX_SPEED,
                    MAX_SPEED
                )

                # Tinh goc moi
                q_candidate = self.q + q_dot * DT

                # Gioi han theo goc servo
                servo_candidate = (
                    self.joint_to_servo(q_candidate)
                )

                servo_candidate = np.clip(
                    servo_candidate,
                    SERVO_MIN,
                    SERVO_MAX
                )

                # Chuyen nguoc lai ve goc khop
                self.q = (
                    Q_ZERO
                    + np.radians(
                        (
                            servo_candidate
                            - SERVO_ZERO
                        ) / SERVO_DIRECTION
                    )
                )

        self.publish_state()

    # =====================================
    # GUI TRANG THAI
    # =====================================

    def publish_state(self):

        servo = self.joint_to_servo(self.q)

        all_angles = list(servo) + [
            self.gripper
        ]

        msg = String()

        msg.data = ','.join(
            f'{angle:.2f}'
            for angle in all_angles
        )

        self.pub.publish(msg)

        position = self.forward_kinematics(
            self.q
        )

        pos_msg = Point()

        pos_msg.x = float(position[0])
        pos_msg.y = float(position[1])
        pos_msg.z = float(position[2])

        self.position_pub.publish(pos_msg)

        error_msg = Float64()

        if self.target is None:
            error_msg.data = 0.0
        else:
            error_msg.data = float(
                np.linalg.norm(
                    self.target - position
                )
            )

        self.error_pub.publish(error_msg)


def main(args=None):

    rclpy.init(args=args)

    node = InverseKinematic()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
