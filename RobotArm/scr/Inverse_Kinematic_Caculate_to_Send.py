
import math
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

max_length = 36.0
max_height = 45.0

link1 = 10.0
link2 = 18.0
link3 = 18.0
tool_length = 9.0


class InverseKinematic(Node):

    def __init__(self):
        super().__init__('inverse_kinematic_node')

        # Publisher gửi góc servo
        self.pub = self.create_publisher(
            String,
            '/stm32_cmd',
            10
        )

        # Subscriber nhận tọa độ đích X, Y, Z
        self.sub = self.create_subscription(
            Point,
            '/robot/target_position',
            self.target_callback,
            10
        )

        # Trạng thái 6 servo
        self.servo_angles = [90.0] * 6

        # Cấu hình hướng servo
        # Đổi thành -1 nếu servo quay ngược
        self.servo_direction = [1, 1, 1, 1, 1, 1]

        # Góc gập đầu gắp so với phương ngang
        self.pitch = 0.0

        # Góc xoay cổ tay
        self.wrist_rotation = 90.0

        # Trạng thái kẹp
        self.gripper_angle = 90.0

        # Timer xuất góc servo
        self.timer = self.create_timer(
            0.1,
            self.publish_angles
        )

        self.get_logger().info(
            'Robot 5 DOF + Gripper initialized'
        )

        self.publish_angles()

    def clamp(self, angle):
        return max(0.0, min(180.0, angle))

    def target_callback(self, msg):
        self.inverse_kinematics(
            msg.x,
            msg.y,
            msg.z
        )

    def inverse_kinematics(self, x, y, z):

        # Kiểm tra tọa độ hợp lệ
        if not all(
            math.isfinite(v) for v in (x, y, z)
        ):
            self.get_logger().warn(
                'Invalid target coordinates'
            )
            return

        radial = math.hypot(x, y)

        if radial > max_length or z > max_height:
            self.get_logger().warn(
                'Target outside configured workspace'
            )
            return

        # Khớp 1: xoay đế
        if radial < 1e-9:
            theta1 = 90.0
        else:
            theta1 = 90.0 + math.degrees(
                math.atan2(y, x)
            )

        # Góc định hướng đầu gắp
        pitch_rad = math.radians(self.pitch)

        # Tọa độ tâm cổ tay
        wrist_r = radial - (
            tool_length * math.cos(pitch_rad)
        )

        wrist_z = z - link1 - (
            tool_length * math.sin(pitch_rad)
        )

        # Khoảng cách từ vai tới cổ tay
        distance_sq = (
            wrist_r ** 2 + wrist_z ** 2
        )

        # Định lý cos
        cos_elbow = (
            distance_sq - link2 ** 2 - link3 ** 2
        ) / (2.0 * link2 * link3)

        if cos_elbow < -1.0 or cos_elbow > 1.0:
            self.get_logger().warn(
                'Target unreachable'
            )
            return

        # Chọn một nhánh nghiệm elbow-down
        elbow = math.acos(
            max(-1.0, min(1.0, cos_elbow))
        )

        shoulder = (
            math.atan2(wrist_z, wrist_r)
            - math.atan2(
                link3 * math.sin(elbow),
                link2 + link3 * math.cos(elbow)
            )
        )

        # Góc cổ tay để giữ pitch
        wrist_pitch = (
            pitch_rad - shoulder - elbow
        )

        # Chuyển góc hình học thành góc servo
        # Đây là ánh xạ ví dụ, cần hiệu chuẩn
        theta2 = 90.0 + math.degrees(shoulder)
        theta3 = 90.0 + math.degrees(elbow)
        theta4 = self.wrist_rotation
        theta5 = 90.0 + math.degrees(wrist_pitch)
        theta6 = self.gripper_angle

        raw_angles = [
            theta1,
            theta2,
            theta3,
            theta4,
            theta5,
            theta6
        ]

        # Không clamp nghiệm IK sai giới hạn:
        # từ chối toàn bộ mục tiêu thay vì
        # làm sai vị trí đầu gắp.
        if any(
            angle < 0.0 or angle > 180.0
            for angle in raw_angles
        ):
            self.get_logger().warn(
                'Target violates servo limits'
            )
            return

        self.servo_angles = [
            self.clamp(
                90.0 + direction * (angle - 90.0)
            )
            for angle, direction in zip(
                raw_angles,
                self.servo_direction
            )
        ]

        self.get_logger().info(
            f'Target: X={x:.2f}, '
            f'Y={y:.2f}, Z={z:.2f}'
        )

    def set_gripper(self, angle):
        self.gripper_angle = self.clamp(angle)
        self.servo_angles[5] = self.gripper_angle

    def publish_angles(self):

        msg = String()

        msg.data = ','.join(
            f'{angle:.2f}'
            for angle in self.servo_angles
        )

        self.pub.publish(msg)


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
