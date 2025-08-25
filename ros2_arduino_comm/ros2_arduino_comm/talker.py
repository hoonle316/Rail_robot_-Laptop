import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import serial

class CustomController(Node):

    def __init__(self):
        super().__init__('custom_controller')
        self.subscription = self.create_subscription(
            Int32,
            'steps',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.joint_name = "slider_to_cart"
        self.current_position = 2.0
        self.serial_port = '/dev/ttyACM0'  # 아두이노 시리얼 포트 경로
        self.baud_rate = 9600  # 시리얼 통신 속도
        self.arduino = None
        self.connect_to_arduino()
        self.steps_to_meters_factor = 0.0000075  # 스텝 당 이동 거리 (m/스텝)
        self.sync_factor = 10  # 실제 슬라이드바와 URDF 모델 간의 비율

    def connect_to_arduino(self):
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        self.current_position += msg.data * self.steps_to_meters_factor * self.sync_factor
        self.send_trajectory_command(self.current_position)
        self.send_to_arduino(msg.data)

    def send_trajectory_command(self, position):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 1
        traj_msg.points = [point]
        self.publisher_.publish(traj_msg)

    def send_to_arduino(self, steps):
        if self.arduino is not None:
            self.serial_port.write(b'M')
            command = f"{steps}\n"
            self.arduino.write(command.encode())
            self.get_logger().info(f'Sent to Arduino: "{command.strip()}"')

def main(args=None):
    rclpy.init(args=args)
    custom_controller = CustomController()
    rclpy.spin(custom_controller)
    custom_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
