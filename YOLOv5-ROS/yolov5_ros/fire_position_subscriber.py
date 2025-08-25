#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class FirePositionSubscriber(Node):
    def __init__(self):
        super().__init__('fire_position_subscriber')
        self.sub_fire_position = self.create_subscription(String, 'yolov5/fire_position', self.fire_position_callback, 10)
       
        # 아두이노와의 직렬 통신 설정
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.serial_port.flush()
        time.sleep(2)  # 시리얼 포트 초기화 후 대기 시간을 추가

        # 상태 추적 변수
        self.current_state = 'no_fire'
        self.o_active = False  # O 명령어가 활성화되었는지 여부를 추적
        self.p_active = False  # P 명령어가 활성화되었는지 여부를 추적

        # 초기 상태에서 'P' 명령 전송
        self.send_command_to_arduino('P')
        self.p_active = True

    def send_command_to_arduino(self, command):
        """명령어를 아두이노에 전송하고 로그를 출력하는 함수"""
        try:
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sent '{command}' command to Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def fire_position_callback(self, msg: String):
        fire_position = msg.data

        if fire_position == 'no_fire':
            self.handle_no_fire_state()
        else:
            self.handle_fire_detected_state(fire_position)

    def handle_no_fire_state(self):
        """불이 감지되지 않았을 때 처리 로직"""
        if self.current_state != 'no_fire':
            self.send_command_to_arduino('P')
            self.p_active = True

            if self.o_active:
                self.send_command_to_arduino('S')  # 물 분사를 중지
                self.o_active = False

            self.current_state = 'no_fire'

    def handle_fire_detected_state(self, fire_position):
        """불이 감지되었을 때 처리 로직"""
        if self.current_state == 'no_fire':
            self.send_command_to_arduino('F')
            if not self.o_active:
                self.send_command_to_arduino('O')
                self.o_active = True

            self.p_active = False
            self.current_state = 'fire_detected'

        # fire_position 값에 따라 'R' 또는 'L' 명령어를 실행
        if fire_position == 'left':
            self.send_command_to_arduino('R')
        elif fire_position == 'right':
            self.send_command_to_arduino('L')

def main(args=None):
    rclpy.init(args=args)
    fire_position_subscriber = FirePositionSubscriber()
    rclpy.spin(fire_position_subscriber)
    fire_position_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
