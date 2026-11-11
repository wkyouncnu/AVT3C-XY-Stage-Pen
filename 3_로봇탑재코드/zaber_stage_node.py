#!/usr/bin/env python3
"""
ROS 2 노드: Zaber X-LSM-E XY 스테이지 브리지 (rclpy)

구독
  /stage/target_pos  geometry_msgs/Point    x, y [mm]   → 마지막 값을 cmd_rate 로 move abs (non-blocking)
  /stage/cmd_vel     geometry_msgs/Vector3  x, y [mm/s] → move vel
                     (cmd_timeout 동안 새 명령이 없으면 자동 stop: 통신 끊김 대비)
  /stage/stop        std_msgs/Empty                     → 즉시 감속 정지
발행
  /stage/pos         geometry_msgs/PointStamped x, y [mm]  (publish_rate Hz)

파라미터: port, x_device, y_device, publish_rate, cmd_rate, cmd_deadband, cmd_timeout, home_on_start
실행:     python3 zaber_stage_node.py --ros-args -p port:=/dev/ttyUSB0
"""
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, Vector3
from std_msgs.msg import Empty

from zaber_motion import Library, Units
from zaber_motion.ascii import Connection

MM = Units.LENGTH_MILLIMETRES
MMPS = Units.VELOCITY_MILLIMETRES_PER_SECOND


class ZaberStageNode(Node):
    def __init__(self):
        super().__init__("zaber_stage")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("x_device", 1)       # 체인 주소 (/renumber 결과 기준)
        self.declare_parameter("y_device", 2)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("cmd_timeout", 0.5)  # s
        # /stage/target_pos 는 제어기가 100 Hz 로 보낸다. 받을 때마다 시리얼로 move abs 두 줄을
        # 보내면 115200 bps 한 줄(요청+응답)에 수 ms 씩 걸려 get pos 와 섞이며 명령이 밀린다.
        # → 마지막 목표만 기억했다가 cmd_rate 로 보낸다 (시뮬레이션 stage_cmd_Ts = 1/cmd_rate)
        self.declare_parameter("cmd_rate", 25.0)    # Hz
        self.declare_parameter("cmd_deadband", 0.001)  # mm, 이보다 작게 바뀌면 다시 보내지 않음
        self.declare_parameter("home_on_start", True)
        p = lambda n: self.get_parameter(n).value

        Library.enable_device_db_store()
        self.conn = Connection.open_serial_port(p("port"))
        self.conn.enable_alerts()
        self.conn.detect_devices()
        self.x = self.conn.get_device(p("x_device")).get_axis(1)
        self.y = self.conn.get_device(p("y_device")).get_axis(1)
        self.lock = threading.Lock()   # 시리얼 1개를 여러 콜백이 공유

        if p("home_on_start"):
            self.get_logger().info("Homing X/Y ...")
            self.x.home(wait_until_idle=False)
            self.y.home(wait_until_idle=False)
            self.x.wait_until_idle()
            self.y.wait_until_idle()
            self.get_logger().info("Homing done")

        self.cmd_timeout = p("cmd_timeout")
        self.cmd_deadband = p("cmd_deadband")
        self.last_vel_time = None
        self.target = None      # 마지막으로 받은 목표 (x, y) [mm]
        self.sent = None        # 마지막으로 시리얼로 보낸 목표

        self.create_subscription(Point, "/stage/target_pos", self.on_target, 10)
        self.create_subscription(Vector3, "/stage/cmd_vel", self.on_vel, 10)
        self.create_subscription(Empty, "/stage/stop", self.on_stop, 10)
        self.pub = self.create_publisher(PointStamped, "/stage/pos", 10)
        self.create_timer(1.0 / p("publish_rate"), self.on_timer)
        self.create_timer(1.0 / p("cmd_rate"), self.on_cmd_timer)

    # ---- 명령 ----
    def on_target(self, msg: Point):
        # 받기만 한다. 실제 move abs 는 on_cmd_timer 가 cmd_rate 로 보낸다
        self.target = (msg.x, msg.y)
        self.last_vel_time = None

    def on_cmd_timer(self):
        t = self.target
        if t is None or self.last_vel_time is not None:
            return
        if self.sent is not None and max(abs(t[0] - self.sent[0]), abs(t[1] - self.sent[1])) < self.cmd_deadband:
            return
        try:
            with self.lock:
                # 움직이는 중에 새 move abs 가 오면 펌웨어가 그 자리에서 목표만 바꾼다
                self.x.move_absolute(t[0], MM, wait_until_idle=False)
                self.y.move_absolute(t[1], MM, wait_until_idle=False)
            self.sent = t
        except Exception as e:
            self.get_logger().error(f"move abs failed: {e}")

    def on_vel(self, msg: Vector3):
        with self.lock:
            self.x.move_velocity(msg.x, MMPS)
            self.y.move_velocity(msg.y, MMPS)
            self.last_vel_time = self.get_clock().now()

    def on_stop(self, _msg=None):
        with self.lock:
            self.last_vel_time = None
            self.target = None      # 정지 뒤에는 새 목표가 올 때까지 움직이지 않는다
            self.x.stop(wait_until_idle=False)
            self.y.stop(wait_until_idle=False)

    # ---- 주기 처리: 위치 발행 + 속도명령 watchdog ----
    def on_timer(self):
        if self.last_vel_time is not None:
            dt = (self.get_clock().now() - self.last_vel_time).nanoseconds * 1e-9
            if dt > self.cmd_timeout:
                self.get_logger().warn("cmd_vel timeout → stop")
                self.on_stop()
        try:
            with self.lock:
                px = self.x.get_position(MM)
                py = self.y.get_position(MM)
        except Exception as e:  # 경고 플래그/통신 오류 등
            self.get_logger().error(f"position read failed: {e}")
            return
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "stage_base"
        out.point.x, out.point.y = px, py
        self.pub.publish(out)

    def destroy_node(self):
        try:
            self.on_stop()
            self.conn.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = ZaberStageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
