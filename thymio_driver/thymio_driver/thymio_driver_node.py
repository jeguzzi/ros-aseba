import array
import os
import time
from math import copysign, cos, log, sin, sqrt
from typing import Any, Callable, List, Tuple

import rclpy
import rclpy.clock
import rclpy.duration
import rclpy.node
import rclpy.parameter
import rclpy.time
import std_srvs.srv
from ament_index_python.packages import get_package_share_directory
from asebaros_msgs.msg import AsebaEvent
from asebaros_msgs.srv import GetNodeList, LoadScripts
# from builtin_interfaces.msg import Time
from geometry_msgs.msg import (Quaternion, Transform, TransformStamped, Twist,
                               Vector3)
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu, JointState, Joy, LaserScan, Range, Temperature
from std_msgs.msg import Bool, ColorRGBA, Empty, Float32, Header, Int8, Int16
from tf2_msgs.msg import TFMessage
# CHANGED: (not yet available in release) from tf.broadcaster import TransformBroadcaster
from thymio_msgs.msg import Comm, Led, LedGesture, Sound, SystemSound

BASE_WIDTH = 91.5     # millimeters
MAX_SPEED = 500.0     # units
SPEED_COEF = 2.93     # 1mm/sec corresponds to X units of real thymio speed
WHEEL_RADIUS = 0.022   # meters
GROUND_MIN_RANGE = 9     # millimeters
GROUND_MAX_RANGE = 30     # millimeters


BUTTONS = ['backward', 'forward', 'center', 'right', 'left']
PROXIMITY_NAMES = ['left', 'center_left', 'center',
                   'center_right', 'right', 'rear_left', 'rear_right']
GROUND_NAMES = ['left', 'right']
BODY_LEDS = ['bottom_left', 'bottom_right', 'top']
LED_NUMBER = {Led.CIRCLE: 8, Led.PROXIMITY: 8, Led.GROUND: 2,
              Led.REMOTE: 1, Led.BUTTONS: 4, Led.TEMPERATURE: 2, Led.MICROPHONE: 1}


def sign(x: float) -> int:
    return int(copysign(1, x))


def motor_speed_conversion(q0: float = (0.001 / SPEED_COEF), q1: float = 0
                           ) -> Tuple[Callable[[float], float], Callable[[float], float]]:
    def f(x: float) -> float:
        return q0 * x + q1 * x * abs(x)
    if q1 == 0:
        def inv_f(x: float) -> float:
            return x / q0
    else:
        def inv_f(x: float) -> float:
            return 0.5 * sign(x) * (- q0 + sqrt(q0 ** 2 + 4 * q1 * abs(x))) / q1
    return f, inv_f


class ThymioDriver(rclpy.node.Node):

    def frame_name(self, name: str) -> str:
        if self.tf_prefix:
            return f'{self.tf_prefix}/{name}'
        return name

    # CHANGED: How to replicate dynamic reconfigure?
    # def change_config(self, config, level):
    #     self.motor_speed_deadband = config.motor_speed_deadband
    #     return config

    def param_callback(self, parameters: List[rclpy.parameter.Parameter]
                       ) -> SetParametersResult:
        for param in parameters:
            if param.name == 'motor_speed_deadband':
                # TODO: enforce bounds
                self.motor_speed_deadband = param.value
                self.get_logger().info(
                    f"Changed motor_speed_deadband to {self.motor_speed_deadband}")
        response = SetParametersResult(successful=True)
        return response

    @property
    def motor_speed_deadband(self) -> float:
        return self._motor_speed_deadband

    @motor_speed_deadband.setter
    def motor_speed_deadband(self, value: float) -> None:
        self._motor_speed_deadband = value
        self.odom_msg.twist.covariance[0] = speed_cov = 0.5 * (value / 1000 / SPEED_COEF) ** 2
        self.odom_msg.twist.covariance[-1] = speed_cov / (self.axis ** 2)

    def __init__(self) -> None:
        super(ThymioDriver, self).__init__('thymio')

        self.clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.ROS_TIME)

        # load script on the Thymio
        get_aseba_nodes = self.create_client(GetNodeList, 'aseba/get_node_list')
        while not get_aseba_nodes.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_aseba_nodes service not available, waiting again...')
        while True:
            req = GetNodeList.Request()
            resp = get_aseba_nodes.call_async(req)
            rclpy.spin_until_future_complete(self, resp)
            if 'thymio-II' in resp.result().node_list:
                break
            self.get_logger().info('Waiting for thymio node ...')
            time.sleep(1)
        load_script = self.create_client(LoadScripts, 'aseba/load_script')
        while not load_script.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('load_script service not available, waiting again...')

        # script_path = pkg_resources.resource_filename('thymio_driver', 'aseba/thymio_ros.aesl')

        script_path = os.path.join(get_package_share_directory('thymio_driver'), 'thymio_ros.aesl')
        self.declare_parameter('script', script_path)
        script_path = self.get_parameter('script').value
        self.get_logger().info(f"Load aseba script {script_path}")
        req = LoadScripts.Request(file_name=script_path)
        resp = load_script.call_async(req)
        rclpy.spin_until_future_complete(self, resp)

        # initialize parameters
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('odom_max_rate', -1)
        self.declare_parameter('axis_length', BASE_WIDTH / 1000.0)
        self.declare_parameter('motor_speed_deadband', 10.0)
        # def_cal = rclpy.parameter.Parameter('', rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
        #                                     [0.001 / SPEED_COEF, 0.0])
        def_cal = [0.001 / SPEED_COEF, 0.0]
        self.declare_parameter('left_wheel_calibration/q', def_cal)
        self.declare_parameter('right_wheel_calibration/q', def_cal)
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')
        self.declare_parameter('proximity/range_min', 0.0215)
        self.declare_parameter('proximity/range_max', 0.14)
        self.declare_parameter('proximity/field_of_view', 0.3)
        self.declare_parameter('ground/threshold', 200)

        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.odom_frame = self.frame_name(self.get_parameter('odom_frame').value)
        self.robot_frame = self.frame_name(self.get_parameter('robot_frame').value)

        self.x = 0
        self.y = 0
        self.th = 0
        self.then = self.clock.now()
        print(type(self.then))
        odom_rate = self.get_parameter('odom_max_rate').value
        if odom_rate == 0:
            self.odom_min_period = -1
        else:
            self.odom_min_period = 1.0 / odom_rate
        self.odom_msg = Odometry(header=Header(frame_id=self.odom_frame),
                                 child_frame_id=self.robot_frame)

        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.covariance[0] = -1
        self.odom_msg.header.stamp = self.clock.now().to_msg()

        # subscribe to topics

        self.aseba_pub = self.create_publisher(AsebaEvent, 'aseba/events/set_speed', 1)

        self.left_wheel_angle = 0
        self.right_wheel_angle = 0

        self.axis = self.get_parameter('axis_length').value
        self.motor_speed_deadband = self.get_parameter('motor_speed_deadband').value

        left_wheel_calibration = def_cal
        self.left_wheel_speed, self.left_wheel_motor_speed = motor_speed_conversion(
            *left_wheel_calibration)
        self.get_logger().info(f'Init left wheel with calibration {left_wheel_calibration}')

        right_wheel_calibration = def_cal
        self.right_wheel_speed, self.right_wheel_motor_speed = motor_speed_conversion(
            *right_wheel_calibration)

        self.get_logger().info(f'Init right wheel with calibration {right_wheel_calibration}')

        left_wheel_joint = self.get_parameter('left_wheel_joint').value
        right_wheel_joint = self.get_parameter('right_wheel_joint').value

        self.wheel_state_msg = JointState()
        self.wheel_state_msg.name = [left_wheel_joint, right_wheel_joint]

        self.wheel_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        # CHANGED: self.odom_broadcaster = TransformBroadcaster()

        self.create_subscription(AsebaEvent, 'aseba/events/odometry', self.on_aseba_odometry_event, 1)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 1)

        self.buttons = Joy()
        self.buttons_pub = self.create_publisher(Joy, 'buttons', 1)
        self.create_subscription(AsebaEvent, 'aseba/events/buttons', self.on_aseba_buttons_event, 1)

        for button in BUTTONS:
            self.create_subscription(
                AsebaEvent, f'aseba/events/button_{button}', self.on_aseba_button_event(button), 1)

        proximity_range_min = self.get_parameter('proximity/range_min').value
        proximity_range_max = self.get_parameter('proximity/range_max').value
        proximity_field_of_view = self.get_parameter('proximity/field_of_view').value

        self.proximity_sensors = [{
            'publisher': self.create_publisher(Range, f'proximity/{name}', 1),
            'msg': Range(
                header=Header(
                    frame_id=self.frame_name('proximity_{name}_link'.format(name=name))),
                radiation_type=Range.INFRARED,
                field_of_view=proximity_field_of_view,
                min_range=proximity_range_min,
                max_range=proximity_range_max)}
            for name in PROXIMITY_NAMES]

        self.proximityToLaserPublisher = self.create_publisher(LaserScan, 'proximity/laser', 1)
        self.proximityToLaser = LaserScan(
            header=Header(frame_id=self.frame_name('laser_link')),
            angle_min=-0.64, angle_max=0.64, angle_increment=0.32,
            time_increment=0.0, scan_time=0.0, range_min=proximity_range_min + 0.08,
            range_max=proximity_range_max + 0.08)
        self.create_subscription(
            AsebaEvent, 'aseba/events/proximity', self.on_aseba_proximity_event, 1)

        self.ground_sensors = [{
            'publisher': self.create_publisher(Range, f'ground/{name}', 1),
            'msg': Range(
                header=Header(
                    frame_id=self.frame_name('ground_{name}_link'.format(name=name))),
                radiation_type=Range.INFRARED, field_of_view=proximity_field_of_view,
                min_range=(GROUND_MIN_RANGE / 1000.0), max_range=(GROUND_MAX_RANGE / 1000.0))
        } for name in GROUND_NAMES]

        ground_threshold = self.get_parameter('ground/threshold').value
        self.create_subscription(AsebaEvent, 'aseba/events/ground', self.on_aseba_ground_event, 1)
        self.imu = Imu(header=Header(frame_id=self.robot_frame))
        # no orientation or angular velocity information
        self.imu.orientation_covariance[0] = -1
        self.imu.angular_velocity_covariance[0] = -1
        # just an accelerometer
        self.imu.linear_acceleration_covariance[0] = 0.07
        self.imu.linear_acceleration_covariance[4] = 0.07
        self.imu.linear_acceleration_covariance[8] = 0.07

        self.imu_publisher = self.create_publisher(Imu, 'imu', 1)
        self.create_subscription(
            AsebaEvent, 'aseba/events/accelerometer', self.on_aseba_accelerometer_event, 1)

        self.tap_publisher = self.create_publisher(Empty, 'tap', 1)
        self.create_subscription(
            AsebaEvent, 'aseba/events/tap', self.on_aseba_tap_event, 1)

        self.temperature = Temperature(
            header=Header(frame_id=self.robot_frame))
        self.temperature.variance = 0.01
        self.temperature_publisher = self.create_publisher(Temperature, 'temperature', 1)
        self.create_subscription(
            AsebaEvent, 'aseba/events/temperature', self.on_aseba_temperature_event, 1)

        self.sound_publisher = self.create_publisher(Float32, 'sound', 1)
        self.sound_threshold_publisher = self.create_publisher(
            AsebaEvent, 'aseba/events/set_sound_threshold', 1)
        self.create_subscription(AsebaEvent, 'aseba/events/sound', self.on_aseba_sound_event, 1)
        self.create_subscription(Float32, 'sound_threshold', self.on_sound_threshold, 1)

        self.remote_publisher = self.create_publisher(Int8, 'remote', 1)
        self.create_subscription(AsebaEvent, 'aseba/events/remote', self.on_aseba_remote_event, 1)

        self.comm_rx_publisher = self.create_publisher(Comm, 'comm/rx', 1)
        self.aseba_enable_comm_publisher = self.create_publisher(
            AsebaEvent, 'aseba/events/enable_comm', 1)
        self.aseba_set_comm_tx_payload_publisher = self.create_publisher(
            AsebaEvent, 'aseba/events/set_comm_payload', 1)
        self.create_subscription(AsebaEvent, 'aseba/events/comm', self.on_aseba_comm_rx_event, 1)
        self.create_subscription(Int16, 'comm/tx', self.on_comm_tx_payload, 1)
        self.create_subscription(Bool, 'comm/enable', self.on_comm_enable, 1)

        # actuators

        for name in BODY_LEDS:
            self.create_subscription(ColorRGBA, f'led/body/{name}', self.on_body_led(name), 1)

        self.create_subscription(Led, 'led', self.on_led, 1)
        self.aseba_led_publisher = self.create_publisher(AsebaEvent, 'aseba/events/set_led', 6)

        self.create_subscription(Empty, 'led/off', self.on_led_off, 1)

        self.create_subscription(LedGesture, 'led/gesture', self.on_led_gesture, 1)
        self.aseba_led_gesture_publisher = self.create_publisher(
            AsebaEvent, 'aseba/events/set_led_gesture', 6)
        self.create_subscription(Float32, 'led/gesture/circle', self.on_led_gesture_circle, 1)
        self.create_subscription(Empty, 'led/gesture/off', self.on_led_gesture_off, 1)
        self.create_subscription(Float32, 'led/gesture/blink', self.on_led_gesture_blink, 1)
        self.create_subscription(Float32, 'led/gesture/kit', self.on_led_gesture_kit, 1)
        self.create_subscription(Empty, 'led/gesture/alive', self.on_led_gesture_alive, 1)

        self.create_subscription(Sound, 'sound/play', self.on_sound_play, 1)
        self.aseba_led_gesture_publisher = self.create_publisher(
            AsebaEvent, 'aseba/events/set_led_gesture', 6)
        self.create_subscription(SystemSound, 'sound/play/system', self.on_system_sound_play, 1)
        self.aseba_play_sound_publisher = self.create_publisher(
            AsebaEvent, 'aseba/events/play_sound', 1)
        self.aseba_play_system_sound_publisher = self.create_publisher(
            AsebaEvent, 'aseba/events/play_system_sound', 1)

        self.create_subscription(Bool, 'alarm', self.on_alarm, 1)
        self.alarm_timer = None

        self.create_subscription(Empty, 'shutdown', self.on_shutdown_msg, 1)
        self.aseba_shutdown_publisher = self.create_publisher(
            AsebaEvent, 'aseba/events/shutdown', 1)

        # REVIEW: not implemented yet
        # self.on_shutdown(self.shutdown)
        # REVIEW:
        # Server(ThymioConfig, self.change_config)
        # tell ros that we are ready
        self.create_service(std_srvs.srv.Empty, 'thymio_is_ready', self.ready)
        self.set_parameters_callback(self.param_callback)

    def on_aseba_comm_rx_event(self, msg: AsebaEvent) -> None:
        rmsg = Comm()
        rmsg.value = msg.data[0]
        # Why must I cast to a list???
        rmsg.payloads = list(msg.data[1:8])
        rmsg.intensities = list(msg.data[8:])
        self.comm_rx_publisher.publish(rmsg)

    def on_comm_enable(self, msg: Bool) -> None:
        msg = AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[msg.data])
        self.aseba_enable_comm_publisher.publish(msg)

    def on_comm_tx_payload(self, msg: Int16) -> None:
        msg = AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[msg.data])
        self.aseba_set_comm_tx_payload_publisher.publish(msg)

    def on_shutdown_msg(self, msg: Empty) -> None:
        self.aseba_shutdown_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[]))

    def ready(self, request: std_srvs.srv.Empty.Request, response: std_srvs.srv.Empty.Response
              ) -> std_srvs.srv.Empty.Response:
        return response

    def play_system_sound(self, sound: int) -> None:
        self.aseba_play_system_sound_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[sound]))

    def on_alarm(self, msg: Bool) -> None:
        if msg.data and not self.alarm_timer:
            self.alarm_timer = self.create_timer(3.0, lambda: self.play_system_sound(2))
        if not msg.data and self.alarm_timer:
            self.alarm_timer.destroy()
            self.alarm_timer = None

    def on_sound_play(self, msg: Sound) -> None:
        freq = max(1, int(msg.frequency))
        duration = rclpy.duration.Duration.from_msg(msg.duration).nanoseconds / 1e9
        duration = max(1, int(duration * 60))
        self.aseba_play_sound_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[freq, duration]))

    def on_system_sound_play(self, msg: SystemSound) -> None:
        self.play_system_sound(msg.sound)

    def set_led_gesture(self, gesture: int, leds: int, wave: int, period: float,
                        length: int, mirror: int, mask: List[int]) -> None:
        period = max(-32678, min(32678, int(period * 1000)))
        data = [gesture, leds, wave, period, length, mirror] + mask[:8]
        data += [1] * (14 - len(data))
        self.aseba_led_gesture_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=data))

    def on_led_gesture(self, msg: LedGesture) -> None:
        self.set_led_gesture(msg.gesture, msg.leds, msg.wave,
                             msg.period, msg.length, msg.mirror, msg.mask)

    def on_led_gesture_off(self, msg: Empty) -> None:
        self.set_led_gesture(LedGesture.OFF, 0, 0, 0, 0, 0, [])

    def on_led_gesture_circle(self, msg: Float32) -> None:
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.HARMONIC, msg.data, 8, 0, [])

    def on_led_gesture_blink(self, msg: Float32) -> None:
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.HARMONIC, msg.data, 1, 0, [])

    def on_led_gesture_kit(self, msg: Float32) -> None:
        self.set_led_gesture(LedGesture.WAVE, LedGesture.PROXIMITY,
                             LedGesture.HARMONIC, msg.data, 12, 11, [1, 1, 1, 1, 1, 1, 0, 0])

    def on_led_gesture_alive(self, msg: Empty) -> None:
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.RECT, 3.0, 24, 0, [])

    def on_led_off(self, msg: Empty) -> None:
        for i in LED_NUMBER.keys():
            self.aseba_led_publisher.publish(
                AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[i] + 8 * [0]))
            # sleep to avoid that aseba or ros do not process all messages.
            # could be improved by having 6 separate aseba topics where to send
            # messages
            time.sleep(0.005)

    def on_led(self, msg: Led) -> None:
        i = msg.id
        num = LED_NUMBER.get(i, 0)
        if num <= len(msg.values):
            data = [i] + [int(32 * v) for v in msg.values[:8]]
            data += [0] * (9 - len(data))
            self.aseba_led_publisher.publish(
                AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=data))

    def on_body_led(self, name: str) -> Callable[[ColorRGBA], None]:
        publisher = self.create_publisher(AsebaEvent, f'aseba/events/set_led_{name}', 1)

        def callback(msg: ColorRGBA) -> None:
            r = int(msg.r * 32)
            g = int(msg.g * 32)
            b = int(msg.b * 32)
            aseba_msg = AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[r, g, b])
            publisher.publish(aseba_msg)
        return callback

    def on_aseba_comm_event(self, msg: AsebaEvent) -> None:
        self.comm_publisher.publish(Int16(data=msg.data[0]))

    def on_aseba_remote_event(self, msg: AsebaEvent) -> None:
        self.remote_publisher.publish(Int8(data=msg.data[1]))

    def on_sound_threshold(self, msg: AsebaEvent) -> None:
        value = msg * 255
        if value < 0:
            value = 1
        if value > 255:
            value = 0
        self.sound_threshold_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[value]))

    def on_aseba_sound_event(self, msg: AsebaEvent) -> None:
        self.sound_publisher.publish(Float32(data=msg.data[0] / 255.0))

    def on_aseba_tap_event(self, msg: AsebaEvent) -> None:
        self.tap_publisher.publish(Empty())

    def on_aseba_temperature_event(self, msg: AsebaEvent) -> None:
        self.temperature.temperature = msg.data[0] / 10.0
        self.temperature_publisher.publish(self.temperature)

    # TODO check how it's implemented in the firmware.
    def on_aseba_accelerometer_event(self, msg: AsebaEvent) -> None:
        self.imu.linear_acceleration.x = msg.data[1] / 23.0 * 9.81
        self.imu.linear_acceleration.y = -msg.data[0] / 23.0 * 9.81
        self.imu.linear_acceleration.z = msg.data[2] / 23.0 * 9.81
        self.imu.header.stamp = self.clock.now().to_msg()
        self.imu_publisher.publish(self.imu)

    def on_aseba_ground_event(self, msg: AsebaEvent) -> None:
        data = msg.data
        ir_threshold = self.get_parameter("ground/threshold").value

        for sensor, value in zip(self.ground_sensors, data):
            sensor['msg'].range = float('inf') if (
                value < ir_threshold) else -float('inf')
            sensor['msg'].header.stamp = self.clock.now().to_msg()
            sensor['publisher'].publish(sensor['msg'])

    # basics logarithmic fit
    @staticmethod
    def proximity2range(raw: float) -> float:
        if raw > 4000:
            return -float('inf')
        if raw < 800:
            return float('inf')
        return -0.0736 * log(raw) + 0.632

    def on_aseba_proximity_event(self, msg: AsebaEvent) -> None:
        data = msg.data
        values = [self.proximity2range(d) for d in data]
        for sensor, value in zip(self.proximity_sensors, values):
            sensor['msg'].range = value
            sensor['msg'].header.stamp = self.clock.now().to_msg()
            sensor['publisher'].publish(sensor['msg'])

        self.proximityToLaser.ranges = []
        self.proximityToLaser.intensities = []
        self.proximityToLaser.header.stamp = self.clock.now().to_msg()
        for dist, raw in list(zip(values, data))[4::-1]:
            if dist > 0.14:
                dist = 0.14
            if dist < 0.0215:
                dist = 0.0215
            self.proximityToLaser.ranges.append(dist + 0.08)
            self.proximityToLaser.intensities.append(raw)
        self.proximityToLaserPublisher.publish(self.proximityToLaser)

    def on_aseba_button_event(self, button: str) -> Callable[[AsebaEvent], None]:
        publisher = self.create_publisher(Bool, f'buttons/{button}', 1)

        def callback(msg: AsebaEvent) -> None:
            bool_msg = Bool(data=bool(msg.data[0]))
            publisher.publish(bool_msg)
        return callback

    # ======== we send the speed to the aseba running on the robot  ========
    def set_speed(self, values: List[float]) -> None:
        self.aseba_pub.publish(AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=values))

    # ======== stop the robot safely ========
    def shutdown(self) -> None:
        self.set_speed([0, 0])

    def on_aseba_buttons_event(self, data: AsebaEvent) -> None:
        self.buttons.header.stamp = self.clock.now().to_msg()
        # data.data is array('h'), we need array('i')
        self.buttons.buttons = array.array('i', data.data)
        self.buttons_pub.publish(self.buttons)

    # ======== processing odometry events received from the robot ========
    def on_aseba_odometry_event(self, data: AsebaEvent) -> None:
        now = rclpy.time.Time.from_msg(data.stamp)
        dt = (now - self.then).nanoseconds / 1e9
        self.then = now
        stamp = now.to_msg()

        m_l, m_r = data.data
        if abs(m_l) < self.motor_speed_deadband:
            m_l = 0
        if abs(m_r) < self.motor_speed_deadband:
            m_r = 0

        vl = self.left_wheel_speed(m_l)
        vr = self.right_wheel_speed(m_r)

        # wheel joint states
        left_wheel_angular_speed = vl / WHEEL_RADIUS
        right_wheel_angular_speed = vr / WHEEL_RADIUS

        self.left_wheel_angle += dt * left_wheel_angular_speed
        self.right_wheel_angle += dt * right_wheel_angular_speed

        dsl = vl * dt  # left wheel delta in m
        dsr = vr * dt  # right wheel delta in m

        # robot traveled distance in meters
        ds = ((dsl + dsr) / 2.0)
        dth = (dsr - dsl) / self.axis  # turn angle

        self.x += ds * cos(self.th + dth / 2.0)
        self.y += ds * sin(self.th + dth / 2.0)
        self.th += dth

        # We publish odometry, tf, and wheel joint state only at a maximal rate:
        odom_time = rclpy.time.Time.from_msg(self.odom_msg.header.stamp)
        if self.odom_min_period > (now - odom_time).nanoseconds / 1e9:
            return

        self.wheel_state_msg.header.stamp = stamp
        self.wheel_state_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.wheel_state_msg.velocity = [left_wheel_angular_speed, right_wheel_angular_speed]
        self.wheel_state_pub.publish(self.wheel_state_msg)

        # prepare tf from base_link to odom
        quaternion = Quaternion()
        quaternion.z = sin(self.th / 2.0)
        quaternion.w = cos(self.th / 2.0)

        # prepare odometry
        self.odom_msg.header.stamp = stamp  # OR TO TAKE ONE FROM THE EVENT?
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation = quaternion

        if(dt > 0):
            self.odom_msg.twist.twist.linear.x = ds / dt
            self.odom_msg.twist.twist.angular.z = dth / dt
        self.odom_pub.publish(self.odom_msg)
        # publish odometry
        # self.odom_broadcaster.sendTransform(
        #     (self.x, self.y, 0),
        #     (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        #     self.then, self.robot_frame, self.odom_frame)
        # No Python TFBoradcaster yet in ros2. Send the tranform manually.
        pose = self.odom_msg.pose.pose
        translation = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
        transform = TransformStamped(
            header=self.odom_msg.header,
            child_frame_id=self.robot_frame,
            transform=Transform(translation=translation, rotation=pose.orientation))
        self.tf_pub.publish(TFMessage(transforms=[transform]))

    def set_linear_angular_speed(self, speed: float, ang_speed: float) -> None:
        left_wheel_speed = speed - ang_speed * 0.5 * self.axis
        right_wheel_speed = speed + ang_speed * 0.5 * self.axis
        left_motor_speed = round(self.left_wheel_motor_speed(left_wheel_speed))
        right_motor_speed = round(self.right_wheel_motor_speed(right_wheel_speed))
        max_motor_speed = max(abs(left_motor_speed), abs(right_motor_speed))
        if max_motor_speed > MAX_SPEED:
            return self.set_linear_angular_speed(speed * MAX_SPEED / max_motor_speed,
                                                 ang_speed * MAX_SPEED / max_motor_speed)
        self.set_speed([left_motor_speed, right_motor_speed])

    def on_cmd_vel(self, data: Twist) -> None:
        self.set_linear_angular_speed(data.linear.x, data.angular.z)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    thymio_driver = ThymioDriver()
    rclpy.spin(thymio_driver)
    thymio_driver.destroy_node()
    rclpy.shutdown()
