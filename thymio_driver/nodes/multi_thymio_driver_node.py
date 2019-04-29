#!/usr/bin/env python

import os
from math import copysign, cos, log, sin, sqrt

import roslib
import rospy
import std_srvs.srv
from asebaros_msgs.msg import AsebaEvent
from asebaros_msgs.srv import GetNodeList, LoadScripts, LoadScriptToTarget
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, Joy, LaserScan, Range, Temperature
from std_msgs.msg import Bool, ColorRGBA, Empty, Float32, Int8, Int16
from tf.broadcaster import TransformBroadcaster
from thymio_driver.cfg import ThymioConfig
from thymio_msgs.msg import Led, LedGesture, Sound, SystemSound
import roslaunch
import subprocess


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


def sign(x):
    return copysign(1, x)


def motor_speed_conversion(q0=(0.001 / SPEED_COEF), q1=0):
    def f(x):
        return q0 * x + q1 * x * abs(x)
    if q1 == 0:
        def inv_f(x):
            return x / q0
    else:
        def inv_f(x):
            return 0.5 * sign(x) * (- q0 + sqrt(q0 ** 2 + 4 * q1 * abs(x))) / q1
    return f, inv_f


class ThymioDriver(object):

    def frame_name(self, name):
        if self.tf_prefix:
            return '{self.tf_prefix}/{name}'.format(**locals())
        return name

    # def change_config(self, config, level):
    #     # self.diff_factor = config.diff_factor
    #     # self.ticks2mm = config.ticks2mm
    #     self.motor_speed_deadband = config.motor_speed_deadband
    #     return config

    @property
    def motor_speed_deadband(self):
        return self._motor_speed_deadband

    @motor_speed_deadband.setter
    def motor_speed_deadband(self, value):
        self._motor_speed_deadband = value
        self.odom_msg.twist.covariance[0] = speed_cov = 0.5 * (value / 1000 / SPEED_COEF) ** 2
        self.odom_msg.twist.covariance[-1] = speed_cov / (self.axis ** 2)

    def __init__(self, node_name):
        # initialize parameters
        # node_name = "thymio%d" % id
        # ns = '/%s' % name
        def _aseba(topic):
            return 'aseba/{name}/events/{topic}'.format(name=node_name, topic=topic)

        def _ros(topic):
            return '{name}/{topic}'.format(name=node_name, topic=topic)

        self.tf_prefix = node_name

        self.odom_frame = self.frame_name(rospy.get_param('~odom_frame', 'odom'))
        self.robot_frame = self.frame_name(rospy.get_param('~robot_frame', 'base_link'))

        self.x = 0
        self.y = 0
        self.th = 0
        self.then = rospy.Time.now()
        odom_rate = rospy.get_param('~odom_max_rate', -1)
        if odom_rate == 0:
            self.odom_min_period = -1
        else:
            self.odom_min_period = 1.0 / odom_rate
        self.odom_msg = Odometry(header=rospy.Header(frame_id=self.odom_frame),
                                 child_frame_id=self.robot_frame)

        self.odom_msg.pose.pose.position.z = 0
        self.odom_msg.pose.covariance[0] = -1
        self.odom_msg.header.stamp = rospy.Time.now()

        # subscribe to topics

        self.aseba_pub = rospy.Publisher(_aseba('set_speed'), AsebaEvent, queue_size=1)

        self.left_wheel_angle = 0
        self.right_wheel_angle = 0

        self.axis = rospy.get_param('~axis_length', BASE_WIDTH / 1000.0)
        self.motor_speed_deadband = rospy.get_param('~motor_speed_deadband', 10)
        # self.ticks2mm = rospy.get_param('~ticks2mm', 1.0 / SPEED_COEF)
        # self.diff_factor = rospy.get_param('~diff_factor', 1.0)

        def_cal = [0.001 / SPEED_COEF, 0]

        left_wheel_calibration = rospy.get_param('~left_wheel_calibration/q', def_cal)

        self.left_wheel_speed, self.left_wheel_motor_speed = motor_speed_conversion(
            *left_wheel_calibration)
        # rospy.loginfo('Init left wheel with calibration %s', left_wheel_calibration)

        right_wheel_calibration = rospy.get_param('~right_wheel_calibration/q', def_cal)
        self.right_wheel_speed, self.right_wheel_motor_speed = motor_speed_conversion(
            *right_wheel_calibration)

        # rospy.loginfo('Init right wheel with calibration %s', right_wheel_calibration)

        left_wheel_joint = rospy.get_param('~left_wheel_joint', 'left_wheel_joint')
        right_wheel_joint = rospy.get_param('~right_wheel_joint', 'right_wheel_joint')

        self.wheel_state_msg = JointState()
        self.wheel_state_msg.name = [left_wheel_joint, right_wheel_joint]

        self.wheel_state_pub = rospy.Publisher(_ros('joint_states'), JointState, queue_size=1)
        self.odom_pub = rospy.Publisher(_ros('odom'), Odometry, queue_size=1)
        if rospy.get_param('~broadcast_odom_tf', True):
            self.odom_broadcaster = TransformBroadcaster()
        else:
            self.odom_broadcaster = None

        rospy.Subscriber(_aseba('odometry'), AsebaEvent, self.on_aseba_odometry_event)
        rospy.Subscriber(_ros('cmd_vel'), Twist, self.on_cmd_vel)

        self.buttons = Joy()
        self.buttons_pub = rospy.Publisher(_ros('buttons'), Joy, queue_size=1)
        rospy.Subscriber(_aseba("buttons"), AsebaEvent, self.on_aseba_buttons_event)

        for button in BUTTONS:
            rospy.Subscriber(_aseba('button_%s' % button),
                             AsebaEvent, self.on_aseba_button_event(_ros('buttons/%s' % button)))

        proximity_range_min = rospy.get_param('~proximity/range_min', 0.0215)
        proximity_range_max = rospy.get_param('~proximity/range_min', 0.14)
        proximity_field_of_view = rospy.get_param('~proximity/field_of_view', 0.3)

        self.proximity_sensors = [{
            'publisher': rospy.Publisher(_ros('proximity/%s' % name), Range, queue_size=1),
            'msg': Range(
                header=rospy.Header(
                    frame_id=self.frame_name('proximity_{name}_link'.format(name=name))),
                radiation_type=Range.INFRARED,
                field_of_view=proximity_field_of_view,
                min_range=proximity_range_min,
                max_range=proximity_range_max)}
            for name in PROXIMITY_NAMES]

        self.proximityToLaserPublisher = rospy.Publisher(
            _ros('proximity/laser'), LaserScan, queue_size=1)
        self.proximityToLaser = LaserScan(
            header=rospy.Header(frame_id=self.frame_name('laser_link')),
            angle_min=-0.64, angle_max=0.64, angle_increment=0.32,
            time_increment=0, scan_time=0, range_min=proximity_range_min + 0.08,
            range_max=proximity_range_max + 0.08)
        rospy.Subscriber(_aseba('proximity'), AsebaEvent, self.on_aseba_proximity_event)

        self.ground_sensors = [{
            'publisher': rospy.Publisher(_ros('ground/%s' % name), Range, queue_size=1),
            'msg': Range(
                header=rospy.Header(
                    frame_id=self.frame_name('ground_{name}_link'.format(name=name))),
                radiation_type=Range.INFRARED, field_of_view=proximity_field_of_view,
                min_range=(GROUND_MIN_RANGE / 1000.0), max_range=(GROUND_MAX_RANGE / 1000.0))
        } for name in GROUND_NAMES]

        ground_threshold = rospy.get_param('~ground/threshold', 200)

        rospy.Subscriber(_aseba('ground'), AsebaEvent, self.on_aseba_ground_event)
        rospy.set_param("~ground_threshold", ground_threshold)

        self.imu = Imu(header=rospy.Header(frame_id=self.robot_frame))
        # no orientation or angular velocity information
        self.imu.orientation_covariance[0] = -1
        self.imu.angular_velocity_covariance[0] = -1
        # just an accelerometer
        self.imu.linear_acceleration_covariance[0] = 0.07
        self.imu.linear_acceleration_covariance[4] = 0.07
        self.imu.linear_acceleration_covariance[8] = 0.07

        self.imu_publisher = rospy.Publisher(_ros('imu'), Imu, queue_size=1)
        rospy.Subscriber(_aseba('accelerometer'), AsebaEvent, self.on_aseba_accelerometer_event)

        self.tap_publisher = rospy.Publisher(_ros('tap'), Empty, queue_size=1)
        rospy.Subscriber(_aseba('tap'), AsebaEvent, self.on_aseba_tap_event)

        self.temperature = Temperature(
            header=rospy.Header(frame_id=self.robot_frame))
        self.temperature.variance = 0.01
        self.temperature_publisher = rospy.Publisher(
            _ros('temperature'), Temperature, queue_size=1)
        rospy.Subscriber(_aseba('temperature'), AsebaEvent, self.on_aseba_temperature_event)

        self.sound_publisher = rospy.Publisher(_ros('sound'), Float32, queue_size=1)
        self.sound_threshold_publisher = rospy.Publisher(
            _aseba('set_sound_threshold'), AsebaEvent, queue_size=1)
        rospy.Subscriber(_aseba('sound'), AsebaEvent, self.on_aseba_sound_event)
        rospy.Subscriber(_ros('sound_threshold'), Float32, self.on_sound_threshold)

        self.remote_publisher = rospy.Publisher(_ros('remote'), Int8, queue_size=1)
        rospy.Subscriber(_aseba('remote'), AsebaEvent, self.on_aseba_remote_event)

        self.comm_rx_publisher = rospy.Publisher(_ros('comm/rx'), Int16, queue_size=1)
        self.aseba_enable_comm_publisher = rospy.Publisher(
            _aseba('enable_comm'), AsebaEvent, queue_size=1)
        self.aseba_set_comm_tx_payload_publisher = rospy.Publisher(
            _aseba('set_comm_payload'), AsebaEvent, queue_size=1)
        rospy.Subscriber(_ros('comm/tx'), Int16, self.on_comm_tx_payload)
        rospy.Subscriber(_ros('comm/enable'), Bool, self.on_comm_enable)
        rospy.Subscriber(_aseba('comm'), AsebaEvent, self.on_aseba_comm_rx_event)

        # actuators

        for name in BODY_LEDS:
            rospy.Subscriber(_ros('led/body/%s' % name), ColorRGBA,
                             self.on_body_led(_aseba('set_led_%s' % name)))

        rospy.Subscriber(_ros('led'), Led, self.on_led)
        self.aseba_led_publisher = rospy.Publisher(_aseba('set_led'), AsebaEvent, queue_size=6)

        rospy.Subscriber(_ros('led/off'), Empty, self.on_led_off)

        rospy.Subscriber(_ros('led/gesture'), LedGesture, self.on_led_gesture)
        self.aseba_led_gesture_publisher = rospy.Publisher(
            _aseba('set_led_gesture'), AsebaEvent, queue_size=6)
        rospy.Subscriber(_ros('led/gesture/circle'), Float32, self.on_led_gesture_circle)
        rospy.Subscriber(_ros('led/gesture/off'), Empty, self.on_led_gesture_off)
        rospy.Subscriber(_ros('led/gesture/blink'), Float32, self.on_led_gesture_blink)
        rospy.Subscriber(_ros('led/gesture/kit'), Float32, self.on_led_gesture_kit)
        rospy.Subscriber(_ros('led/gesture/alive'), Empty, self.on_led_gesture_alive)

        rospy.Subscriber(_ros('sound/play'), Sound, self.on_sound_play)
        self.aseba_led_gesture_publisher = rospy.Publisher(
            _aseba('set_led_gesture'), AsebaEvent, queue_size=6)
        rospy.Subscriber(_ros('sound/play/system'), SystemSound, self.on_system_sound_play)
        self.aseba_play_sound_publisher = rospy.Publisher(
            _aseba('play_sound'), AsebaEvent, queue_size=1)
        self.aseba_play_system_sound_publisher = rospy.Publisher(
            _aseba('play_system_sound'), AsebaEvent, queue_size=1)

        rospy.Subscriber(_ros('alarm'), Bool, self.on_alarm)
        self.alarm_timer = None

        rospy.Subscriber(_ros('shutdown'), Empty, self.on_shutdown_msg)
        self.aseba_shutdown_publisher = rospy.Publisher(
            _aseba('shutdown'), AsebaEvent, queue_size=1)

        rospy.on_shutdown(self.shutdown)

        # Server(ThymioConfig, self.change_config)
        # tell ros that we are ready
        rospy.Service(_ros('thymio_is_ready'), std_srvs.srv.Empty, self.ready)


    def on_comm_enable(self, msg):
        msg = AsebaEvent(rospy.get_rostime(), 0, [msg.data])
        self.aseba_enable_comm_publisher.publish(msg)

    def on_comm_tx_payload(self, msg):
        msg = AsebaEvent(rospy.get_rostime(), 0, [msg.data])
        self.aseba_set_comm_tx_payload_publisher.publish(msg)

    def on_shutdown_msg(self, msg):
        self.aseba_shutdown_publisher.publish(
            AsebaEvent(rospy.get_rostime(), 0, []))

    def ready(self, req):
        return std_srvs.srv.Empty()

    def play_system_sound(self, sound):
        self.aseba_play_system_sound_publisher.publish(
            AsebaEvent(rospy.get_rostime(), 0, [sound]))

    def alarm_cb(self, evt):
        self.play_system_sound(2)

    def on_alarm(self, msg):
        if msg.data and not self.alarm_timer:
            self.alarm_timer = rospy.Timer(rospy.Duration(3), self.alarm_cb)
        if not msg.data and self.alarm_timer:
            self.alarm_timer.shutdown()
            self.alarm_timer = None

    def on_sound_play(self, msg):
        freq = max(1, int(msg.frequency))
        duration = max(1, int(msg.duration.to_sec() * 60))
        self.aseba_play_sound_publisher.publish(
            AsebaEvent(rospy.get_rostime(), 0, [freq, duration]))

    def on_system_sound_play(self, msg):
        self.play_system_sound(msg.sound)

    def set_led_gesture(self, gesture, leds, wave, period, length, mirror, mask):
        period = max(-32678, min(32678, int(period * 1000)))
        data = [gesture, leds, wave, period, length, mirror] + mask[:8]
        data += [1] * (14 - len(data))
        self.aseba_led_gesture_publisher.publish(
            AsebaEvent(rospy.get_rostime(), 0, data))

    def on_led_gesture(self, msg):
        self.set_led_gesture(msg.gesture, msg.leds, msg.wave,
                             msg.period, msg.length, msg.mirror, msg.mask)

    def on_led_gesture_off(self, msg):
        self.set_led_gesture(LedGesture.OFF, 0, 0, 0, 0, 0, [])

    def on_led_gesture_circle(self, msg):
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.HARMONIC, msg.data, 8, 0, [])

    def on_led_gesture_blink(self, msg):
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.HARMONIC, msg.data, 1, 0, [])

    def on_led_gesture_kit(self, msg):
        self.set_led_gesture(LedGesture.WAVE, LedGesture.PROXIMITY,
                             LedGesture.HARMONIC, msg.data, 12, 11, [1, 1, 1, 1, 1, 1, 0, 0])

    def on_led_gesture_alive(self, msg):
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.RECT, 3.0, 24, 0, [])

    def on_led_off(self, msg):
        for i in LED_NUMBER.keys():
            self.aseba_led_publisher.publish(
                AsebaEvent(rospy.get_rostime(), 0, [i] + 8 * [0]))
            # sleep to avoid that aseba or ros do not process all messages.
            # could be improved by having 6 separate aseba topics where to send
            # messages
            rospy.sleep(0.005)

    def on_led(self, msg):
        i = msg.id
        num = LED_NUMBER.get(i, 0)
        if num <= len(msg.values):
            data = [i] + [int(32 * v) for v in msg.values[:8]]
            data += [0] * (9 - len(data))
            self.aseba_led_publisher.publish(
                AsebaEvent(rospy.get_rostime(), 0, data))

    def on_body_led(self, topic):
        publisher = rospy.Publisher(topic, AsebaEvent, queue_size=1)

        def callback(msg):
            r = int(msg.r * 32)
            g = int(msg.g * 32)
            b = int(msg.b * 32)
            aseba_msg = AsebaEvent(rospy.get_rostime(), 0, [r, g, b])
            publisher.publish(aseba_msg)
        return callback

    def on_aseba_comm_rx_event(self, msg):
        self.comm_rx_publisher.publish(Int16(msg.data[0]))

    def on_aseba_remote_event(self, msg):
        self.remote_publisher.publish(Int8(msg.data[1]))

    def on_sound_threshold(self, msg):
        value = msg * 255
        if value < 0:
            value = 1
        if value > 255:
            value = 0
        self.sound_threshold_publisher.publish(
            AsebaEvent(rospy.get_rostime(), 0, [value]))

    def on_aseba_sound_event(self, msg):
        self.sound_publisher.publish(Float32(msg.data[0] / 255.0))

    def on_aseba_tap_event(self, msg):
        self.tap_publisher.publish(Empty())

    def on_aseba_temperature_event(self, msg):
        self.temperature.temperature = msg.data[0] / 10.0
        self.temperature_publisher.publish(self.temperature)

# TODO check how it's implemented in the firmware.

    def on_aseba_accelerometer_event(self, msg):
        self.imu.linear_acceleration.x = msg.data[1] / 23.0 * 9.81
        self.imu.linear_acceleration.y = -msg.data[0] / 23.0 * 9.81
        self.imu.linear_acceleration.z = msg.data[2] / 23.0 * 9.81
        self.imu.header.stamp = rospy.Time.now()
        self.imu_publisher.publish(self.imu)

    def on_aseba_ground_event(self, msg):
        data = msg.data
        ir_threshold = rospy.get_param("~ground_threshold", 200)

        for sensor, value in zip(self.ground_sensors, data):
            sensor['msg'].range = float('inf') if (
                value < ir_threshold) else -float('inf')
            sensor['msg'].header.stamp = rospy.Time.now()
            sensor['publisher'].publish(sensor['msg'])

    # basics logarithmic fit
    @staticmethod
    def proximity2range(raw):
        if raw > 4000:
            return -float('inf')
        if raw < 800:
            return float('inf')
        return -0.0736 * log(raw) + 0.632

    def on_aseba_proximity_event(self, msg):
        data = msg.data
        values = [self.proximity2range(d) for d in data]
        for sensor, value in zip(self.proximity_sensors, values):
            sensor['msg'].range = value
            sensor['msg'].header.stamp = rospy.Time.now()
            sensor['publisher'].publish(sensor['msg'])

        self.proximityToLaser.ranges = []
        self.proximityToLaser.intensities = []
        self.proximityToLaser.header.stamp = rospy.Time.now()
        for dist, raw in zip(values, data)[4::-1]:
            if dist > 0.14:
                dist = 0.14
            if dist < 0.0215:
                dist = 0.0215
            self.proximityToLaser.ranges.append(dist + 0.08)
            self.proximityToLaser.intensities.append(raw)
        self.proximityToLaserPublisher.publish(self.proximityToLaser)

    def on_aseba_button_event(self, topic):
        publisher = rospy.Publisher(topic, Bool, queue_size=1)

        def callback(msg):
            bool_msg = Bool(msg.data[0])
            publisher.publish(bool_msg)
        return callback

    # ======== we send the speed to the aseba running on the robot  ========
    def set_speed(self, values):
        self.aseba_pub.publish(AsebaEvent(rospy.get_rostime(), 0, values))

    # ======== stop the robot safely ========
    def shutdown(self):
        self.set_speed([0, 0])

    def on_aseba_buttons_event(self, data):
        self.buttons.header.stamp = rospy.Time.now()
        self.buttons.buttons = data.data
        self.buttons_pub.publish(self.buttons)

    # ======== processing odometry events received from the robot ========
    def on_aseba_odometry_event(self, data):
        now = data.stamp
        dt = (now - self.then).to_sec()
        self.then = now

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
        if self.odom_min_period > (now - self.odom_msg.header.stamp).to_sec():
            return

        self.wheel_state_msg.header.stamp = rospy.Time.now()
        self.wheel_state_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.wheel_state_msg.velocity = [left_wheel_angular_speed, right_wheel_angular_speed]
        self.wheel_state_pub.publish(self.wheel_state_msg)

        # prepare tf from base_link to odom
        quaternion = Quaternion()
        quaternion.z = sin(self.th / 2.0)
        quaternion.w = cos(self.th / 2.0)

        # prepare odometry
        self.odom_msg.header.stamp = rospy.Time.now()  # OR TO TAKE ONE FROM THE EVENT?
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation = quaternion

        if(dt > 0):
            self.odom_msg.twist.twist.linear.x = ds / dt
            self.odom_msg.twist.twist.angular.z = dth / dt

        # publish odometry
        if self.odom_broadcaster:
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                self.then, self.robot_frame, self.odom_frame)
        self.odom_pub.publish(self.odom_msg)

    def set_linear_angular_speed(self, speed, ang_speed):
        left_wheel_speed = speed - ang_speed * 0.5 * self.axis
        right_wheel_speed = speed + ang_speed * 0.5 * self.axis

        left_motor_speed = round(self.left_wheel_motor_speed(left_wheel_speed))
        right_motor_speed = round(self.right_wheel_motor_speed(right_wheel_speed))
        max_motor_speed = max(abs(left_motor_speed), abs(right_motor_speed))
        if max_motor_speed > MAX_SPEED:
            return self.set_linear_angular_speed(speed * MAX_SPEED / max_motor_speed,
                                                 ang_speed * MAX_SPEED / max_motor_speed)

        self.set_speed([left_motor_speed, right_motor_speed])

    def on_cmd_vel(self, data):
        self.set_linear_angular_speed(data.linear.x, data.angular.z)

    def __delete__(self):
        # TODO: remove all ros subscribers, publishers, and services
        pass


class ThymioManager:

    def __init__(self):
        rospy.init_node('thymio_manager')
        rospy.wait_for_service('aseba/get_node_list')
        self.get_aseba_nodes = rospy.ServiceProxy('aseba/get_node_list', GetNodeList)
        rospy.wait_for_service('aseba/load_script')
        self.load_script = rospy.ServiceProxy('aseba/load_script', LoadScripts)
        self.load_script_to_target = rospy.ServiceProxy(
            'aseba/load_script_to_target', LoadScriptToTarget)

        default_script_path = os.path.join(roslib.packages.get_pkg_dir('thymio_driver'),
                                           'aseba/multi_thymio_ros.aesl')
        self.script_path = rospy.get_param('~script', default_script_path)

        rospy.sleep(5)

        rospy.loginfo("waiting")

        self.thymios = {}
        self.loaded_script = False

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        rospy.Timer(rospy.Duration(1), self.update_aseba_network)

        self.model_pc = {}
        rospy.spin()

    def update_aseba_network(self, evt):
        nodes = self.get_aseba_nodes().nodeList

        if nodes and not self.loaded_script:
            rospy.loginfo("Load aseba script %s", self.script_path)
            self.load_script(self.script_path)
            self.loaded_script = True
        for node in nodes:
            if 'thymio-II' in node.name and node.id not in self.thymios:
                rospy.loginfo('Connected Thymio with ID %d', node.id)
                try:
                    self.load_script_to_target(node.id)
                except rospy.ServiceException:
                    rospy.logwarn("Cound not load aseba script %s to %d", self.script_path, node.id)
                    continue
                rospy.loginfo("Loaded aseba script %s to %d", self.script_path, node.id)
                name = rospy.get_param('aseba/names/%d' % node.id, 'id_%s' % node.id)
                rospy.loginfo("Create driver for Thymio %s at index %d", name, node.id)
                self.thymios[node.id] = ThymioDriver(node_name=name)
                # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
                #     ['thymio_description', 'model.launch'])
                # p = (roslaunch_file, ['name:=thymio%d' % node.id])
                # print(p)
                # launch = roslaunch.parent.ROSLaunchParent(self.uuid, p)
                # launch.start()
                self.model_pc[node.id] = subprocess.Popen(
                    ['roslaunch', 'thymio_description', 'model.launch',
                     'name:=%s' % name])

        indices = [node.id for node in nodes]
        print(indices)
        for i, _ in list(self.thymios.items()):
            if i not in indices:
                rospy.loginfo("Delete Thymio %d", i)
                del self.thymios[i]
                if i in self.model_pc:
                    self.model_pc[i].kill()


if __name__ == '__main__':
    ThymioManager()
