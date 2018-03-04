#!/usr/bin/env python

import os
from math import cos, log, sin

import roslib
import rospy
import std_srvs.srv
from asebaros_msgs.msg import AsebaEvent
from asebaros_msgs.srv import GetNodeList, LoadScripts
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, Joy, LaserScan, Range, Temperature
from std_msgs.msg import Bool, ColorRGBA, Empty, Float32, Int8, Int16
from tf.broadcaster import TransformBroadcaster
from thymio_msgs.msg import Led, LedGesture, Sound, SystemSound


BASE_WIDTH = 95.0     # millimeters
MAX_SPEED = 500.0     # units
SPEED_COEF = 2.93   # 1mm/sec corresponds to X units of real thymio speed
WHEEL_RADIUS = 22.0   # millimeters

BUTTONS = ['backward', 'forward', 'center', 'right', 'left']
PROXIMITY_NAMES = ['left', 'center_left', 'center',
                   'center_right', 'right', 'rear_left', 'rear_right']
GROUND_NAMES = ['left', 'right']
BODY_LEDS = ['bottom_left', 'bottom_right', 'top']
LED_NUMBER = {Led.CIRCLE: 8, Led.PROXIMITY: 8, Led.GROUND: 2,
              Led.REMOTE: 1, Led.BUTTONS: 4, Led.TEMPERATURE: 2, Led.MICROPHONE: 1}


class ThymioDriver(object):

    def frame_name(self, name):
        if self.tf_prefix:
            return '{self.tf_prefix}/{name}'.format(**locals())
        return name

    def __init__(self):
        rospy.init_node('thymio')

        # load script on the Thymio

        rospy.wait_for_service('aseba/get_node_list')
        get_aseba_nodes = rospy.ServiceProxy(
            'aseba/get_node_list', GetNodeList)

        while True:
            if 'thymio-II' in get_aseba_nodes().nodeList:
                break
            rospy.loginfo('Waiting for thymio node ...')
            rospy.sleep(1)

        rospy.wait_for_service('aseba/load_script')
        load_script = rospy.ServiceProxy('aseba/load_script', LoadScripts)
        default_script_path = os.path.join(
            roslib.packages.get_pkg_dir('thymio_driver'), 'aseba/thymio_ros.aesl')

        script_path = rospy.get_param('~script', default_script_path)

        rospy.loginfo("Load aseba script %s", script_path)

        load_script(script_path)

        # initialize parameters

        self.tf_prefix = rospy.get_param('tf_prefix', '')

        self.odom_frame = self.frame_name(rospy.get_param('~odom_frame', 'odom'))
        self.robot_frame = self.frame_name(rospy.get_param('~robot_frame', 'base_link'))

        self.x = 0
        self.y = 0
        self.th = 0
        self.then = rospy.Time.now()
        self.odom_msg = Odometry(header=rospy.Header(frame_id=self.odom_frame),
                                 child_frame_id=self.robot_frame)

        # subscribe to topics

        self.aseba_pub = rospy.Publisher(
            'aseba/events/set_speed', AsebaEvent, queue_size=1)

        self.left_wheel_angle = 0
        self.right_wheel_angle = 0

        self.ticks2mm = rospy.get_param('~ticks2mm', 1.0 / SPEED_COEF)
        left_wheel_joint = rospy.get_param('~left_wheel_joint', 'left_wheel_joint')
        right_wheel_joint = rospy.get_param('~right_wheel_joint', 'right_wheel_joint')

        self.wheel_state_msg = JointState()
        self.wheel_state_msg.name = [left_wheel_joint, right_wheel_joint]

        self.wheel_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.odom_broadcaster = TransformBroadcaster()

        rospy.Subscriber('aseba/events/odometry', AsebaEvent, self.on_aseba_odometry_event)
        rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel)

        self.buttons = Joy()
        self.buttons_pub = rospy.Publisher('buttons', Joy, queue_size=1)
        rospy.Subscriber("aseba/events/buttons", AsebaEvent, self.on_aseba_buttons_event)

        for button in BUTTONS:
            rospy.Subscriber('aseba/events/button_' + button,
                             AsebaEvent, self.on_aseba_button_event(button))

        proximity_range_min = rospy.get_param('~proximity/range_min', 0.0215)
        proximity_range_max = rospy.get_param('~proximity/range_min', 0.14)
        proximity_field_of_view = rospy.get_param('~proximity/field_of_view', 0.3)

        self.proximity_sensors = [{
            'publisher': rospy.Publisher('proximity/' + name, Range, queue_size=1),
            'msg': Range(
                header=rospy.Header(
                    frame_id=self.frame_name('proximity_{name}_link'.format(name=name))),
                radiation_type=Range.INFRARED,
                field_of_view=proximity_field_of_view,
                min_range=proximity_range_min,
                max_range=proximity_range_max)}
            for name in PROXIMITY_NAMES]

        self.proximityToLaserPublisher = rospy.Publisher(
            'proximity/laser', LaserScan, queue_size=1)
        self.proximityToLaser = LaserScan(
            header=rospy.Header(frame_id=self.frame_name('laser_link')),
            angle_min=-0.64, angle_max=0.64, angle_increment=0.32,
            time_increment=0, scan_time=0, range_min=proximity_range_min + 0.08,
            range_max=proximity_range_max + 0.08)
        rospy.Subscriber('aseba/events/proximity', AsebaEvent, self.on_aseba_proximity_event)

        self.ground_sensors = [{
            'publisher': rospy.Publisher('ground/' + name, Range, queue_size=1),
            'msg': Range(
                header=rospy.Header(
                    frame_id=self.frame_name('ground_{name}_link'.format(name=name))),
                radiation_type=Range.INFRARED, field_of_view=proximity_field_of_view,
                min_range=0.008, max_range=0.008)
        } for name in GROUND_NAMES]

        rospy.Subscriber('aseba/events/ground', AsebaEvent, self.on_aseba_ground_event)
        rospy.set_param("~ground_threshold", 200)

        self.imu = Imu(header=rospy.Header(frame_id=self.robot_frame))
        # no orientation or angular velocity information
        self.imu.orientation_covariance[0] = -1
        self.imu.angular_velocity_covariance[0] = -1
        # just an accelerometer
        self.imu.linear_acceleration_covariance[0] = 0.07
        self.imu.linear_acceleration_covariance[4] = 0.07
        self.imu.linear_acceleration_covariance[8] = 0.07

        self.imu_publisher = rospy.Publisher('imu', Imu, queue_size=1)
        rospy.Subscriber('aseba/events/accelerometer',
                         AsebaEvent, self.on_aseba_accelerometer_event)

        self.tap_publisher = rospy.Publisher('tap', Empty, queue_size=1)
        rospy.Subscriber('aseba/events/tap', AsebaEvent, self.on_aseba_tap_event)

        self.temperature = Temperature(
            header=rospy.Header(frame_id=self.robot_frame))
        self.temperature.variance = 0.01
        self.temperature_publisher = rospy.Publisher('temperature', Temperature, queue_size=1)
        rospy.Subscriber('aseba/events/temperature', AsebaEvent, self.on_aseba_temperature_event)

        self.sound_publisher = rospy.Publisher('sound', Float32, queue_size=1)
        self.sound_threshold_publisher = rospy.Publisher(
            'aseba/events/set_sound_threshold', AsebaEvent, queue_size=1)
        rospy.Subscriber('aseba/events/sound', AsebaEvent, self.on_aseba_sound_event)
        rospy.Subscriber('sound_threshold', Float32, self.on_sound_threshold)

        self.remote_publisher = rospy.Publisher('remote', Int8, queue_size=1)
        rospy.Subscriber('aseba/events/remote', AsebaEvent, self.on_aseba_remote_event)

        rospy.Subscriber('comm/transmit', Int16, self.on_sound_threshold)
        self.comm_publisher = rospy.Publisher('comm/receive', Int16, queue_size=1)
        self.aseba_set_comm_publisher = rospy.Publisher(
            'aseba/events/set_comm', AsebaEvent, queue_size=1)
        rospy.Subscriber('aseba/events/comm', AsebaEvent, self.on_aseba_comm_event)

        # actuators

        for name in BODY_LEDS:
            rospy.Subscriber('led/body/' + name, ColorRGBA, self.on_body_led(name))

        rospy.Subscriber('led', Led, self.on_led)
        self.aseba_led_publisher = rospy.Publisher(
            'aseba/events/set_led', AsebaEvent, queue_size=6)

        rospy.Subscriber('led/off', Empty, self.on_led_off)

        rospy.Subscriber('led/gesture', LedGesture, self.on_led_gesture)
        self.aseba_led_gesture_publisher = rospy.Publisher(
            'aseba/events/set_led_gesture', AsebaEvent, queue_size=6)
        rospy.Subscriber('led/gesture/circle', Float32, self.on_led_gesture_circle)
        rospy.Subscriber('led/gesture/off', Empty, self.on_led_gesture_off)
        rospy.Subscriber('led/gesture/blink', Float32, self.on_led_gesture_blink)
        rospy.Subscriber('led/gesture/kit', Float32, self.on_led_gesture_kit)
        rospy.Subscriber('led/gesture/alive', Empty, self.on_led_gesture_alive)

        rospy.Subscriber('sound/play', Sound, self.on_sound_play)
        self.aseba_led_gesture_publisher = rospy.Publisher(
            'aseba/events/set_led_gesture', AsebaEvent, queue_size=6)
        rospy.Subscriber('sound/play/system', SystemSound, self.on_system_sound_play)
        self.aseba_play_sound_publisher = rospy.Publisher(
            'aseba/events/play_sound', AsebaEvent, queue_size=1)
        self.aseba_play_system_sound_publisher = rospy.Publisher(
            'aseba/events/play_system_sound', AsebaEvent, queue_size=1)

        rospy.Subscriber('alarm', Bool, self.on_alarm)
        self.alarm_timer = None

        rospy.Subscriber('shutdown', Empty, self.on_shutdown_msg)
        self.aseba_shutdown_publisher = rospy.Publisher(
            'aseba/events/shutdown', AsebaEvent, queue_size=1)

        rospy.on_shutdown(self.shutdown)

        # tell ros that we are ready
        rospy.Service('thymio_is_ready', std_srvs.srv.Empty, self.ready)

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

    def on_body_led(self, name):
        publisher = rospy.Publisher(
            'aseba/events/set_led_' + name, AsebaEvent, queue_size=1)

        def callback(msg):
            r = int(msg.r * 32)
            g = int(msg.g * 32)
            b = int(msg.b * 32)
            aseba_msg = AsebaEvent(rospy.get_rostime(), 0, [r, g, b])
            publisher.publish(aseba_msg)
        return callback

    def on_aseba_comm_event(self, msg):
        self.comm_publisher.publish(Int16(msg.data[0]))

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

    def on_aseba_button_event(self, button):
        publisher = rospy.Publisher('buttons/' + button, Bool, queue_size=1)

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

        vl = data.data[0] * self.ticks2mm
        vr = data.data[1] * self.ticks2mm

        # wheel joint states
        left_wheel_angular_speed = vl / WHEEL_RADIUS
        right_wheel_angular_speed = vr / WHEEL_RADIUS

        self.left_wheel_angle += dt * left_wheel_angular_speed
        self.right_wheel_angle += dt * right_wheel_angular_speed

        self.wheel_state_msg.header.stamp = rospy.Time.now()
        self.wheel_state_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.wheel_state_msg.velocity = [left_wheel_angular_speed, right_wheel_angular_speed]
        self.wheel_state_pub.publish(self.wheel_state_msg)

        dsl = vl * dt  # left wheel delta in mm
        dsr = vr * dt  # right wheel delta in mm

        # robot traveled distance in meters
        ds = ((dsl + dsr) / 2.0) / 1000.0
        dth = (dsr - dsl) / BASE_WIDTH  # turn angle

        self.x += ds * cos(self.th + dth / 2.0)
        self.y += ds * sin(self.th + dth / 2.0)
        self.th += dth

        # prepare tf from base_link to odom
        quaternion = Quaternion()
        quaternion.z = sin(self.th / 2.0)
        quaternion.w = cos(self.th / 2.0)

        # prepare odometry
        self.odom_msg.header.stamp = rospy.Time.now()  # OR TO TAKE ONE FROM THE EVENT?
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0
        self.odom_msg.pose.pose.orientation = quaternion

        if(dt > 0):
            self.odom_msg.twist.twist.linear.x = ds / dt
            self.odom_msg.twist.twist.angular.z = dth / dt

        # publish odometry
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            self.then, self.robot_frame, self.odom_frame)
        self.odom_pub.publish(self.odom_msg)

    # ======== processing events received from the robot  ========
    def on_cmd_vel(self, data):
        x = data.linear.x * 1000.0  # from meters to millimeters
        x = x / self.ticks2mm  # to thymio units
        th = data.angular.z * (BASE_WIDTH / 2)  # in mm
        th = th / self.ticks2mm  # in thymio units
        k = max(abs(x - th), abs(x + th))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            x = x * MAX_SPEED / k
            th = th * MAX_SPEED / k
        self.set_speed([int(x - th), int(x + th)])


if __name__ == '__main__':
    ThymioDriver()
    rospy.spin()
