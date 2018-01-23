#!/usr/bin/env python

from math import cos, log, sin

import roslib
import rospy
import std_srvs.srv

from asebaros_msgs.msg import AsebaEvent
from asebaros_msgs.srv import GetNodeList, LoadScripts
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy, LaserScan, Range, Temperature
from std_msgs.msg import Bool, ColorRGBA, Empty, Float32, Int16, Int8
from tf.broadcaster import TransformBroadcaster
from thymio_msgs.msg import Led, LedGesture, Sound, SystemSound

BASE_WIDTH = 95     # millimeters
MAX_SPEED = 500     # units
SPEED_COEF = 2.93   # 1mm/sec corresponds to X units of real thymio speed

BUTTONS = ['backward', 'forward', 'center', 'right', 'left']
PROXIMITY_NAMES = ['left', 'center_left', 'center',
                   'center_right', 'right', 'rear_left', 'rear_right']
GROUND_NAMES = ['left', 'right']
BODY_LEDS = ['bottom_left', 'bottom_right', 'top']
LED_NUMBER = {Led.CIRCLE: 8, Led.PROXIMITY: 8, Led.GROUND: 2,
              Led.REMOTE: 1, Led.BUTTONS: 4, Led.TEMPERATURE: 2, Led.MICROPHONE: 1}


class ThymioDriver():
    # ======== class initialization ========

    def __init__(self):
        rospy.init_node('thymio')
        # initialize parameters
        self.x = 0
        self.y = 0
        self.th = 0
        self.then = rospy.Time.now()
        self.odom = Odometry(header=rospy.Header(
            frame_id='odom'), child_frame_id='base_link')

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
        script_filename = roslib.packages.get_pkg_dir(
            'thymio_driver') + '/aseba/thymio_ros.aesl'

        load_script(script_filename)

        # subscribe to topics

        self.aseba_pub = rospy.Publisher(
            'aseba/events/set_speed', AsebaEvent, queue_size=1)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.odom_broadcaster = TransformBroadcaster()
        rospy.Subscriber('aseba/events/odometry', AsebaEvent,
                         self.on_aseba_odometry_event)
        rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel)

        self.buttons = Joy()
        self.buttons_pub = rospy.Publisher('buttons', Joy, queue_size=1)
        rospy.Subscriber("aseba/events/buttons", AsebaEvent,
                         self.on_aseba_buttons_event)

        for button in BUTTONS:
            rospy.Subscriber('aseba/events/button_' + button,
                             AsebaEvent, self.on_aseba_button_event(button))

        self.proximity_sensors = [{
            'publisher': rospy.Publisher('proximity/' + name, Range, queue_size=1),
            'msg': Range(header=rospy.Header(frame_id='proximity_' + name + "_link"),
                         radiation_type=Range.INFRARED,
                         field_of_view=0.3,
                         min_range=0.0215,
                         max_range=0.14)} for name in PROXIMITY_NAMES]

        self.proximityToLaserPublisher = rospy.Publisher(
            'proximity/laser', LaserScan, queue_size=1)
        self.proximityToLaser = LaserScan(header=rospy.Header(frame_id="base_link"),
                                          angle_min=-0.64, angle_max=0.64, angle_increment=0.32,
                                          time_increment=0, scan_time=0, range_min=0.0215 + 0.08,
                                          range_max=0.14 + 0.08)
        rospy.Subscriber('aseba/events/proximity', AsebaEvent,
                         self.on_aseba_proximity_event)

        self.ground_sensors = [{
            'publisher': rospy.Publisher('ground/' + name, Range, queue_size=1),
            'msg': Range(header=rospy.Header(frame_id='ground_' + name + "_link"),
                         radiation_type=Range.INFRARED, field_of_view=0.3,
                         min_range=0.008, max_range=0.008)
        } for name in GROUND_NAMES]

        rospy.Subscriber('aseba/events/ground', AsebaEvent,
                         self.on_aseba_ground_event)
        rospy.set_param("~ground_threshold", 200)

        self.imu = Imu(header=rospy.Header(frame_id='base_link'))
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
        rospy.Subscriber('aseba/events/tap', AsebaEvent,
                         self.on_aseba_tap_event)

        self.temperature = Temperature(
            header=rospy.Header(frame_id='base_link'))
        self.temperature.variance = 0.01
        self.temperature_publisher = rospy.Publisher(
            'temperature', Temperature, queue_size=1)
        rospy.Subscriber('aseba/events/temperature', AsebaEvent,
                         self.on_aseba_temperature_event)

        self.sound_publisher = rospy.Publisher('sound', Float32, queue_size=1)
        self.sound_threshold_publisher = rospy.Publisher(
            'aseba/events/set_sound_threshold', AsebaEvent, queue_size=1)
        rospy.Subscriber('aseba/events/sound', AsebaEvent,
                         self.on_aseba_sound_event)
        rospy.Subscriber('sound_threshold', Float32, self.on_sound_threshold)

        self.remote_publisher = rospy.Publisher('remote', Int8, queue_size=1)
        rospy.Subscriber('aseba/events/remote', AsebaEvent,
                         self.on_aseba_remote_event)

        rospy.Subscriber('comm/transmit', Int16, self.on_sound_threshold)
        self.comm_publisher = rospy.Publisher(
            'comm/receive', Int16, queue_size=1)
        self.aseba_set_comm_publisher = rospy.Publisher(
            'aseba/events/set_comm', AsebaEvent, queue_size=1)
        rospy.Subscriber('aseba/events/comm', AsebaEvent,
                         self.on_aseba_comm_event)

        # actuators

        for name in BODY_LEDS:
            rospy.Subscriber('led/body/' + name, ColorRGBA,
                             self.on_body_led(name))

        rospy.Subscriber('led', Led, self.on_led)
        self.aseba_led_publisher = rospy.Publisher(
            'aseba/events/set_led', AsebaEvent, queue_size=6)

        rospy.Subscriber('led/off', Empty, self.on_led_off)

        rospy.Subscriber('led/gesture', LedGesture, self.on_led_gesture)
        self.aseba_led_gesture_publisher = rospy.Publisher(
            'aseba/events/set_led_gesture', AsebaEvent, queue_size=6)
        rospy.Subscriber('led/gesture/circle', Float32,
                         self.on_led_gesture_circle)
        rospy.Subscriber('led/gesture/off', Empty, self.on_led_gesture_off)
        rospy.Subscriber('led/gesture/blink', Float32,
                         self.on_led_gesture_blink)
        rospy.Subscriber('led/gesture/kit', Float32, self.on_led_gesture_kit)
        rospy.Subscriber('led/gesture/alive', Empty, self.on_led_gesture_alive)

        rospy.Subscriber('sound/play', Sound, self.on_sound_play)
        self.aseba_led_gesture_publisher = rospy.Publisher(
            'aseba/events/set_led_gesture', AsebaEvent, queue_size=6)
        rospy.Subscriber('sound/play/system', SystemSound,
                         self.on_system_sound_play)
        self.aseba_play_sound_publisher = rospy.Publisher(
            'aseba/events/play_sound', AsebaEvent, queue_size=1)
        self.aseba_play_system_sound_publisher = rospy.Publisher(
            'aseba/events/play_system_sound', AsebaEvent, queue_size=1)

        rospy.Subscriber('alarm', Bool, self.on_alarm)
        self.alarm_timer = None

        rospy.Subscriber('shutdown', Empty, self.on_shutdown_msg)
        self.aseba_shutdown_publisher = rospy.Publisher(
            'aseba/events/shutdown', AsebaEvent, queue_size=1)

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
        dsl = (data.data[0] * dt) / SPEED_COEF  # left wheel delta in mm
        dsr = (data.data[1] * dt) / SPEED_COEF  # right wheel delta in mm
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
        self.odom.header.stamp = rospy.Time.now()  # OR TO TAKE ONE FROM THE EVENT?
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation = quaternion

        if(dt > 0):
            self.odom.twist.twist.linear.x = ds / dt
            self.odom.twist.twist.angular.z = dth / dt

        # publish odometry
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            self.then, "base_link", "odom")
        self.odom_pub.publish(self.odom)

    # ======== processing events received from the robot  ========
    def on_cmd_vel(self, data):
        x = data.linear.x * 1000.0  # from meters to millimeters
        x = x * SPEED_COEF  # to thymio units
        th = data.angular.z * (BASE_WIDTH / 2)  # in mm
        th = th * SPEED_COEF  # in thymio units
        k = max(abs(x - th), abs(x + th))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            x = x * MAX_SPEED / k
            th = th * MAX_SPEED / k
        self.set_speed([int(x - th), int(x + th)])

    # ======== ======== ======== ======== ======== ======== ========
    def control_loop(self):
        rospy.on_shutdown(self.shutdown)  # on shutdown hook
        while not rospy.is_shutdown():
            rospy.spin()


def main():
    try:
        robot = ThymioDriver()
        robot.control_loop()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
