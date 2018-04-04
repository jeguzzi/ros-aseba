#!/usr/bin/env python

import csv
import os
from datetime import datetime as dt

import numpy as np

import rospkg
import rospy
import yaml
from asebaros_msgs.msg import AsebaEvent
from scipy.optimize import curve_fit


IN = 0
OFF = 1
UNKNOWN = None

# DONE:
# 1) save sampls to file
# 2) save calibration to file
# 3) load calibration in driver
# TODO:
# 4) (ev.) calibrate the distance too
# 5) print a T (and adapt instructions)


def calibrate(samples):
    def f(x, a, b):
        return a * x + b * x ** 2
    motor_speed = samples[:, 0]
    speed = samples[:, 1]
    popt, pcov = curve_fit(f, motor_speed, speed)
    return popt.tolist()


class Line(object):
    def __init__(self):
        self.state = UNKNOWN
        self.in_ = None


DEFAULT_MOTOR_SPEEDS = [20, 40, 60, 80, 100, 120, 140, 160, 200, 250, 300, 350, 400, 450]
# DEFAULT_MOTOR_SPEEDS = [60, 120, 240]

# def count(n):
#     for t in range(n):
#         rospy.logingo("-%d", n - t)
#         rospy.sleep(1)


class Calibration(object):

    def __init__(self, target_number_of_samples=1, motor='right', angle=(2 * np.pi),
                 sample_folder=None, motor_speeds=DEFAULT_MOTOR_SPEEDS, gap=100,
                 axis_length=0.0935):
        self.line = Line()
        self.pick = None
        self.motor_speed = None
        self._samples = [[0, 0]]
        self.target_number_of_samples = target_number_of_samples
        self.motor = motor
        self.angle = angle
        self.motor_speeds = motor_speeds
        self.sample_folder = sample_folder
        self.gap = gap
        self.axis_length = axis_length
        self.pub = rospy.Publisher('aseba/events/set_speed', AsebaEvent, queue_size=1)
        self.sub = rospy.Subscriber('aseba/events/ground', AsebaEvent, self.update_state,
                                    queue_size=1)

    def run(self):
        standing_wheel = 'right' if self.motor == 'left' else 'left'
        rospy.loginfo('Place the robot with the %s wheel at the center of the T',
                      standing_wheel)
        raw_input('Press a button when the robot is ready ')
        msg = rospy.wait_for_message('aseba/events/ground', AsebaEvent)
        self.ths = [d - self.gap for d in msg.data]
        # count(3)
        rospy.loginfo('Start calibrating %s motor using thresholds %s', self.motor, self.ths)
        _n = 1
        self.pick = None
        for motor_speed in self.motor_speeds:
            rospy.loginfo('Set motor speed to %s', motor_speed)
            _n += self.target_number_of_samples
            self.motor_speed = motor_speed
            if self.motor == 'right':
                data = [0, self.motor_speed]
            else:
                data = [self.motor_speed, 0]

            self.pub.publish(AsebaEvent(data=data))
            while len(self._samples) < _n:
                rospy.sleep(0.05)
        self.pub.publish(AsebaEvent(data=[0, 0]))
        self.sub.unregister()
        rospy.logdebug('Calculate with samples %s', self._samples)
        self.save_samples()
        samples = np.array(self._samples)
        results = calibrate(samples)
        rospy.loginfo('Calibration of motor %s done: %s', self.motor, results)
        self.save_calibration(results)

    def save_samples(self):
        if self.sample_folder:
            path = os.path.join(self.sample_folder, '{}.csv'.format(self.motor))
            with open(path, 'w') as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(['motor_speed', 'speed'])
                writer.writerows(self._samples)

    def save_calibration(self, result, kind='quadratic'):
        path = os.path.abspath(os.path.join(self.sample_folder, '{}.yaml'.format(self.motor)))
        t_path = os.path.abspath(
            os.path.join(self.sample_folder, '..', '{}.yaml'.format(self.motor)))
        with open(path, 'w') as f:
            cal = {'kind': kind, 'q': result}
            yaml.dump(cal, f)
        rospy.loginfo('Create symlink %s -> %s', t_path, path)
        try:
            os.symlink(path, t_path)
        except OSError:
            os.remove(t_path)
            os.symlink(path, t_path)

    def speed(self, period):
        return self.angle / period * self.axis_length

    def add_pick(self, pick):
        if self.pick is not None:
            dt = pick - self.pick
            self._samples.append([self.motor_speed, self.speed(dt)])
            rospy.loginfo('Added sample %.3f -> %s', dt, self._samples[-1])
        self.pick = pick

    def update_state(self, msg):
        if self.motor_speed is None:
            return
        line = self.line
        if self.motor == 'right':
            v = msg.data[0]
            th = self.ths[0]
        else:
            v = msg.data[1]
            th = self.ths[1]
        if v > th and line.state == UNKNOWN:
            line.state = OFF
        if v < th and line.state == OFF:
            line.state = IN
            line.in_ = msg.stamp
        if v > th and line.state == IN:
            rospy.loginfo('Line')
            line.state = OFF
            pick = (msg.stamp.to_sec() + line.in_.to_sec()) / 2
            self.add_pick(pick)


def main():

    rospy.init_node("calibrate")
    folder = rospkg.RosPack().get_path('thymio_driver')
    gap = rospy.get_param('~gap', 100)
    axis_length = rospy.get_param('~axis_length', 0.0935)
    number_of_samples = rospy.get_param('~number_of_samples', 1)
    motor_speeds = rospy.get_param('~motor_speeds', DEFAULT_MOTOR_SPEEDS)
    sample_folder = os.path.join(folder, 'calibration', dt.now().isoformat())
    os.makedirs(sample_folder)
    for motor in rospy.get_param('~motors', ['right', 'left']):
        cal = Calibration(target_number_of_samples=number_of_samples, motor=motor,
                          sample_folder=sample_folder, gap=gap, motor_speeds=motor_speeds,
                          axis_length=axis_length)
        cal.run()
    rospy.loginfo("Motor calibration was successful")


if __name__ == '__main__':
    main()
