#!/usr/bin/env python
from __future__ import print_function
import threading
import roslib
import rospy
from led_pwm_control import LEDController 

from led_pwm_control_ros.srv import LedPwm
from led_pwm_control_ros.srv import LedPwmResponse


class LedPwmNode(object):

    AllowedPins = [3,5,6,9,10,11]
    MinValue = 0
    MaxValue = 255

    def __init__(self):

        self.dev = LEDController('/dev/ttyUSB0')
        self.lock = threading.Lock()

        rospy.init_node('led_pwm')
        self.led_pwm_srv = rospy.Service('led_pwm', LedPwm, self.led_pwm_srv_callback)

    def led_pwm_srv_callback(self, req):
        if not req.pin in self.AllowedPins:
            return LedPwmResponse(False,'invalid pin number {}'.format(req.pin))
        if req.value < self.MinValue:
            return LedPwmResponse(False,'pwm value, {} < min allowed {}'.format(req.value,self.MinValue))
        if req.value > self.MaxValue:
            return LedPwmResponse(False,'pwm value, {} > max allowed {}'.format(req.value,self.MaxValue))
        self.set_led_pwm(req.pin,req.value)
        return LedPwmResponse(True,"")

    def set_led_pwm(self, pin, value):
        with self.lock:
            self.dev.set_value(pin,value)

    def run(self):
        print('led pwm node running')
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

        for pin in self.AllowedPins:
            self.set_led_pwm(pin,0)


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = LedPwmNode()
    node.run()
