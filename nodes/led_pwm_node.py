#!/usr/bin/env python
from __future__ import print_function
import threading
import roslib
import rospy
import std_msgs.msg

from led_pwm_control import LEDController 
from led_pwm_control_ros.msg import LedPwmInfo
from led_pwm_control_ros.srv import LedPwm
from led_pwm_control_ros.srv import LedPwmResponse


class LedPwmNode(object):

    AllowedPins = LEDController.AllowedPins 
    MinValue = LEDController.MinPwmValue  
    MaxValue = LEDController.MaxPwmValue 

    def __init__(self):

        self.dev = LEDController('/dev/ttyUSB0')
        self.lock = threading.Lock()

        rospy.init_node('led_pwm')
        self.led_pwm_srv = rospy.Service('led_pwm', LedPwm, self.led_pwm_srv_callback)
        self.led_pwm_info_pub = rospy.Publisher('led_pwm_info', LedPwmInfo, queue_size=10) 

    def led_pwm_srv_callback(self, req):
        ok = True
        message = ''
        if not req.pin in self.AllowedPins:
            ok = False
            message = 'invalid pin number {}'.format(req.pin)
        if ok and req.value < self.MinValue:
            ok = False
            message = 'pwm value, {} < min allowed {}'.format(req.value,self.MinValue)
        if ok and req.value > self.MaxValue:
            ok = False
            message = 'pwm value, {} > max allowed {}'.format(req.value,self.MaxValue)

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        self.led_pwm_info_pub.publish(LedPwmInfo(header,ok,message,req.pin,req.value))
        self.set_led_pwm(req.pin,req.value)
        return LedPwmResponse(ok,message)

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
