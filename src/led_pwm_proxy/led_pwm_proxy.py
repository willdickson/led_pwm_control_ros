#!/usr/bin/env python
from __future__ import print_function
import roslib
import rospy

from led_pwm_control_ros.srv import LedPwm


class LedPwmProxyException(Exception):
    pass


class LedPwmProxy(object):

    def __init__(self, namespace=None):
        self.namespace = namespace
        if namespace is None:
            led_pwm_srv_name  = '/led_pwm'.format(self.namespace)
        else:
            led_pwm_srv_name  = '/{}/led_pwm'.format(self.namespace)
        self.led_pwm_proxy = rospy.ServiceProxy(led_pwm_srv_name,LedPwm)

    def set_value(self, pin, value):
        rsp = self.led_pwm_proxy(pin, value)
        return rsp


