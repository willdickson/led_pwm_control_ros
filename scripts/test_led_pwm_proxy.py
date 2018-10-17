from __future__ import print_function
import time
from led_pwm_proxy import LedPwmProxy

led_pwm = LedPwmProxy()
pin = 3

for i in range(500):

    if i%2==0:
        value = 0
    else:
        value = 255
    print(i, value)
    led_pwm.set_value(pin,value)
    time.sleep(0.1)

led_pwm.set_value(pin,0)





