import Jetson.GPIO as GPIO
import time

led_pin = 7

GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)


try:
    while True:
        GPIO.output(led_pin, GPIO.HIGH)
        time.sleep(1.0)
        GPIO.output(led_pin, GPIO.LOW)
        time.sleep(1.0)
        print("Toggling ping {}".format(led_pin))
except KeyboardInterrupt:
    print("Keyboard Interrupt Received: Closing")
    GPIO.cleanup()

