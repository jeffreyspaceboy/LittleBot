#! /usr/bin/env python3

import RPi.GPIO as GPIO #Raspberry Pi
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Motor:
    def __init__ (self, gpio_speed, gpio_dir_1, gpio_dir_2, gpio_enc_a, gpio_enc_b, frequency=20, max_speed=255):
        self.gpio_speed = gpio_speed
        self.gpio_dir_1 = gpio_dir_1
        self.gpio_dir_2 = gpio_dir_2
        self.gpio_enc_a = gpio_enc_a
        self.gpio_enc_b = gpio_enc_b

        #  Configure GPIO
        GPIO.setup(self.gpio_speed,  GPIO.OUT)
        GPIO.setup(self.gpio_dir_1, GPIO.OUT)
        GPIO.setup(self.gpio_dir_2, GPIO.OUT)
        
        GPIO.setup(self.gpio_enc_a, GPIO.IN)
        GPIO.setup(self.gpio_enc_b, GPIO.IN)

        #  get a handle to PWM
        self.frequency = frequency
        self.max_speed = max_speed
        self.pwm_speed = GPIO.PWM(self.gpio_speed, self.frequency)
        self.stop()

    def spin (self, velocity):
        if(velocity > self.max_speed):
            velocity = self.max_speed
        if(velocity < -self.max_speed):
            velocity = -self.max_speed

        if(velocity > 0):
            GPIO.output(self.gpio_dir_1, GPIO.HIGH)
            GPIO.output(self.gpio_dir_2, GPIO.LOW)
            self.pwm_speed.start(velocity)
        elif(velocity < 0):
            GPIO.output(self.gpio_dir_1, GPIO.LOW)
            GPIO.output(self.gpio_dir_2, GPIO.HIGH)
            self.pwm_speed.start(velocity)
        else:
            self.stop()

    def stop (self):
        GPIO.output(self.gpio_dir_1, GPIO.LOW)
        GPIO.output(self.gpio_dir_2, GPIO.LOW)
        self.pwm_speed.stop()

def main():
    left_motor = Motor(gpio_speed = 2, gpio_dir_1 = 4, gpio_dir_2 = 3, gpio_enc_a = 27, gpio_enc_b = 17, frequency=20, max_speed=255)
    right_motor = Motor(gpio_speed = 13, gpio_dir_1 = 6, gpio_dir_2 = 5, gpio_enc_a = 26, gpio_enc_b = 19, frequency=20, max_speed=255)

    left_motor.spin(255)
    right_motor.spin(255)
    time.sleep(0.859) #Should drive 1 foot
    left_motor.stop()
    right_motor.stop()

if __name__ == '__main__':
    main()