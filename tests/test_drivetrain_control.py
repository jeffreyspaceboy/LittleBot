#! /usr/bin/env python3

import RPi.GPIO as GPIO #Raspberry Pi
import math
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Motor:
    def __init__ (self, gpio_speed, gpio_dir_1, gpio_dir_2, gpio_enc_a, gpio_enc_b, rpm=100, max_power=100, enc_count_per_revolution=1200, frequency=20):
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
        GPIO.add_event_detect(self.gpio_enc_a, GPIO.BOTH)
        GPIO.add_event_callback(self.gpio_enc_a, self.refresh_encoder_callback)
        # GPIO.add_event_detect(self.gpio_enc_b, GPIO.BOTH)
        # GPIO.add_event_callback(self.gpio_enc_b, self.refresh_encoder_callback)

        self.enc_count = 0
        self.enc_count_per_revolution = enc_count_per_revolution
        self.enc_state = GPIO.input(self.gpio_enc_a)
        self.enc_last_state = self.enc_state

        #  get a handle to PWM
        self.frequency = frequency
        self.rpm = rpm
        self.rps = self.rpm/60
        self.max_speed = max_speed
        self.pwm_speed = GPIO.PWM(self.gpio_speed, self.frequency)
        self.stop()

    def __del__(self):
        self.stop()
        GPIO.remove_event_detect(self.gpio_enc_a)
        GPIO.remove_event_detect(self.gpio_enc_b)

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

    def reset_encoder(self):
        self.enc_count = 0

    def refresh_encoder_callback(self, channel):
        self.enc_state = GPIO.input(self.gpio_enc_a)
        if(self.enc_state != self.enc_last_state):
            if(GPIO.input(self.gpio_enc_b) != self.enc_state):
                self.enc_count = self.enc_count + 1
            else:
                self.enc_count = self.enc_count - 1
        self.enc_last_state = self.enc_state

class Drivetrain:
    def __init__(self, left_motor=Motor(2,4,3,27,17,100,100.0,1200,20), right_motor=Motor(13,6,5,26,19,100,100.0,1200,20), wheel_diameter=0.06777, wheel_base=0.2):
        self.motor_left = left_motor
        self.motor_right = right_motor
        #Distance is in Meters
        self.wheel_diameter = wheel_diameter # meter
        self.wheel_base = wheel_base # meter #TODO: MEASURE 
        self.max_speed_left = self.motor_left.max_speed # duty cycle
        self.max_speed_right = self.motor_right.max_speed #duty cycle
        
        self.wheel_distance_per_rotation = self.wheel_diameter * math.pi # meter/rotation
        self.top_speed_left = self.wheel_distance_per_rotation*self.motor_left.rps # meter/second
        self.top_speed_right = self.wheel_distance_per_rotation*self.motor_right.rps # meter/second

    def drive_foot_enc(self):
        self.motor_left.spin(100)
        self.motor_right.spin(100)
        while(self.motor_left.enc_count < 1718):
            print(self.motor_left.enc_count)
        self.motor_left.stop()
        self.motor_right.stop()

def main():
    test_drivetrain = Drivetrain()
    test_drivetrain.drive_foot_enc()

if __name__ == '__main__':
    main()