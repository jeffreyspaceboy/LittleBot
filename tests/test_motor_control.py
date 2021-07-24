#! /usr/bin/env python3

import RPi.GPIO as GPIO #Raspberry Pi
import time
import threading

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#NOTES:
# Encoder and motor should only deal with units that they already have (Drivetrain can worry about distance)
# Encoder should be able to produce RPM, RPS, Revolutions, Encoder Counts
 
# Motor should be able to produce calculated RPM, RPS, Revolutions (Of motor output (must calculate))
# Motor should be able to run at a specific RPM/RPS on command
# Motor should be able to run at a specific duty cycle on command
# Motor should be able to stop without over shooting 
# Motor should be able to use PID to reach a desired target
# Motor calibration:
# 1. Spin Motor at max speed (in air)
# 2. Get RPM of max speed spin and save as RPM Value
# 3. Try To achieve other specific RPM values and record the duty cycle of those values for use later

#Once Motor is calebrated, we need to do drivetrain calibration:
# 1. Drive forward until Lidar registers x distance of change.
# 2. Drive backwards to measure the same change
# 3. Put something close to the robot (like a pole of somekind)
# 4. Turn the robot in a stationary circle until the lidar registers the object back at the front of the robot
# 5. repeat 4 in both directions
 



# Drivetrain should be able to drive in mm/cm/m/in/ft
# Drivetrain should be able to return it's position relative to where it started
# Drivetrain should be able to drive straight


class Encoder:
    def __init__(self, gpio_a, gpio_b):
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b

        GPIO.setup(self.gpio_a, GPIO.IN)
        GPIO.setup(self.gpio_b, GPIO.IN)

        self.count = 0.0
        
        GPIO.add_event_detect(self.gpio_a, GPIO.BOTH)
        GPIO.add_event_callback(self.gpio_a, self.gpio_a_event_callback)
        GPIO.add_event_detect(self.gpio_b, GPIO.BOTH)
        GPIO.add_event_callback(self.gpio_b, self.gpio_b_event_callback)

        self.rps = 0.0
        self.rps_last_count = 0.0
        self.rps_timer = time.time()

        self.refresh_rps()

    def reset(self):
        self.count = 0.0

    def refresh_rps(self):
        self.rps = (self.count-self.rps_last_count)/(time.time()-self.rps_timer)
        self.rps_last_count = self.count
        self.rps_timer = time.time() 
        threading.Timer(0.05, refresh_rps,self).start()

    def gpio_a_event_callback(self, channel):
        if(GPIO.input(self.gpio_b) == 0):
            if(GPIO.input(self.gpio_a) == 0):
                self.count = self.count - 1.0
            else:
                self.count = self.count + 1.0
        else:
            if(GPIO.input(self.gpio_a) == 0):
                self.count = self.count + 1.0
            else:
                self.count = self.count - 1.0
    
    def gpio_b_event_callback(self, channel):
        if(GPIO.input(self.gpio_a) == 0):
            if(GPIO.input(self.gpio_b) == 0):
                self.count = self.count + 1.0
            else:
                self.count = self.count - 1.0
        else:
            if(GPIO.input(self.gpio_b) == 0):
                self.count = self.count - 1.0
            else:
                self.count = self.count + 1.0
        
class Motor:
    def __init__(self, gpio_speed, gpio_dir_1, gpio_dir_2, gpio_enc_a, gpio_enc_b, rpm=100.0, max_power=100.0, enc_rev=1200, pwm_frequency=20.0):
        self.rpm = rpm
        self.rps = rpm/60
        self.max_power = max_power
        self.frequency = pwm_frequency
        
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
        GPIO.add_event_detect(self.gpio_enc_b, GPIO.BOTH)
        GPIO.add_event_callback(self.gpio_enc_b, self.refresh_encoder_callback)

        self.enc_count = 0
        self.enc_state = GPIO.input(self.gpio_enc_a)
        self.enc_last_state = self.enc_state

        self.pwm_speed = GPIO.PWM(self.gpio_speed, self.frequency)
        self.stop()

    def __del__(self):
        self.stop()
        GPIO.remove_event_detect(self.gpio_enc_a)
        GPIO.remove_event_detect(self.gpio_enc_b)

    def spin(self, power):
        if(power > self.max_power):
            power = self.max_power
        if(power < -self.max_power):
            power = -self.max_power
        if(power > 0):
            GPIO.output(self.gpio_dir_1, GPIO.HIGH)
            GPIO.output(self.gpio_dir_2, GPIO.LOW)
            self.pwm_speed.start(power)
        elif(power < 0):
            GPIO.output(self.gpio_dir_1, GPIO.LOW)
            GPIO.output(self.gpio_dir_2, GPIO.HIGH)
            self.pwm_speed.start(power)
        else:
            self.stop()

    def stop (self):
        GPIO.output(self.gpio_dir_1, GPIO.LOW)
        GPIO.output(self.gpio_dir_2, GPIO.LOW)
        self.pwm_speed.stop()

    

def main():
    left_motor = Motor(gpio_speed = 2, gpio_dir_1 = 4, gpio_dir_2 = 3, gpio_enc_a = 27, gpio_enc_b = 17, frequency=20, max_speed=100)
    right_motor = Motor(gpio_speed = 13, gpio_dir_1 = 6, gpio_dir_2 = 5, gpio_enc_a = 26, gpio_enc_b = 19, frequency=20, max_speed=100)

    left_motor.spin(100)
    right_motor.spin(100)
    time.sleep(0.859) #Should drive 1 foot
    left_motor.stop()
    right_motor.stop()

if __name__ == '__main__':
    main()