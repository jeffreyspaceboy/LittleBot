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
        GPIO.add_event_detect(self.gpio_enc_a, GPIO.BOTH)
        GPIO.add_event_callback(self.gpio_enc_a, self.refresh_encoder_callback)
        GPIO.add_event_detect(self.gpio_enc_b, GPIO.BOTH)
        GPIO.add_event_callback(self.gpio_enc_b, self.refresh_encoder_callback)

        self.enc_count = 0
        self.enc_state = GPIO.input(self.gpio_enc_a)
        self.enc_last_state = self.enc_state

        #  get a handle to PWM
        self.frequency = frequency
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

    def refresh_encoder_callback(self):
        self.enc_state = GPIO.input(self.gpio_enc_a)
        if(self.enc_state != self.enc_last_state):
            if(GPIO.input(self.gpio_enc_b) != self.enc_state):
                self.enc_count = self.enc_count + 1
            else:
                self.enc_count = self.enc_count - 1
        self.enc_last_state = self.enc_state

class PID_Controller:
    def __init__(self, Kp, Ki, Kd, Dt):
        self.set_constants(Kp, Ki, Kd, Dt)
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.velocity = 0.0
        
    def set_constants(self, Kp, Ki, Kd, Dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Dt = Dt

    def base_move_loop(self, setpoint, max_velocity, timeout_sec, subsystem):
        move_complete = False
        times_good = 0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0
        subsystem.pid_stop_func()
        time.sleep(0.05)
        subsystem.sensor_reset()
        start_time = time.time()
        while(not move_complete and subsystem.get_sensor_value() <= setpoint):
            self.error = setpoint - subsystem.get_sensor_value()
            self.integral = self.integral + self.error
            if(self.error <= 0.0 or subsystem.get_sensor_value() > setpoint):
                self.integral = 0.0
            if(self.error > setpoint):
                self.integral = 0.0
            self.derivative = self.error - self.prev_error
            self.prev_error = self.error
            self.velocity = ((self.error * self.Kp) + (self.integral * self.Ki) + (self.derivative * self.Kd))
            time.sleep(self.Dt)
            if(self.velocity > max_velocity):
                self.velocity = max_velocity
            subsystem.pid_move_func(self.velocity)
            if(self.error <= 100.0):
                times_good += 1
            if(times_good >= 100):
                move_complete = True
            time.sleep(0.001); 
            if((time.time() - start_time) > timeout_sec):
                return False
        subsystem.pid_stop_func()
        return True


class Drivetrain:
    def __init__(self):
        self.motor_left = Motor(gpio_speed = 2, gpio_dir_1 = 4, gpio_dir_2 = 3, gpio_enc_a = 27, gpio_enc_b = 17, frequency=20, max_speed=255)
        self.motor_right = Motor(gpio_speed = 13, gpio_dir_1 = 6, gpio_dir_2 = 5, gpio_enc_a = 26, gpio_enc_b = 19, frequency=20, max_speed=255)
    
    def drive_foot_time(self):
        self.motor_left.spin(255)
        self.motor_right.spin(255)
        time.sleep(0.859) #Should drive 1 foot
        self.motor_left.stop()
        self.motor_right.stop()

    def drive_foot_enc(self):
        self.motor_left.spin(255)
        self.motor_right.spin(255)
        while(self.motor_left.enc_count < 1718):
            print(self.motor_left.enc_count)
        self.motor_left.stop()
        self.motor_right.stop()

    def pid_stop_func(self):
        self.motor_left.stop()
        self.motor_right.stop()

    def sensor_reset(self):


    def pid_move_func(self, velocity):
        



def main():
    test_drivetrain = Drivetrain()
    test_drivetrain.drive_foot_enc()

if __name__ == '__main__':
    main()