#! /usr/bin/env python3

import RPi.GPIO as GPIO #Raspberry Pi
import sched
import time

import threading

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

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
        self.prev_error = setpoint
        #subsystem.pid_stop_func()
        #time.sleep(0.05)
        subsystem.pid_sensor_reset()
        start_time = time.time()
        
        while(not move_complete and subsystem.pid_get_sensor_value() <= setpoint):
            print(subsystem.pid_get_sensor_value(), setpoint)
            self.error = setpoint - subsystem.pid_get_sensor_value()
            self.integral = self.integral + self.error
            if(self.error <= 0.0 or subsystem.pid_get_sensor_value() > setpoint):
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
            if(self.error < 5.0):
                times_good += 1
            if(times_good >= 100):
                move_complete = True
            if((time.time() - start_time) > timeout_sec):
                return False
            print("Error: {}, Integral: {}, Derivative: {}".format(self.error,self.integral,self.derivative))
        print(move_complete)
        subsystem.pid_stop_func()
        return True

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
        self.rpm = 0.0
        self.rps_last_count = 0.0
        self.rps_timer = time.time()

        #self.refresh_scheduler = sched.scheduler(time.time, time.sleep)
        #self.refresh_scheduler.enter(0.01, 1, self.refresh_rps, (self.refresh_scheduler,))
        #self.refresh_scheduler.run()

    def __del__(self):
        GPIO.remove_event_detect(self.gpio_a)
        GPIO.remove_event_detect(self.gpio_b)

    def reset(self):
        self.count = 0.0

    def refresh_rps(self):
        self.rps = (self.count-self.rps_last_count)/(time.time()-self.rps_timer)
        self.rps_last_count = self.count
        self.rps_timer = time.time() 
        self.rpm = self.rps*60.0
        #self.refresh_scheduler.enter(0.01, 1, self.refresh_rps, (sc,))
        #threading.Timer(0.05, refresh_rps, (self)).start()

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
        if(self.count % 11 == 0):
            self.refresh_rps()
    
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
        if(self.count % 11 == 0):
            self.refresh_rps()

class Motor:
    def __init__(self, gpio_speed, gpio_dir_1, gpio_dir_2, gpio_enc_a, gpio_enc_b, max_power=100.0, enc_rev=2400.0, frequency=20.0):
        self.rpm = rpm
        self.rps = rpm/60.0
        self.max_power = max_power
        self.frequency = frequency
        
        self.gpio_speed = gpio_speed
        self.gpio_dir_1 = gpio_dir_1
        self.gpio_dir_2 = gpio_dir_2
        self.encoder = Encoder(gpio_enc_a, gpio_enc_b)
        #self.encoder.refresh_rps()
        #  Configure GPIO
        GPIO.setup(self.gpio_speed,  GPIO.OUT)
        GPIO.setup(self.gpio_dir_1, GPIO.OUT)
        GPIO.setup(self.gpio_dir_2, GPIO.OUT)

        self.pwm_speed = GPIO.PWM(self.gpio_speed, self.frequency)
        self.stop()

    def spin(self, power):
        if(power > self.max_power):
            power = self.max_power
        if(power < -self.max_power):
            power = -self.max_power
        if(power > 0.0):
            GPIO.output(self.gpio_dir_1, GPIO.HIGH)
            GPIO.output(self.gpio_dir_2, GPIO.LOW)
            self.pwm_speed.start(power)
        elif(power < 0.0):
            GPIO.output(self.gpio_dir_1, GPIO.LOW)
            GPIO.output(self.gpio_dir_2, GPIO.HIGH)
            self.pwm_speed.start(-power)
        else:
            self.stop()

    def stop (self):
        GPIO.output(self.gpio_dir_1, GPIO.LOW)
        GPIO.output(self.gpio_dir_2, GPIO.LOW)
        self.pwm_speed.stop()
    
    def pid_stop_func(self):
        self.stop()
    def pid_move_func(self, power):
        if(power > self.max_power):
            power = self.max_power
        if(power < -self.max_power):
            power = -self.max_power
        if(power > 0.0):
            self.pwm_speed.start(power)
            GPIO.output(self.gpio_dir_1, GPIO.HIGH)
            GPIO.output(self.gpio_dir_2, GPIO.LOW)
        elif(power < 0.0):
            self.pwm_speed.start(-power)
            GPIO.output(self.gpio_dir_1, GPIO.LOW)
            GPIO.output(self.gpio_dir_2, GPIO.HIGH)
            
def main():
    left_motor = Motor(gpio_speed = 2, gpio_dir_1 = 4, gpio_dir_2 = 3, gpio_enc_a = 27, gpio_enc_b = 17)
    #right_motor = Motor(gpio_speed = 13, gpio_dir_1 = 6, gpio_dir_2 = 5, gpio_enc_a = 19, gpio_enc_b = 26)
    #left_pid_controller = PID_Controller(Kp=0.4, Ki=0.00, Kd=0.0, Dt=0.01)

    #print(left_pid_controller.base_move_loop(setpoint=2400.0, max_velocity=100.0, timeout_sec=10.0, subsystem=left_motor))

    left_motor.spin(50)
    while 1:#(left_motor.encoder.count < 2200):
        print(left_motor.encoder.rpm)
    # left_motor.spin(-100)
    #left_motor.stop()

if __name__ == '__main__':
    main()


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