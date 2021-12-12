# LittleBot
LittleBot was developed for a small and simple two wheel robot. The code within this repository is meant to function with ROS2 Foxy, and run on a Raspberry Pi 3/4. 

## Authors
Jeffrey Fisher II and Jeff Fisher

## Usage
### Install:
    pigpio.h
    1. wget https://github.com/joan2937/pigpio/archive/master.zip
    2. unzip master.zip
    3. cd pigpio-master
    4. make
    5. sudo make install
### Install:
    curses.h
    1. sudo apt-get install libncurses5-dev libncursesw5-dev

### Build:
    gcc -Wall -pthread -o little_bot_program main.c src/Drivetrain.c src/PID.c src/Motor.c src Encoder.c -lpigpio -lrt -lncurses -ltinfo

### Run:
    sudo ./little_bot_program

## Notes:
    Motor Max Speed: 201RPM
    Motor Internal Ratio: 21.3 
    Encoder Rotations : 1 Wheel Rotation
    Encoder Output Ticks: 44
    Wheel Radius: 38 mm
    Wheel Diameter: 76 mm 

    Motor Specifications:
    Rated Voltage: 12V
    Voltage Range: 6-24V
    No-load Speed: 201RPM
    Load speed: 168RPM
    No-load Current: 46mA
    Load Current: 300mA
    Stall Current: 1A
    Torque: 0.53kg.cm
    Reduction Ratio: 21.3:1
    Gearbox Size(D*L): 25x19mm/0.98x0.75inch
    Motor Size(D*L): 24.5x30mm/0.96x1.18inch
    Shaft Size(D*L): 4x10mm/0.16x0.39inch
    Encoder Cable Length: 20cm/7.87inch
    Gross Weight: Approx. 100g