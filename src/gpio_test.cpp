// #include <pigpio.h>
// #include <stdio.h>

// #define LEFT_MOTOR_SPEED 2
// #define LEFT_MOTOR_CW 4
// #define LEFT_MOTOR_CCW 3
// #define LEFT_ENCODER_A 27
// #define LEFT_ENCODER_B 17

// #define RIGHT_MOTOR_SPEED 13
// #define RIGHT_MOTOR_CW 6
// #define RIGHT_MOTOR_CCW 5
// #define RIGHT_ENCODER_A 26
// #define RIGHT_ENCODER_B 19


// #define WHEEL_RADIUS_IN 1.334
// #define WHEEL_RADIUS_MM 33.885
// #define PI 3.14159265359
// #define ENCODER_PER_REVOLUTION 11

// #define ON 255
// #define OFF 0

// #define HIGH 1
// #define LOW 0

// void motors_off(){
// 	gpioWrite(LEFT_MOTOR_SPEED, OFF);
// 	gpioWrite(LEFT_MOTOR_CW,    LOW);
// 	gpioWrite(LEFT_MOTOR_CCW,   LOW);

// 	gpioWrite(RIGHT_MOTOR_SPEED, OFF);
// 	gpioWrite(RIGHT_MOTOR_CW,    LOW);
// 	gpioWrite(RIGHT_MOTOR_CCW,   LOW);
// }

// int main(){
// 	printf("Initializing\n");
// 	gpioInitialise();
// 	gpioSetMode(LEFT_MOTOR_SPEED, PI_OUTPUT);
// 	gpioSetMode(LEFT_MOTOR_CW,    PI_OUTPUT);
// 	gpioSetMode(LEFT_MOTOR_CCW,   PI_OUTPUT);
// 	gpioSetMode(LEFT_ENCODER_A,   PI_INPUT);
// 	gpioSetMode(LEFT_ENCODER_B,   PI_INPUT);
	
// 	gpioSetMode(RIGHT_MOTOR_SPEED, PI_OUTPUT);
// 	gpioSetMode(RIGHT_MOTOR_CW,    PI_OUTPUT);
// 	gpioSetMode(RIGHT_MOTOR_CCW,   PI_OUTPUT);
// 	gpioSetMode(RIGHT_ENCODER_A,   PI_INPUT);
// 	gpioSetMode(RIGHT_ENCODER_B,   PI_INPUT);
// 	motors_off();
	
// 	int counter = 0;
//     int l_state = gpioRead(LEFT_ENCODER_A);
//     int last_l_state;
// 	while(true){
// 		l_state = gpioRead(LEFT_ENCODER_A);
//         if(l_state != last_l_state){
//             if(gpioRead(LEFT_ENCODER_B) != l_state){
//                 counter++;
//             }else{
//                 counter--;
//             }
//             printf("Left: %d\n",counter);
//         }
//         last_l_state = l_state;
// 	}
// 	motors_off();
// 	gpioTerminate();
// 	return 0;
// }


// // #include <pigpio.h>
// // #include <stdio.h>

// // int main(){
// // 	printf("Initializing\n");
// // 	gpioInitialise();
// // 	gpioSetMode(17,PI_OUTPUT);
// // 	gpioSetMode(18,PI_OUTPUT);
// // 	gpioSetMode(27,PI_OUTPUT);
// // 	for(int i=0; i<10; i++){
// // 		gpioWrite(17, 0);
// // 		gpioWrite(18, 1);
// // 		gpioPWM(27, 255);
// // 		gpioDelay(100000);
// // 	}
// // 	gpioWrite(17, 0);
// //         gpioWrite(18, 0);
// //         gpioWrite(27, 0);

// // 	gpioTerminate();
// // 	return 0;
// // }
