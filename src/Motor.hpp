// /*---MOTOR_CPP---*/
// /*----------------------------------------------------------------------------*/
// /*    Module:       Motor.cpp                                                 */
// /*    Author:       Jeffrey Fisher II                                         */
// /*    Created:      2020-07-20                                                */
// /*----------------------------------------------------------------------------*/
// #include <stdint.h>
// #include <pigpio.h>

// class Motor{
//     private:
//         uint8_t gpio_speed, gpio_dir_1, gpio_dir_2, gpio_enc_a, gpio_enc_b;
//         double max_speed = 255.0;
//     public: 
//         Motor(uint8_t gpio_speed, uint8_t gpio_dir_1, uint8_t gpio_dir_2, uint8_t gpio_enc_a, uint8_t gpio_enc_b){
//             this->gpio_speed = gpio_speed;
//             this->gpio_dir_1 = gpio_dir_1;
//             this->gpio_dir_2 = gpio_dir_2;
//             gpioSetMode(this->gpio_speed, PI_OUTPUT);
//             gpioSetMode(this->gpio_dir_1, PI_OUTPUT);
//             gpioSetMode(this->gpio_dir_2, PI_OUTPUT);
//             gpioWrite(this->gpio_speed, 0);
//             gpioWrite(this->gpio_dir_1, 0);
//             gpioWrite(this->gpio_dir_2, 0);

//             this->gpio_enc_a = gpio_enc_a;
//             this->gpio_enc_b = gpio_enc_b;
//             gpioSetMode(this->gpio_enc_a, PI_INPUT);
//             gpioSetMode(this->gpio_enc_b, PI_INPUT);
//         }

//         void spin(double velocity){
//             if(velocity > this->max_speed){
//                 velocity = this->max_speed;
//             }
//             if(velocity < -this->max_speed){
//                 velocity = -this->max_speed;
//             }
//             if(velocity > 0.0){
//                 gpioWrite(this->gpio_dir_1, 1);
//                 gpioWrite(this->gpio_dir_2, 0);
//                 gpioPWM(this->gpio_speed, velocity);
//             }else if(velocity < 0.0){
//                 gpioWrite(this->gpio_dir_1, 0);
//                 gpioWrite(this->gpio_dir_2, 1);
//                 gpioPWM(this->gpio_speed, velocity);
//             }else{
//                 gpioWrite(this->gpio_dir_1, 0);
//                 gpioWrite(this->gpio_dir_2, 0);
//                 gpioWrite(this->gpio_speed, 0);
//             }

//         }

//         void stop(){
//             this->spin(0);
//         }
        
//         void set_max_speed(double new_max_speed){
//             this->max_speed = new_max_speed;
//         }
// };