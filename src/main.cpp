#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <stdint.h>
#include <pigpio.h>

class Motor{
  private:
    uint8_t gpio_speed, gpio_dir_1, gpio_dir_2, gpio_enc_a, gpio_enc_b;
    int max_speed = 255;
  public: 
    Motor(){}
    Motor(uint8_t gpio_speed, uint8_t gpio_dir_1, uint8_t gpio_dir_2, uint8_t gpio_enc_a, uint8_t gpio_enc_b){
      gpio_setup(gpio_speed, gpio_dir_1, gpio_dir_2, gpio_enc_a, gpio_enc_b);
    }

    void gpio_setup(uint8_t gpio_speed, uint8_t gpio_dir_1, uint8_t gpio_dir_2, uint8_t gpio_enc_a, uint8_t gpio_enc_b){
      this->gpio_speed = gpio_speed;
      this->gpio_dir_1 = gpio_dir_1;
      this->gpio_dir_2 = gpio_dir_2;
      gpioSetMode(this->gpio_speed, PI_OUTPUT);
      gpioSetMode(this->gpio_dir_1, PI_OUTPUT);
      gpioSetMode(this->gpio_dir_2, PI_OUTPUT);
      gpioWrite(this->gpio_speed, 0);
      gpioWrite(this->gpio_dir_1, 0);
      gpioWrite(this->gpio_dir_2, 0);

      this->gpio_enc_a = gpio_enc_a;
      this->gpio_enc_b = gpio_enc_b;
      gpioSetMode(this->gpio_enc_a, PI_INPUT);
      gpioSetMode(this->gpio_enc_b, PI_INPUT);
    }

    void spin(int velocity){
      if(velocity > this->max_speed){
        velocity = this->max_speed;
      }
      if(velocity < -this->max_speed){
        velocity = -this->max_speed;
      }
      if(velocity > 0){
        gpioWrite(this->gpio_dir_1, 1);
        gpioWrite(this->gpio_dir_2, 0);
        gpioPWM(this->gpio_speed, velocity);
      }else if(velocity < 0){
        gpioWrite(this->gpio_dir_1, 0);
        gpioWrite(this->gpio_dir_2, 1);
        gpioPWM(this->gpio_speed, velocity);
      }else{
        gpioWrite(this->gpio_dir_1, 0);
        gpioWrite(this->gpio_dir_2, 0);
        gpioWrite(this->gpio_speed, 0);
      }
    }

    void stop(){
      this->spin(0);
    }
    
    void set_max_speed(double new_max_speed){
      this->max_speed = new_max_speed;
    }
};

#define LEFT_MOTOR_SPEED 2
#define LEFT_MOTOR_DIR_1 4
#define LEFT_MOTOR_DIR_2 3
#define LEFT_ENCODER_A 27
#define LEFT_ENCODER_B 17

#define RIGHT_MOTOR_SPEED 13
#define RIGHT_MOTOR_DIR_1 6
#define RIGHT_MOTOR_DIR_2 5
#define RIGHT_ENCODER_A 26
#define RIGHT_ENCODER_B 19

class Drivetrain{
  private:
    Motor left_motor;
    Motor right_motor;
  public:
    Drivetrain(){
      left_motor.gpio_setup(LEFT_MOTOR_SPEED, LEFT_MOTOR_DIR_1, LEFT_MOTOR_DIR_2, LEFT_ENCODER_A, LEFT_ENCODER_B);
      right_motor.gpio_setup(RIGHT_MOTOR_SPEED, RIGHT_MOTOR_DIR_1, RIGHT_MOTOR_DIR_2, RIGHT_ENCODER_A, RIGHT_ENCODER_B);
    }

    void drive(int velocity){
      this->left_motor.spin(velocity);
      this->right_motor.spin(velocity);
    }

    void turn(int velocity){
      this->left_motor.spin(velocity);
      this->right_motor.spin(-velocity);
    }

    void stop(){
      this->left_motor.stop();
      this->right_motor.stop();
    }
};

#include <string>
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node{
    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            std::string command = msg->data.c_str();
            if(command == "drive"){
                printf("Driving\n");
                this->base.drive(255);
            }else if(command == "stop"){
                printf("Stopping\n");
                this->base.stop();
            }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    Drivetrain base;
  public:
    MinimalSubscriber() : Node("minimal_subscriber"){
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  
};

int main(int argc, char * argv[]){
    gpioInitialise();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    gpioTerminate();
    return 0;
}