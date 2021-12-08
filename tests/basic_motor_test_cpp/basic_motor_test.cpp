#include <stdint.h> //For uint8_t
#include <pigpio.h> //For GPIO control
#include <unistd.h> //For usleep

class Motor{
  private:
    uint8_t gpio_en, gpio_dir_a, gpio_dir_b;
    uint8_t gpio_enc_a, gpio_enc_b;
    int max_speed = 255;
  public: 
    Motor(){}
    Motor(uint8_t gpio_en_pin, uint8_t gpio_dir_a_pin, uint8_t gpio_dir_b_pin, uint8_t gpio_enc_a_pin, uint8_t gpio_enc_b_pin){
      gpio_setup(gpio_en_pin, gpio_dir_a_pin, gpio_dir_b_pin, gpio_enc_a_pin, gpio_enc_b_pin);
    }

    void gpio_setup(uint8_t gpio_en_pin, uint8_t gpio_dir_a_pin, uint8_t gpio_dir_b_pin, uint8_t gpio_enc_a_pin, uint8_t gpio_enc_b_pin){
      this->gpio_en = gpio_en_pin;
      this->gpio_dir_a = gpio_dir_a_pin;
      this->gpio_dir_b = gpio_dir_b_pin;

      gpioSetMode(this->gpio_en, PI_OUTPUT);
      gpioSetMode(this->gpio_dir_a, PI_OUTPUT);
      gpioSetMode(this->gpio_dir_b, PI_OUTPUT);

      gpioWrite(this->gpio_en, 0);
      gpioWrite(this->gpio_dir_a, 0);
      gpioWrite(this->gpio_dir_b, 0);

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
        gpioWrite(this->gpio_dir_a, 1);
        gpioWrite(this->gpio_dir_b, 0);
        gpioPWM(this->gpio_en, velocity);
      }else if(velocity < 0){
        gpioWrite(this->gpio_dir_a, 0);
        gpioWrite(this->gpio_dir_b, 1);
        gpioPWM(this->gpio_en, velocity);
      }else{
        gpioWrite(this->gpio_dir_a, 0);
        gpioWrite(this->gpio_dir_b, 0);
        gpioWrite(this->gpio_en, 0);
      }
    }

    void stop(){
      this->spin(0);
    }
    
    void set_max_speed(double new_max_speed){
      this->max_speed = new_max_speed;
    }
};

#define LEFT_EN_PIN 6
#define LEFT_DIR_A_PIN 27
#define LEFT_DIR_B_PIN 22
#define LEFT_ENC_A_PIN 17
#define LEFT_ENC_B_PIN 4

#define RIGHT_EN_PIN 5
#define RIGHT_DIR_A_PIN 12
#define RIGHT_DIR_B_PIN 13
#define RIGHT_ENC_A_PIN 19
#define RIGHT_ENC_B_PIN 26

int main(int argc, char * argv[]){
    gpioInitialise();
    Motor LeftMotor(LEFT_EN_PIN, LEFT_DIR_A_PIN, LEFT_DIR_B_PIN, LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
    Motor RightMotor(RIGHT_EN_PIN, RIGHT_DIR_A_PIN, RIGHT_DIR_B_PIN, RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
    LeftMotor.spin(255);
    RightMotor.spin(255);
    usleep(1000000);
    LeftMotor.spin(0);
    RightMotor.spin(0);
    gpioTerminate();
    return 0;
}

