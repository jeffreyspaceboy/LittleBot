#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define MOTOR_PIN_1 8
#define MOTOR_PIN_2 9
#define MOTOR_PIN_PWM 10

#define MOTOR_MAX_POWER 255.0
#define MOTOR_GEAR_RATIO 9.28

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

#define ENCODER_TPR 22.0 //Ticks per rotation

class PID_Controller{
  private:
    float target,kp,ki,kd,dt=0.0,power=0.0;
    float error=0.0,prev_error=0.0,integral=0.0,derivative=0.0;
    unsigned long current_time=0,prev_time=0;
  public:
    PID_Controller(float kp=0.0, float ki=0.0, float kd=0.0){
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
    }
    bool control_loop(float target, float max_output, float error_cutoff, float times_good_cutoff, float timeout_sec, float *current_position, float *output){
      bool move_complete = false;
      int times_good = 0;
      prev_error = target;
      unsigned long start_time = micros();
      while(!move_complete && *current_position <= target){
        current_time = micros();
        dt = ((float)(current_time-prev_time))/(1.0e6);
        prev_time = current_time;
        error = target-*current_position;
        integral = integral+(error*dt);
        derivative = (error-prev_error)/dt;
        power = (kp*error)+(kd*derivative)+(ki*integral);
        *output = fabs(power);
        if(*output>max_output){*output = max_output;}
        if(power<0){ *output = -*output;}
        if(error < error_cutoff){ times_good += 1; }
        if(times_good >= times_good_cutoff){ move_complete = true; }
        if((micros()-start_time) > timeout_sec){ return false; }
        prev_error = error;
      }
      return true;
    }
    float output(float target, float max_output, float current_position){
      current_time = micros();
      dt = ((float)(current_time-prev_time))/(1.0e6);
      prev_time = current_time;
      error = target-*current_position;
      integral = integral+(error*dt);
      derivative = (error-prev_error)/dt;
      power = (kp*error)+(kd*derivative)+(ki*integral);
      float output = fabs(power);
      if(output>max_output){output = max_output;}
      if(power<0){ output = -output;}
      prev_error = error;
      return output;
    }
};

class Sensor{
  public:
    static volatile float _value = 0.0; //Specify ticks as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/  
    virtual void _reset(void){
      _value = 0.0;
    }
    virtual float _get_value(void){
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ return _value; }
    }
};

class Encoder : Sensor{
  public:
    static int pin_1, pin_2;
    Encoder(){}
    Encoder(int gpio_pin_1, int gpio_pin_2){
      pin_1 = gpio_pin_1;
      pin_2 = gpio_pin_2;
      pinMode(pin_1,INPUT);
      pinMode(pin_2,INPUT);
      attachInterrupt(digitalPinToInterrupt(pin_1),event,CHANGE);
    }
    static void event(void) { //Results in double the number of encoder counts (22 ticks/rot)
      if(digitalRead(pin_1)==HIGH){
        if(digitalRead(pin_2)==LOW){_value++;}
        else{_value--;}
      }else{
        if(digitalRead(pin_2)==LOW){_value--;}
        else{_value++;}
      }
    }
};

class Motor{
  private:
    int pin_1, pin_2, pin_pwm;
    float power = 0.0, max_power = 255.0;
    Encoder encoder;
    PID_Controller pid_controller;
  public:
    Motor(int gpio_pin_1, int gpio_pin_2, int gpio_pin_pwm, Encoder motor_encoder){
      pin_1 = gpio_pin_1;
      pin_2 = gpio_pin_2;
      pin_pwm = gpio_pin_pwm;
      pinMode(pin_1,OUTPUT);
      pinMode(pin_2,OUTPUT);
      pinMode(pin_pwm,OUTPUT);
      encoder = motor_encoder;
    }
    Motor(int gpio_pin_1, int gpio_pin_2, int gpio_pin_pwm, int gpio_enc_pin_1, int gpio_enc_pin_2){
      pin_1 = gpio_pin_1;
      pin_2 = gpio_pin_2;
      pin_pwm = gpio_pin_pwm;
      pinMode(pin_1,OUTPUT);
      pinMode(pin_2,OUTPUT);
      pinMode(pin_pwm,OUTPUT);
      encoder = Encoder(gpio_enc_pin_1,gpio_enc_pin_2);
    }   
    
    void pid_control(float target = 0.0){
      spin(pid_controller.output(target, max_power, encoder._get_value());
    }

    void spin(float pwm_val){
      power = fabs(pwm_val);
      if(pwm_val > 0){
        digitalWrite(pin1,HIGH);
        digitalWrite(pin2,LOW);
      }else if(pwm_val < 0){
        digitalWrite(pin1,LOW);
        digitalWrite(pin2,HIGH);
      }else{
        digitalWrite(pin1,LOW);
        digitalWrite(pin2,LOW);
      }
      analogWrite(pin_pwm,power); 
    }
};



void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_PIN_A,INPUT);
  pinMode(ENCODER_PIN_B,INPUT);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A),readEncoder,RISING);
  
  pinMode(MOTOR_PIN_PWM,OUTPUT);
  pinMode(MOTOR_PIN_1,OUTPUT);
  pinMode(MOTOR_PIN_2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {
  //int target = 350;
  int target = 250*sin(time_previous/1e6);

  // PID constants
  float kp = 20.00;
  float kd = 2.00;
  float ki = 5.00;

  long time_current = micros(); // time difference
  float time_delta = ((float) (time_current - time_previous))/( 1.0e6 );
  time_previous = time_current;

  // Read the position in an atomic block to avoid a potential misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  int error = target - pos; //Error
  error_integral = error_integral + error*time_delta; //Integral
  float dedt = (error-error_previous)/(time_delta); //Derivative  
  float u = (kp*error) + (kd*dedt) + (ki*error_integral); //Control signal

  float power = fabs(u); //Motor power
  if( power > MOTOR_MAX_POWER ){ power = MOTOR_MAX_POWER; }

  
  int dir = 1; // motor direction
  if(u<0){ dir = -1; }

  setMotor(dir,power,MOTOR_PIN_PWM,MOTOR_PIN_1,MOTOR_PIN_2); //Signal the motor

  error_previous = error; // store previous error

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}


//#define MOTOR_PIN_1 9
//#define MOTOR_PIN_2 8
//#define MOTOR_DIR_PIN 10
//
//#define ENCODER_PIN_A 2
//#define ENCODER_PIN_B 3
//
//int count = 0;
//
//void setup() {
//  Serial.begin(9600);
//  pinMode(MOTOR_PIN_1, OUTPUT);
//  pinMode(MOTOR_PIN_2, OUTPUT);
//  pinMode(MOTOR_DIR_PIN, OUTPUT);
//
//  pinMode(ENCODER_PIN_A, INPUT);
//  pinMode(ENCODER_PIN_B, INPUT);
//  
//  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), tick, CHANGE);
//  //attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), tick, CHANGE);
//}
//void loop() {
//  noInterrupts();
//  digitalWrite(MOTOR_PIN_1, LOW);
//  digitalWrite(MOTOR_PIN_2, HIGH);
//  analogWrite(MOTOR_DIR_PIN, 255);
//  delay(5000);
//  analogWrite(MOTOR_DIR_PIN, 0);
//  while(1){ delay(500); }
//}
//
//void tick(){
//  count++;
//  Serial.print("Count: ");
//  Serial.print(count);
//  Serial.print("\n");
//}
