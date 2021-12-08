#include "../include/Definitions.h"
#include "../include/Motor.h"
#include "../include/Encoder.h"

#include <pigpio.h>


int main(int argc, char * argv[]){
    if (gpioInitialise() < 0) { return FAILURE; }
    
    gpioTerminate();
    return SUCCESS;
}