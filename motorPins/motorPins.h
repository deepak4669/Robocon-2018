/* motorPins.h*/
#ifndef motorPins_h
#define motorPins_h

#include "Arduino.h"

class motorPins
{
    public:
        motorPins(int a,int b,int c);
        void forward(int pwm);
        void backward(int pwm);
        void stop();
        void powerStop();
	
    private:
        int _M1;
        int _M2;
        int _pwm;
};
#endif
