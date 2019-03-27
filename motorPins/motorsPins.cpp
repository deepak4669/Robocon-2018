#include "Arduino.h"
#include "motorPins.h"

motorPins::motorPins(int a,int b,int c)
{
    pinMode(a,OUTPUT);
    pinMode(b,OUTPUT);
    pinMode(c,OUTPUT);
    _M1=a;
    _M2=b;
    _pwm=c;
}
void motorPins::forward(int pwm)
{
	
    digitalWrite(_M1,HIGH);
    digitalWrite(_M2,LOW);
    analogWrite(_pwm,pwm);
}
void motorPins::backward(int pwm)
{
	
    digitalWrite(_M1,LOW);
    digitalWrite(_M2,HIGH);
    analogWrite(_pwm,pwm);
}
void motorPins::stop()
{
    digitalWrite(_M1,LOW);
    digitalWrite(_M2,LOW);
}
void motorPins::powerStop()
{
    digitalWrite(_M1,HIGH);
    digitalWrite(_M2,HIGH);
}


