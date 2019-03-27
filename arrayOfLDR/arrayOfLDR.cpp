/*arrayOfLDR.cpp*/

#include "Arduino.h"
#include "arrayOfLDR.h"

arrayOfLDR::arrayOfLDR(int ld1,int ld2,int ld3,int ld4,int ld5,int ld6)
{
    pinMode(ld1,INPUT);
    pinMode(ld2,INPUT);
    pinMode(ld3,INPUT);
    pinMode(ld4,INPUT);
    pinMode(ld5,INPUT);
    pinMode(ld6,INPUT);
    
    _ld1=ld1;
    _ld2=ld2;
    _ld3=ld3;
    _ld4=ld4;
    _ld5=ld5;
    _ld6=ld6;
    
    	
    
}

float arrayOfLDR::read_sensor_values()
{


    int ldr1=digitalRead(_ld1);
    int ldr2=digitalRead(_ld2);
    int ldr3=digitalRead(_ld3);
    int ldr4=digitalRead(_ld4);
    int ldr5=digitalRead(_ld5);
    int ldr6=digitalRead(_ld6);
    
    int junction=ldr1+ldr2+ldr3+ldr4+ldr5+ldr6;
    
    float ret_error = 10;

    if(junction > 3) return 20;
    
    else if(ldr1==0 && ldr2==0 && ldr3==1 && ldr4==1 && ldr5==0 && ldr6==0)
        ret_error = 0;

    else if(ldr1==0 && ldr2==0 && ldr3==1 && ldr4==0 && ldr5==0 && ldr6==0)
        ret_error = 1;
        
    else if(ldr1==0 && ldr2==1 && ldr3==1 && ldr4==0 && ldr5==0 && ldr6==0)
        ret_error = 2;
        
    else if(ldr1==0 && ldr2==1 && ldr3==0 && ldr4==0 && ldr5==0 && ldr6==0 )
        ret_error = 3;
        
    else if(ldr1==1 && ldr2==1 && ldr3==0 && ldr4==0 && ldr5==0 && ldr6==0)
        ret_error = 4;
    
    else if(ldr1==1 && ldr2==0 && ldr3==0 && ldr4==0 && ldr5==0 && ldr6==0)
        ret_error = 5;
    
    else if(ldr1==0 && ldr2==0 && ldr3==0 && ldr4==1 && ldr5==0 && ldr6==0)
        ret_error = -1;
        
    else if(ldr1==0 && ldr2==0 && ldr3==0 && ldr4==1 && ldr5==1 && ldr6==0)
        ret_error = -2;
    
    else if(ldr1==0 && ldr2==0 && ldr3==0 && ldr4==0 && ldr5==1 && ldr6==0)
        ret_error = -3;
        
    else if(ldr1==0 && ldr2==0 && ldr3==0 && ldr4==0 && ldr5==1 && ldr6==1)
        ret_error = -4;
        
    else if(ldr1==0 && ldr2==0 && ldr3==0 && ldr4==0 && ldr5==0 && ldr6==1)
        ret_error = -5;

	else if(ldr1==0 && ldr2==0 && ldr3==0 && ldr4==0 && ldr5==0 && ldr6==0)   	 	
	    ret_error = 0.5;
	    
    else ret_error = 10;
	
	return ret_error;
	     
}

double arrayOfLDR::calculate_pid(int error,int previous_error,int previous_sum)
{
    int p=error;
    int i=error+previous_sum;
    int d=error-previous_error;
    double kp=7.5,kd=2.5,ki=0;
    double correction_value=(kp)*p+(ki)*i+(kd)*d;
    return correction_value;
    
}

