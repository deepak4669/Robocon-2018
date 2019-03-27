#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <nRF24L01.h>

#include<motorPins.h>
#include<arrayOfLDR.h>

motorPins M1(46, 47, 8);
motorPins M2(A12, A13, 11);//a=52
motorPins M3(48, 49, 9);
motorPins M4(A14, A15, 10);//a=50,b=51

boolean isItBlue = 0;

arrayOfLDR array1(22, 23, 24, 25, 26, 27);
arrayOfLDR array2(28, 29, 30, 31, 32, 33);
arrayOfLDR array3(34, 35, 36, 37, 38, 39);
arrayOfLDR array4(40, 41, 42, 43, 44, 45);

const int countforTz1Red = 2;
const int countforTz2Red = 2;
const int countforTz3Red = 6;

const int backforTz1Red = 3;
const int backforTz2Red = 3;
const int backforTz3Red = 7;

const int countforTz1Blue = 2;
const int countforTz2Blue = 2;
const int countforTz3Blue = 6;

const int backforTz1 = 3;
const int backforTz2 = 3;
const int backforTz3 = 7;

const int  const_speed = 55; //Basic forward speed

const double Kp2 = 6.5; //Sideways motion constant(for 2 and 4)
const double Kp3 = 8;  //Sideways motion constant(for 1 and 3)

double error21; //error from 2nd sensor to 1st motor
double previous21;
double previous_sum21;

double error43;// error from 4th sensor to 3 motor
double previous43;
double previous_sum43;

double error12; // error from 1st sensor to 2nd motor
double previous12;
double previous_sum12;

double error34;// error from 3rd sensor to 4th motor
double previous34;
double previous_sum34;  

void moveToJ1()
{
  error21 = 0; //error from 2nd array for 1st motor
  error43 = 0; //error from 4th array for 3rd motor

  error34 = 0; //These are for junction stoppage.
  error12 = 0;

  previous21 = 0; //for derivative controller
  previous43 = 0;

  previous_sum21 = 0;// for integral controller.
  previous_sum43 = 0;

  while (1)
  {
    
    error34 = array3.read_sensor_values();//For Junction.
    error12 = array1.read_sensor_values();

    error21 = array2.read_sensor_values();
    error43 = array4.read_sensor_values();
    
    if (error21 == 0 && error43 == 0) //Case:1
    { 
      M1.forward(const_speed);
      M3.forward(const_speed);     
      Serial.println("FORWARD");
    }
    else if(error43==20) continue;
    
    else if ( error21 == 20) //junction
    {
      
      M2.stop();
      M4.stop();
      M1.stop();
      M3.stop();
      
      Serial.println("Stop");
      delay(500);
      
      error34 = array3.read_sensor_values();
      error12 = array1.read_sensor_values();
      
      Serial.println(error34);
      Serial.println(error12);
      
      while(error12==0.5 || error34==0.5)
      {
        
        error34 = array3.read_sensor_values();
        error12 = array1.read_sensor_values();
        
        M1.backward(30);
        M3.backward(30);
        
        Serial.println("LOOP");
        
      }
      
      junction_stop();
      M1.stop();
      M3.stop();
      
      Serial.println("Stop");
      break;
      
    }
    
    else if (error21 == 10 || error43 == 10) //
    {
      
      Serial.println("GARBAGE");
      Serial.println(error21);
      Serial.println(error43);

      M1.forward(const_speed);
      M3.forward(const_speed);

    }
    else if (error21 == 0.5 && error43 == 0.5) //Both Full Low
    {

      M1.forward(const_speed);
      M3.forward(const_speed);
      
      Serial.println("Both Sensors are low");

    }

    else if (error21 >=1 && error43 >=1)
    {
      
      M1.forward(const_speed);
      M3.forward(const_speed);
      
      M2.backward(abs(Kp2 * error21));
      M4.backward(abs(Kp2 * error43));

    }

    else if (error21 <=-1 && error43 <= -1)
    {
      
      M1.forward(const_speed);
      M3.forward(const_speed);

      M2.forward(abs(Kp2 * error21));
      M4.forward(abs(Kp2 * error43));
      
    } 
    else {

      double correction43 = array4.calculate_pid(error43, previous43, previous_sum43);
      double correction21 = array2.calculate_pid(error21, previous21, previous_sum21);

      Serial.println(error21);
      Serial.println(error43);

      Serial.print("correction43:");
      Serial.println(const_speed + correction43);
      Serial.println("correction21:");
      Serial.println(const_speed + correction21);

      M3.forward(const_speed + correction43);
      M1.forward(const_speed + correction21);


    }
    
    previous_sum43 += error43;
    previous_sum21 += error21;

    previous43 = error43;
    previous21 = error21;

    //delay(1000);
  }
}


void junction_stop()
{
  /*
  int error12 = array1.read_sensor_values();
  int error34 = array3.read_sensor_values();

  while (error12 != 0.5 && error34 != 0.5);
  */
  while (1)
    {
      M2.stop();
      M4.stop();
      error12 = array1.read_sensor_values();
      error34 = array3.read_sensor_values();
      if (error12 < 0)
      {
        M1.backward(30);
      }
      if (error34 < 0)
      {
        M3.backward(30);
      }
      if (error12 > 0)
      {
        M1.forward(30);
      }
      if (error34 > 0)
      {
        M3.forward(30);
      }
      if(error34==0&&error12==0) return;
      Serial.println("looped!");
    }
}
void moveToTz1(int counter)
{
  error12=0;
  previous12=0;
  previous_sum12=0;

  error34=0;
  previous34=0;
  previous_sum34=0;

  error21=0;
  error43=0;

  int count=0;

  while(1)
  {
    
    error12=array1.read_sensor_values();
    error34=array3.read_sensor_values();
    error21=array2.read_sensor_values();
    error43=array4.read_sensor_values();

    if(error12==0 && error34==0)
    {
      M2.backward(const_speed);
      M4.backward(const_speed);
      Serial.println("Forward");
    }
    
    else if(error12==20)
    {
      continue;
    }
    
    else if(error34==20)
    {
      count++;
      delay(150);
      if(count>=counter){
        M1.stop();
        M2.stop();
        M3.stop();
        M4.stop();
        break;
      }
      else
      {
        M2.forward(const_speed);
        M4.forward(const_speed);
      }
      
      
    }
    else if(error12==10||error34==10)
    {
      Serial.println("GARBAGE");
      M2.backward(const_speed);
      M4.backward(const_speed);
    }
    else if(error12==0.5&&error34==0.5)
    {
      
      M2.backward(const_speed);
      M4.backward(const_speed);
      Serial.println("BOTH LOW");
    }
    else if (error12 >0 && error34 >0 )
    {
      
      M2.backward(const_speed);
      M4.backward(const_speed);
      Serial.println("BOTH posituve");
      M1.forward(abs(Kp3 * error12));
      M3.forward(abs(Kp3 * error34));

    }

    else if (error12 <0 && error34 <0)
    {
      
      M2.backward(const_speed);
      M4.backward(const_speed);
      Serial.println("Both -ve");
      M1.backward(abs(Kp3 * error12));
      M3.backward(abs(Kp3 * error34));
      
    }
    else
    {
      Serial.println("GENERAL");
      double correction12=array1.calculate_pid(error12,previous12,previous_sum12);
      double correction34=array3.calculate_pid(error34,previous34,previous_sum34);
      Serial.print("4:");
      Serial.println(const_speed + correction34);
      Serial.print("2:");
      Serial.println(const_speed + correction12);
      M4.backward(const_speed + correction34);
      M2.backward(const_speed + correction12);
    }
    previous12=error12;
    previous34=error34;

    previous_sum12+=error12;
    previous_sum34+=error34;
    //delay(200);
  }
}

void backToJ1(int counter)
{
  error12=0;
  error34=0;

  error21=0;
  error43=0;

  previous12=0;
  previous34=0;

  previous_sum12=0;
  previous_sum34=0;

  int count=0;
 

  while(1)
  {
    error12=array1.read_sensor_values();
    error34=array3.read_sensor_values();
    error21=array2.read_sensor_values();
    error43=array4.read_sensor_values();

    if(error12==0&&error34==0)
    {
      M2.forward(const_speed);
      M4.forward(const_speed);
      Serial.println("Forward");
    }
    else if(error34==20) continue;
    else if(error12==20)//junction
    {
      count++;
      delay(100);
      if(count>=counter)
      {
        M1.stop();
        M2.stop();
        M3.stop();
        M4.stop();
        delay(500);
        while(error21==0.5 || error43==0.5)
      {
        
        error43 = array4.read_sensor_values();
        error21 = array2.read_sensor_values();
        
        M2.backward(30);
        M4.backward(30);
        
        Serial.println("LOOP");
        
      }
        M2.stop();
        M4.stop();
        
        junction_stop2(); //junction tuning
        
        break;           //fine tune here
      }
      else
      {
        
        M2.forward(const_speed);
        M4.forward(const_speed);
        
      }
     }
     else if(error12==10||error34==10)
     
     {
      M2.forward(const_speed);
      M4.forward(const_speed);
     }
     else if(error12==0.5&&error34==0.5)
     {
      M2.forward(const_speed);
      M4.forward(const_speed);
     }
     else if(error12>0&&error34>0)
     {
      M2.forward(const_speed);
      M4.forward(const_speed);

      M1.forward(abs(Kp3*error12));
      M3.forward(abs(Kp3*error34));
     }
     else if(error12<0&&error34<0)
     {
      M2.forward(const_speed);
      M4.forward(const_speed);

      M1.backward(abs(Kp3*error12));
      M3.backward(abs(Kp3*error34));
     }
     else
     {
      double correction32=array3.calculate_pid(error34,previous34,previous_sum34); //Careful
      double correction14=array1.calculate_pid(error12,previous12,previous_sum12);
      M2.forward(const_speed+correction32);
      M4.forward(const_speed+correction14);
     }
     previous34=error34;
     previous12=error12;

     previous_sum34+=error34;
     previous_sum12+=error12;
  }
}


void junction_stop2()
{
  while (1)
    {
      M1.stop();
      M3.stop();
      error21 = array2.read_sensor_values();
      error43 = array4.read_sensor_values();
      if (error21 < 0)
      {
        M2.forward(30);
      }
      if (error43 < 0)
      {
        M4.forward(30);
      }
      if (error21 > 0)
      {   
        M2.backward(30);
      }
      if (error43 > 0)
      {
        M4.backward(30);
      }
      if(error43==0 || error21==0) break;
      Serial.println("looped!");
    }
    //junction_stop();
     
}



    
RF24 radio(6, 7);

const byte address[6] = "00001";


void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
}

void loop()
{
 if(radio.available())
 {
  char text[32] = "";
  radio.read(&text, sizeof(text));
  Serial.println(text);
  
  if(strcmp(text,"J1")==0)
  {
    delay(100);
    Serial.println("J1");
    moveToJ1();
    //Serial.println("J1");
  }
  if(strcmp(text,"J2")==0)
  {
    delay(1000);
    moveToJ1();
    Serial.println("J2");
  }
  if(strcmp(text,"Tz1")==0)
  {
    delay(200);
    moveToTz1(countforTz1Red);
    delay(5000);
    backToJ1(backforTz1Red);
  }

  if(strcmp(text,"Tz21")==0)
  {
    delay(100);
    moveToJ1();
    delay(1000);
    moveToTz1(countforTz1Red);
    delay(1000);
    backToJ1(backforTz1Red);
  }
  if(strcmp(text,"Tz22")==0)
  {
    delay(100);
    moveToTz1(countforTz1Red);
    delay(1000);
    backToJ1(backforTz1Red);
  }
  if(strcmp(text,"Tz32")==0)
  {
    delay(100);
    moveToTz1(countforTz3Red);
    delay(2000);
    backToJ1(backforTz3Red);
  }
 
 if(strcmp(text,"ss")==0){
  M1.stop();
  M2.stop();
  M3.stop();
  M4.stop();
  
 }
}
}
