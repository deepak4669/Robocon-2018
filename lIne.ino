#include<motorPins.h>
#include<arrayOfLDR.h>


motorPins M1(46, 47, 8);
motorPins M2(52, 53, 11);
motorPins M3(48, 49, 9);
motorPins M4(50, 51, 10);


arrayOfLDR array1(22, 23, 24, 25, 26, 27);
arrayOfLDR array2(28, 29, 30, 31, 32, 33);
arrayOfLDR array3(34, 35, 36, 37, 38, 39);
arrayOfLDR array4(40, 41, 42, 43, 44, 45);


#define relay1 50
#define relay2 51


const int optimal_speed = 20;
const int resolution = 2;
const double Kp3 = 1;

const int  const_speed = 60;

const double Kp2 = 6.5;

double error21;
double previous23;
double error43;
double previous41;
double previous_sum23;
double previous_sum41;

//boolean sideways=true;

/*
  void switchToNC()//for 1 and 3 motors
  {
    digitalWrite(relay1,LOW);
    digitalWrite(relay2,LOW);
    sideways=false;
  }

  void switchToNO()//for 2 and 4 motors
  {
    digitalWrite(relay1,HIGH);
    digitalWrite(relay2,LOW);
    sideways=false;
  }
*/


void moveToJ1()
{
  error21 = 0; //error from 2nd array for 1st motor
  error43 = 0; //error from 4th array for 3rd motor

  double error33=25;
  double error11=25;

  previous23 = 0; //for derivative controller
  previous41 = 0;

  previous_sum23 = 0;
  previous_sum41 = 0;

  while (1)
  {
    error33 = array3.read_sensor_values();
    error11 = array1.read_sensor_values();

    error21 = array2.read_sensor_values();
    error43 = array4.read_sensor_values();
    
    
    
    if (error21 == 0 && error43 == 0) //Case:1
    {
      
      // if(sideways) switchToNO();
      M1.forward(const_speed);
      M3.forward(const_speed);
      Serial.println("FORWARD");

    }
    else if ( error21 == 20) //junction
    {
      
      M2.stop();
      M4.stop();
      M1.stop();
      M3.stop();
      Serial.println("Stop");
      //while(error43!=20);
      delay(500);
      error33 = array3.read_sensor_values();
      error11 = array1.read_sensor_values();
      Serial.println(error33);
      Serial.println(error11);
      while(error11==0.5&&error33==0.5)
      {
        
        error33 = array3.read_sensor_values();
        error11 = array1.read_sensor_values();
        
        M1.backward(30);
        M3.backward(30);
        Serial.println("LOOP");
        
      }
      
      M1.stop();
      M3.stop();
      //delay(500);
      Serial.println("Stop");
      //junction_stop();
      break;
      
    }
    
    else if (error21 == 10 || error43 == 10) //
    {
      Serial.println("GARBAGE");
      Serial.println(error21);
      Serial.println(error43);
      //if(sideways) switchToNO();
      M1.forward(const_speed);
      M3.forward(const_speed);

    }
    else if (error21 == 0.5 && error43 == 0.5) //Both Full Low
    {
      /*if(sideways)
        switchToNO();*/
      M1.forward(const_speed);
      M3.forward(const_speed);
      Serial.println("Both Sensors are low");

    }
    /*
      else if(error21>0&&error43>0)
      {
       if(sideways)
        switchToNO();
        double correction41=array4.calculate_pid(error43,previous41,previous_sum41);
        double correction23=array2.calculate_pid(error21,previous23,previous_sum23);
        M3.forward(const_speed);
        M1.forward(const_speed+abs(correction41+correction23));
        Serial.println("Both Negatie ");
        Serial.println("M2 greater");
        Serial.println(const_speed+abs(correction41+correction23));



      }
      else if(error21<0&&error43<0)
      {
        //if(sideways) switchToNO();
        double correction41=array4.calculate_pid(error43,previous41,previous_sum41);
        double correction23=array2.calculate_pid(error21,previous23,previous_sum23);
        M3.forward(const_speed+abs(correction41+correction23));
        M1.forward(const_speed);
        Serial.println("Both Positive");
        Serial.println("M1 greater");
        Serial.println(const_speed+abs(correction41+correction23));

      }
    */

    else if (error21 > 0 && error43 > 0)
    {
      M1.forward(const_speed);
      M3.forward(const_speed);
      
      M2.backward(abs(Kp2 * error21));
      M4.backward(abs(Kp2 * error43));

    }

    else if (error21 < 0 && error43 < 0)
    {
      M1.forward(const_speed);
      M3.forward(const_speed);

      M2.forward(abs(Kp2 * error21));
      M4.forward(abs(Kp2 * error43));
    }
/*
    else if ( error21 == 20) //junction
    {
      M2.stop();
      M4.stop();
      M1.stop();
      M3.stop();
      Serial.println("Stop");
      //junction_stop();
      break;
      
    }
  */  
    else {
      
      //if(sideways)
      //switchToNO();

      double correction41 = array4.calculate_pid(error43, previous41, previous_sum41);
      double correction23 = array2.calculate_pid(error21, previous23, previous_sum23);

      Serial.println(error21);
      Serial.println(error43);

      Serial.print("correction41:");
      Serial.println(const_speed + correction41);
      Serial.println("correction23:");
      Serial.println(const_speed + correction23);

      M3.forward(const_speed + correction41);
      M1.forward(const_speed + correction23);


    }
    previous_sum41 += error43;
    previous_sum23 += error21;

    previous41 = error43;
    previous23 = error21;

    //delay(1000);
  }
}

void junction_stop()
{
  int error11 = array1.read_sensor_values();
  int error33 = array3.read_sensor_values();

  while (error11 != 0.5 && error33 != 0.5);
  while (1)
    {
      M2.stop();
      M4.stop();
      error11 = array1.read_sensor_values();
      error33 = array3.read_sensor_values();
      if (error11 < 0)
      {
        M1.backward(20);
      }
      if (error33 < 0)
      {
        M3.backward(20);
      }
      if (error11 > 0)
      {
        M1.forward(20);
      }
      if (error33 > 0)
      {
        M3.forward(20);
      }
      Serial.println("looped!");
    }
}

void setup()
{
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  moveToJ1();
}

void loop()
{
  M1.stop();
  M2.stop();
  M3.stop();
  M4.stop();
}
