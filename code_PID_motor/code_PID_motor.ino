// Autor: Edison P. Velasco Sánchez
// email: evs25@alu.ua.es 

#define C1 0  // interruption 0
#define C2 1  // interruption 1
#define encoder_A 2  // encoder A physically connected to D2 ***** this data is not used in interruptions
#define encoder_B 3  // encoder B physically connected to D3
#define EN 5  // enable/disable motor
#define PWM 9 // motor control pwm (0 a 255)
#define INA 6 // motor turn CW 
#define INB 7 // motor turn CCW
#define end_1 4 // end switch CW
#define end_2 7 // end switch CCW
#include <digitalWriteFast.h>
#include <Wire.h>

//////////////////global PID////////////////// 
unsigned long pid_time = 0;        // time for PID sampling
float error_prev = 0 ;             // cumulative error
float error_curr = 0 ;              // previous error
float Setpoint = 0 ;
bool sign =0;                      // setpiont data sign for I2c communication
////////////////////////////////////////////////

//////////////// global variables
double rpm = 0;            // revolutions per minute.
double rpm_prev = 0;       // previous rpm
double rpm_curr = 0;       // current rpm
double a = 0.9, b = 0.1;   // variables para filtro en RPM
double cv_prev = 0;         //previous Control value
double cv_curr = 0;         //current Control value
double cv = 0;            

unsigned long num_encod = 220;  // number of encoder slots
unsigned long timeold = 0;      // Tiempo 
unsigned long time_int1 = 0;    // time of first  interruption
unsigned long time_int2 = 0;    // time of second interruption 
unsigned long time_rest = 0;
volatile byte pulses = 0;       // Number of pulses read by the Arduino in one second.
static volatile unsigned long debounce = 0; // Bounce time.
float _RightEncoderTicks = 0, _RightEncoderTicks_ant=0;
bool _RightEncoderBSet;

byte m_velocidad = 100 ;  // motor velocidad
byte slave_num = 1; // numero de Escalvo
 void setup ( ) {
  Serial.begin(9600); // Serial port speed setting
  pinMode(encoder_A, INPUT); 
  pinMode(encoder_B, INPUT); 
  pinMode(end_1, INPUT); 
  pinMode(end_2, INPUT); 
  pinMode (INA,OUTPUT); 
  pinMode (INB,OUTPUT); 
  pinMode (EN ,OUTPUT); 
  pinMode (13,OUTPUT); 
  digitalWrite(EN,HIGH);  
  attachInterrupt(C1, detection_c1, FALLING);// an external interrupt is programmed on pin 2, whenever there is a change from high to low
  digitalWrite(INA,HIGH);

  // We connect this device to the I2C bus with address 1 (Slave 1)
  Wire.begin(slave_num);
 
  // Register the event when receiving data
  Wire.onReceive(i2cdata);
  Wire.onRequest(requestEvent); 
 }

 void loop ( )
 {
  if(millis() - timeold  > 50){
    noInterrupts() ; // stop interruptions
    rpm_curr = (60.0*1000.0 / num_encod )/ (millis() - timeold)* _RightEncoderTicks; // calculation of revolutions per minute
    rpm = a*rpm_prev+b*rpm_curr; //RPM filter
    rpm_prev = rpm;
    timeold = millis(); 
   //Serial.print("Pulses: ");
   //Serial.println(pulses); 
       
    _RightEncoderTicks = 0;
    cv_curr = PID_PWM (rpm,Setpoint,1.95,0.60,0.95); // PID to tune
    cv = 0.9*cv_prev+0.1*cv_curr; // complementary filter 
    cv_prev = cv;   

    //if (!digitalRead(end_2)&& cv>0 && digitalRead(end_1) || digitalRead(end_2)&& !digitalRead(end_1)&& cv<0){
  if (analogRead(end_2)<100 && cv>0 || analogRead(end_1)<100 && cv<0){  
      //Stop the motor 
      digitalWrite(EN,LOW);
      digitalWrite(INA,0);
      digitalWrite(INB,0);
      analogWrite(PWM,0);    
        
    }    
  else{    
    
      digitalWrite(EN,HIGH); 
      digitalWrite(INA,cv<0);
      digitalWrite(INB,cv>0);
      analogWrite(PWM,abs(cv));
    }
    interrupts(); // reset the interruption
    
  }  

//SERIAL TO ADJUST THE MOTOR SPEED
  /*
  if(Serial.available()>0){  
      int inChar = Serial.parseInt();       
      Setpoint= (int)(inChar);       
  }
   Serial.print(Setpoint);
   Serial.print(',');
   Serial.print(cv);  
   Serial.print(',');
   Serial.println(rpm);  

   */
  
 } 


 float PID_PWM(float pv, float sp, float Kp, float Ki,float Kd){

  float error = sp - pv;
  float P=0,
        I=0,
        D=0;
  unsigned long T;
  T=millis() - pid_time;
  float Kz = T/1000.0;
   
  if(analogRead(end_2)>100 && analogRead(end_1)>100)
  
    error_prev += error; 
  else
    error_prev = 0;
  
    P = Kp * error;
    I = Ki * ((error_prev + error) * Kz) ;
    D = Kd * (error - error_curr)/Kz;
  
  error_curr = error;
  //limits of error acumulation
  if(error_prev>25500)
    error_prev = 25500;
  if(error_prev<-25500)
    error_prev = -25500;
    
  float cv = P+I+D;
  pid_time = millis();
  if(cv>255)
    cv = 255;
 if(cv<-255)
   cv = -255;
    if(cv>-20 && cv<20)
      cv = 0;
  return (cv);
  
 }

  void detection_c1()
    {  
      _RightEncoderBSet = digitalReadFast(encoder_B);   // read the input pin
      #ifdef RightEncoderIsReversed
         _RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
      #else 
         _RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
      #endif
  }

 void i2cdata() { 
      int pinOut = 0;
      int estado = 0;
     //digitalWrite (13,HIGH); 
     if (Wire.available() == 2)
        sign = Wire.read();
     if (Wire.available() == 1) {
        Setpoint = Wire.read();
     if(!sign)
          Setpoint = -Setpoint;
     }
     //digitalWrite (13,LOW); 
}

void requestEvent(){ 
 Wire.write(slave_num);  
 Wire.write((byte)abs(Setpoint));  
 //Wire.write((byte)abs(rpm));
 //if(rpm<0)
 if(Setpoint<0)
    Wire.write(0);
  else
    Wire.write(1);   
}
