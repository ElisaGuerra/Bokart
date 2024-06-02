/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * Arduino Mecanum Omni Direction Wheel Robot Car
 * Tutorial URL https://osoyoo.com/?p=43404
 * CopyRight www.osoyoo.com

 * After running the code, smart car will automatically move along the black track line in the white ground.
 *  
 * 
 */
#define MID_SPEED 100    
#define HIGH_SPEED 200    
#define LOW_SPEED 50    
#define LONG_DELAY_TIME 70 
#define DELAY_TIME 40 
#define SHORT_DELAY_TIME 30 
         
#define speedPinR 9   //  Front Wheel PWM pin connect Model-Y M_B ENA 
#define rightMotorDirPin1  22    //Front Right Motor direction pin 1 to Model-Y M_B IN1  (K1)
#define rightMotorDirPin2  24   //Front Right Motor direction pin 2 to Model-Y M_B IN2   (K1)                                 
#define leftMotorDirPin1  26    //Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
#define leftMotorDirPin2  28   //Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define speedPinL 10   //  Front Wheel PWM pin connect Model-Y M_B ENB

#define speedPinRB 12   //  Rear Wheel PWM pin connect Left Model-Y M_A ENA 
#define rightMotorDirPin1B  7    //Rear Right Motor direction pin 1 to Model-Y M_A IN1 ( K1)
#define rightMotorDirPin2B 8    //Rear Right Motor direction pin 2 to Model-Y M_A IN2 ( K1) 
#define leftMotorDirPin1B 5    //Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3)
#define leftMotorDirPin2B 6  //Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define speedPinLB 11    //  Rear Wheel PWM pin connect Model-Y M_A ENB

#define sensor1   A4 // Left most sensor
#define sensor2   A3 // 2nd Left   sensor
#define sensor3   A2 // center sensor
#define sensor4   A1 // 2nd right sensor// Right most sensor
#define sensor5   A0 // Right most sensor

/*motor control*/
void FR_fwd(int speed)  //front-right wheel forward turn
{
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW); 
  analogWrite(speedPinR,speed);
}
void FR_bck(int speed) // front-right wheel backward turn
{
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH); 
  analogWrite(speedPinR,speed);
}
void FL_fwd(int speed) // front-left wheel forward turn
{
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
}
void FL_bck(int speed) // front-left wheel backward turn
{
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
}

void RR_fwd(int speed)  //rear-right wheel forward turn
{
  digitalWrite(rightMotorDirPin1B, LOW);
  digitalWrite(rightMotorDirPin2B,HIGH); 
  analogWrite(speedPinRB,speed);
}
void RR_bck(int speed)  //rear-right wheel backward turn
{
  digitalWrite(rightMotorDirPin1B, HIGH);
  digitalWrite(rightMotorDirPin2B,LOW); 
  analogWrite(speedPinRB,speed);
}
void RL_fwd(int speed)  //rear-left wheel forward turn
{
  digitalWrite(leftMotorDirPin1B,LOW);
  digitalWrite(leftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB,speed);
}
void RL_bck(int speed)    //rear-left wheel backward turn
{
  digitalWrite(leftMotorDirPin1B,HIGH);
  digitalWrite(leftMotorDirPin2B,LOW);
  analogWrite(speedPinLB,speed);
}
//Movement functions
void forward(int speed_left,int speed_right)
{
   RL_fwd(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_fwd(speed_left); 
}
void right_turn(int speed_left,int speed_right)
{
   RL_fwd(speed_left);
   RR_bck(speed_right);
   FR_bck(speed_right);
   FL_fwd(speed_left);
}
void left_turn(int speed_left,int speed_right)
{
   RL_bck(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_bck(speed_left);
}
void reverse(int speed)
{
   RL_bck(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_bck(speed); 
}

void stop_bot()    //Stop
{
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
  digitalWrite(rightMotorDirPin1B, LOW);
  digitalWrite(rightMotorDirPin2B,LOW);   
  digitalWrite(leftMotorDirPin1B, LOW);
  digitalWrite(leftMotorDirPin2B,LOW); 
  digitalWrite(rightMotorDirPin1, LOW);
  digitalWrite(rightMotorDirPin2,LOW);   
  digitalWrite(leftMotorDirPin1, LOW);
  digitalWrite(leftMotorDirPin2,LOW); 
  delay(40);
}


//Pins initialize
void init_GPIO()
{
  pinMode(rightMotorDirPin1, OUTPUT); 
  pinMode(rightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  pinMode(rightMotorDirPin1B, OUTPUT); 
  pinMode(rightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
 
  pinMode(leftMotorDirPin1B, OUTPUT);
  pinMode(leftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  
  stop_bot();
}

void setup()
{
  init_GPIO();
  Serial.begin(9600);
}

void loop(){
  tracking();
}

void tracking()
{
  String senstr="";
  int s0 = !digitalRead(sensor1);
  int s1 = !digitalRead(sensor2);
  int s2 = !digitalRead(sensor3);
  int s3 = !digitalRead(sensor4);
  int s4 = !digitalRead(sensor5);
  int sensorvalue=32;
  sensorvalue +=s0*16+s1*8+s2*4+s3*2+s4;
  senstr= String(sensorvalue,BIN);
  senstr=senstr.substring(1,6);
  
  Serial.print(senstr);
  Serial.print("\t");
  if ( senstr=="01100" || senstr=="00110" || senstr=="01110" || senstr=="00100" || senstr=="11110" || senstr=="01111" || senstr=="11111") //Forward
  {
    forward(HIGH_SPEED,HIGH_SPEED);
    delay(DELAY_TIME);
  }
  if ( senstr=="11100" || senstr=="11000" || senstr=="10000") //Left turn
  {
    left_turn(LOW_SPEED,HIGH_SPEED);
    delay(DELAY_TIME);
  }
  if ( senstr=="00111" || senstr=="00011" || senstr=="00001") //Right turn
  {
    right_turn(HIGH_SPEED,LOW_SPEED);
    delay(DELAY_TIME);
  }
  else
  {
    stop_bot();
  }


//  if ( senstr=="11111") //Stop
//  {
//    stop_bot();
//    delay(DELAY_TIME);
//  }
}
