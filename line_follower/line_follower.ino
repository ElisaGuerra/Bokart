#define SOUND_SPEED 0.0343 //Speed of sound in cm/us
#define DISTANCE_FRONT 6 //Distance before stopping (in cm)
#define DISTANCE_SIDE 12 //Distance from objects to sides (in cm)
         
#define speedPinR 11 //Front Right motor pins
#define rightMotorDirPin1  5 
#define rightMotorDirPin2  6 
#define speedPinL 12 //Front Left motor pins                                
#define leftMotorDirPin1  7
#define leftMotorDirPin2  8
#define speedPinRB 9 //Rear Right motor pins
#define rightMotorDirPin1B  22
#define rightMotorDirPin2B 24 
#define speedPinLB 10 //Rear Left motor pins
#define leftMotorDirPin1B 26
#define leftMotorDirPin2B 28 

#define sensor1   A4 //Pins for IR Sensors 
#define sensor2   A3 
#define sensor3   A2 
#define sensor4   A1 
#define sensor5   A0 

#define triggerPinF 34 //Front ultrasonic sensor pins
#define echoPinF 21   
#define triggerPinL 40 //Left ultrasonic sensor pins
#define echoPinL 20 
#define triggerPinR 38 //Right ultrasonic sensor pins 
#define echoPinR 19 

#define buzzerPin 42 
#define ledR 46 
#define ledL 48 
#define buttonPin 44

float durationF=0.0; //Duration measured by front sensor
bool objectDetectedF = false; //Front object detection flag
float durationL=0.0; //Duration measured by left sensor
bool objectDetectedL = false; //Left object detection flag
float durationR=0.0; //Duration measured by right sensor
bool objectDetectedR = false; //Right object detection flag

float Kp = 220.0; //Control parameters and variables
float Ki = 0.3; 
float Kd = 100.0; 
float lastError = 0.0; //For derivative calculation
float integral = 0.0; //For integral calculation
int minSpeed = 50; //Speed constrains
int baseSpeed = 100;
int maxSpeed = 150;

//Motor control functions
void FR_fwd(int speed){  //Front Right wheel forward turn
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW); 
  analogWrite(speedPinR,speed);
}
void FR_bck(int speed){ //Front Right wheel backward turn
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH); 
  analogWrite(speedPinR,speed);
}
void FL_fwd(int speed){ //Front Left wheel forward turn
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
}
void FL_bck(int speed){ //Front Left wheel backward turn
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
}
void RR_fwd(int speed){ //Rear Right wheel forward turn
  digitalWrite(rightMotorDirPin1B, HIGH);
  digitalWrite(rightMotorDirPin2B,LOW); 
  analogWrite(speedPinRB,speed);
}
void RR_bck(int speed){ //Rear Right wheel backward turn
  digitalWrite(rightMotorDirPin1B, LOW);
  digitalWrite(rightMotorDirPin2B,HIGH); 
  analogWrite(speedPinRB,speed);
}
void RL_fwd(int speed){ //Rear Left wheel forward turn
  digitalWrite(leftMotorDirPin1B,HIGH);
  digitalWrite(leftMotorDirPin2B,LOW);
  analogWrite(speedPinLB,speed);
}
void RL_bck(int speed){ //Rear Left wheel backward turn
  digitalWrite(leftMotorDirPin1B,LOW);
  digitalWrite(leftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB,speed);
}

//Movement functions
void stop_bot(){    
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
void end_routine(){ //Final routine after detecting object in the front
  delay(500); //Delay after stopping the bot
  //Movement to the right
  FR_bck(100);
  FL_fwd(100);
  RR_fwd(100);
  RL_bck(100);

  delay(1000); //Move to the left for 1s

  //Right spin
  FR_fwd(100);
  FL_bck(100);
  RR_fwd(100);
  RL_bck(100);

  delay(1000); //Spin for 1s

  stop_bot();
}
//Pins initialize
void init_GPIO(){
  //Motors' pins
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
  //IR sensors' pins
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
  //Ultrasonic sensors' pins
  pinMode(triggerPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(triggerPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(triggerPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  //Buzzer, button and leds' pins
  pinMode(buzzerPin,OUTPUT);
  pinMode(buttonPin,INPUT);
  pinMode(ledR,OUTPUT);
  pinMode(ledL,OUTPUT);
  
  stop_bot();
}

//Control functions
void control(){
  int s0 = !digitalRead(sensor1);
  int s1 = !digitalRead(sensor2);
  int s2 = !digitalRead(sensor3);
  int s3 = !digitalRead(sensor4);
  int s4 = !digitalRead(sensor5);
  
  if( s0==1 && s1==1 && s2==1 && s3==1 && s4==1){ //Cross-line handle, go forward
    RL_fwd(baseSpeed);
    RR_fwd(baseSpeed);
    FR_fwd(baseSpeed);
    FL_fwd(baseSpeed); 
  }
  else if( s0==0 && s1==0 && s2==0 && s3==0 && s4==0){ //No line handle, go backwards
    RL_bck(baseSpeed);
    RR_bck(baseSpeed);
    FR_bck(baseSpeed);
    FL_bck(baseSpeed); 
  }
  else{ //Error and speed calculation for tracing the line
    float error = (s0*-4 + s1*-2 + s2*0 + s3*2 + s4*4); //Error calculation
    integral += error;
    float derivative = error - lastError;
    float output = Kp * error + Ki * integral + Kd * derivative; //Speed calculation
    lastError = error;
    int leftSpeed = constrain(minSpeed - output, 0, maxSpeed); //Constrains for speed
    int rightSpeed = constrain(minSpeed + output, 0, maxSpeed);
    
    setMotorSpeed(rightSpeed, leftSpeed);
  }
}
void setMotorSpeed(int rightSpeed, int leftSpeed) {//Asign speed calculations to right and left motors
  if (rightSpeed > 0) { //Go forward if speed is positive
    FL_fwd(rightSpeed);
    RL_fwd(rightSpeed);
  } else { //Go backwards if speed is negative
    rightSpeed = -rightSpeed; //Turn speed positive for pwm
    FL_bck(rightSpeed);
    RL_bck(rightSpeed);
  }
  if (leftSpeed > 0) { //Go forward if speed is positive
    FR_fwd(leftSpeed);
    RR_fwd(leftSpeed);
  } else { //Go backwards if speed is negative
    FR_bck(leftSpeed);
    RR_bck(leftSpeed);
    leftSpeed = -leftSpeed; //Turn speed positive for pwm
  }
}

//Ultrasonic sensors interrupt functions
void echoFront() { //Executed each time the echo pin receives a pulse
  if (digitalRead(echoPinF) == HIGH) { //The return of the ultrasonic signal was detected
    durationF = micros(); //Save current time in microseconds
  } 
  else { //Calculation of the time it took for the signal to return
    durationF = micros() - durationF; //Time between initial time and current time
    float distance = durationF * SOUND_SPEED / 2; //Distance calculation
    if (distance < DISTANCE_FRONT) { //Comparisson between the calculated distance and the set distance
      objectDetectedF = true; //Rises a flag if the object's distance from the sensor is less than the ser distance
    } 
    else {
      objectDetectedF = false;
    }
  }
}
void echoLeft() { //Executed each time the echo pin receives a pulse
  if (digitalRead(echoPinL) == HIGH) { //The return of the ultrasonic signal was detected
    durationL = micros(); //Save current time in microseconds
  } 
  else { //Calculation of the time it took for the signal to return
    durationL = micros() - durationL; //Time between initial time and current time
    float distance = durationL * SOUND_SPEED / 2;  //Distance calculation
    if (distance < DISTANCE_SIDE) { //Comparisson between the calculated distance and the set distance
      objectDetectedL = true; //Rises a flag if the object's distance from the sensor is less than the ser distance
    } 
    else {
      objectDetectedL = false;
    }
  }
}
void echoRight() { //Executed each time the echo pin receives a pulse
  if (digitalRead(echoPinR) == HIGH) { //The return of the ultrasonic signal was detected
    durationR = micros(); //Save current time in microseconds
  } 
  else { //Calculation of the time it took for the signal to return
    durationR = micros() - durationR; //Time between initial time and current time
    float distance = durationR * SOUND_SPEED / 2; //Distance calculation    
    if (distance < DISTANCE_SIDE) { //Comparisson between the calculated distance and the set distance
      objectDetectedR = true; //Rises a flag if the object's distance from the sensor is less than the ser distance
    } 
    else {
      objectDetectedR = false;
    }
  }
}

void setup(){
  init_GPIO();
  
  attachInterrupt(digitalPinToInterrupt(echoPinF), echoFront, CHANGE); //Interrupt setup for front ultrasonic sensor, triggered whenever the state of the pin changes
  attachInterrupt(digitalPinToInterrupt(echoPinL), echoLeft, CHANGE); //Interrupt setup for left ultrasonic sensor, triggered whenever the state of the pin changes
  attachInterrupt(digitalPinToInterrupt(echoPinR), echoRight, CHANGE); //Interrupt setup for right ultrasonic sensor, triggered whenever the state of the pin changes
  //Wait until button is pressed
//  delay(2000);
//  int buttonState=digitalRead(buttonPin);
//  while(buttonState==LOW){
//    buttonState=digitalRead(buttonPin);
//    delay(100);
//  }
  Serial.begin(9600);
  delay(15000);
}

void loop(){
  // Start front ultrasonic sensor lecture
  digitalWrite(triggerPinF, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPinF, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinF, LOW);
  delayMicroseconds(50);
  // Start left ultrasonic sensor lecture
  digitalWrite(triggerPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinL, LOW);
  delayMicroseconds(50);
  // Start right ultrasonic sensor lecture
  digitalWrite(triggerPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinR, LOW);
  delayMicroseconds(50);
  
  // Check object detection flags
  if (!objectDetectedF) { //No objects detected in the front of the bot
    if(objectDetectedR && !objectDetectedL){ //Left object detection
      tone(buzzerPin, 500);
      digitalWrite(ledR,HIGH);
      digitalWrite(ledL,LOW);
    }
    else if(objectDetectedL && !objectDetectedR){ //Left object detection
      tone(buzzerPin, 500); 
      digitalWrite(ledL,HIGH);
      digitalWrite(ledR,LOW);
    }
    else if(objectDetectedL && objectDetectedR){ //Left object detection
      tone(buzzerPin, 500); 
      digitalWrite(ledL,HIGH);
      digitalWrite(ledR,HIGH);
    }
    else{ //No objects detected in the sides of the bot
      digitalWrite(ledR,LOW);
      digitalWrite(ledL,LOW);
      noTone(buzzerPin);
    }
    control(); //Tracing the line with PID control
  } 
  else { //Front object detection
    stop_bot();
    end_routine();
    while(true){}
    
//    int buttonState=digitalRead(buttonPin);
//    while(buttonState==LOW){
//      buttonState=digitalRead(buttonPin);
//      delay(10);
//    }
  }
  //Serial.println(distance);
  delay(40);
//  int buttonState=digitalRead(buttonPin);
//  Serial.println(buttonState);
//  delay(500);
}
