#define LONG_DELAY_TIME 70 
#define DELAY_TIME 40
#define SOUND_SPEED 0.0343
#define DISTANCE_FRONT 10 //Distance before stopping (in cm)
#define DISTANCE_SIDE 10 //Distance from objects to sides (in cm)
         
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

#define sensor1   A4 // IR Sensors
#define sensor2   A3 
#define sensor3   A2 
#define sensor4   A1 
#define sensor5   A0 

#define triggerPinF 34 //Front ultrasonic sensor pins
#define echoPinF 21 
#define triggerPinL 10 //Left ultrasonic sensor pins
#define echoPinL 20 
#define triggerPinR 11 //Right ultrasonic sensor pins 
#define echoPinR 19 

#define buzzerPin 40 
#define ledR 1
#define ledL 1

float durationF=0.0; // Duration measured by front sensor
bool objectDetectedF = false; // Front object detection flag
float durationL=0.0; // Duration measured by left sensor
bool objectDetectedL = false; // Left object detection flag
float durationR=0.0; // Duration measured by right sensor
bool objectDetectedR = false; // Right object detection flag

float Kp = 220.0; //Control parameters and variables
float Ki = 0.3; 
float Kd = 100.0; 
float lastError = 0.0;
float integral = 0.0;
int baseSpeed = 120;
int maxSpeed = 230;

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

//Pins initialize
void init_GPIO(){
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
  
  pinMode(triggerPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(triggerPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(triggerPinR, OUTPUT);
  pinMode(echoPinR, INPUT);

  pinMode(buzzerPin,OUTPUT);
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
  
  if( s0==1 && s1==1 && s2==1 && s3==1 && s4==1){ //Cross-line handle
    RL_fwd(baseSpeed);
    RR_fwd(baseSpeed);
    FR_fwd(baseSpeed);
    FL_fwd(baseSpeed); 
  }
  else if( s0==0 && s1==0 && s2==0 && s3==0 && s4==0){ //No line handle
    RL_bck(baseSpeed);
    RR_bck(baseSpeed);
    FR_bck(baseSpeed);
    FL_bck(baseSpeed); 
  }
  else{
    float error = (s0*-4 + s1*-2 + s2*0 + s3*2 + s4*4); //Error calculation
    integral += error;
    float derivative = error - lastError;
    float output = Kp * error + Ki * integral + Kd * derivative; //Speed calculation
    lastError = error;
    int leftSpeed = constrain(baseSpeed - output, 0, maxSpeed);
    int rightSpeed = constrain(baseSpeed + output, 0, maxSpeed);
    
    setMotorSpeed(rightSpeed, leftSpeed);
  }
}
void setMotorSpeed(int rightSpeed, int leftSpeed) {
  if (rightSpeed > 0) { //Go forward
    FL_fwd(rightSpeed);
    RL_fwd(rightSpeed);
  } else { // Go backwards
    rightSpeed = -rightSpeed; //Turn speed positive 
    FL_bck(rightSpeed);
    RL_bck(rightSpeed);
  }
  if (leftSpeed > 0) { //Go forward
    FR_fwd(leftSpeed);
    RR_fwd(leftSpeed);
  } else { // Go backwards
    FR_bck(leftSpeed);
    RR_bck(leftSpeed);
    leftSpeed = -leftSpeed; //Turn speed positive 
  }
}

//Ultrasonic sensors interrupt functions
void echoFront() { 
  if (digitalRead(echoPinF) == HIGH) {
    durationF = micros();
  } 
  else {
    durationF = micros() - durationF;
    float distance = durationF * SOUND_SPEED / 2;
    if (distance < DISTANCE_FRONT) {
      objectDetectedF = true;
    } 
    else {
      objectDetectedF = false;
    }
  }
}
void echoLeft() { 
  if (digitalRead(echoPinL) == HIGH) {
    durationL = micros();
  } 
  else {
    durationL = micros() - durationL;
    float distance = durationL * SOUND_SPEED / 2;
    if (distance < DISTANCE_SIDE) {
      objectDetectedL = true;
    } 
    else {
      objectDetectedL = false;
    }
  }
}
void echoRight() { 
  if (digitalRead(echoPinR) == HIGH) {
    durationR = micros();
  } 
  else {
    durationR = micros() - durationR;
    float distance = durationR * SOUND_SPEED / 2;
    if (distance < DISTANCE_SIDE) {
      objectDetectedR = true;
    } 
    else {
      objectDetectedR = false;
    }
  }
}

void setup(){
  init_GPIO();
  attachInterrupt(digitalPinToInterrupt(echoPinF), echoFront, CHANGE); //Interrupt setup for front ultrasonic sensor
  attachInterrupt(digitalPinToInterrupt(echoPinL), echoLeft, CHANGE); //Interrupt setup for left ultrasonic sensor
  attachInterrupt(digitalPinToInterrupt(echoPinR), echoRight, CHANGE); //Interrupt setup for right ultrasonic sensor
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
  if (!objectDetectedF) {
    if(objectDetectedR){
      tone(buzzerPin, 1800);
      digitalWrite(ledR,HIGH);
    }
    else if(objectDetectedL){
      tone(buzzerPin, 1800); 
      digitalWrite(ledL,HIGH);
    }
    else{
      digitalWrite(ledR,LOW);
      digitalWrite(ledL,LOW);
      noTone(buzzerPin);
    }
    control(); 
  } 
  else {
    stop_bot();
  }
  delay(40);
}
