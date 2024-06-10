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
#define LOW_SPEED 80    
#define LONG_DELAY_TIME 70 
#define DELAY_TIME 40
#define SHORT_DELAY_TIME 10 
#define DISTANCE_FRONT 10 // Distance before stopping (in cm)
#define DISTANCE_SIDE 10 // Distance from objects to sides
         
#define speedPinR 11 //9   //  Front Wheel PWM pin connect Model-Y M_B ENA 
#define rightMotorDirPin1  5 //22    //Front Right Motor direction pin 1 to Model-Y M_B IN1  (K1) Izquierda atras
#define rightMotorDirPin2  6 //24   //Front Right Motor direction pin 2 to Model-Y M_B IN2   (K1)                                 
#define leftMotorDirPin1  7//26    //Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3) derecha atras
#define leftMotorDirPin2  8//28   //Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define speedPinL 12//10   //  Front Wheel PWM pin connect Model-Y M_B ENB

#define speedPinRB 9//12   //  Rear Wheel PWM pin connect Left Model-Y M_A ENA 
#define rightMotorDirPin1B  22//7    //Rear Right Motor direction pin 1 to Model-Y M_A IN1 ( K1) izquierda adelante
#define rightMotorDirPin2B 24//8    //Rear Right Motor direction pin 2 to Model-Y M_A IN2 ( K1) 
#define leftMotorDirPin1B 26 //5    //Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3) Derecha adelante
#define leftMotorDirPin2B 28 //6  //Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define speedPinLB 10 //11    //  Rear Wheel PWM pin connect Model-Y M_A ENB

#define sensor1   A4 // Left most sensor
#define sensor2   A3 // 2nd Left   sensor
#define sensor3   A2 // Center sensor
#define sensor4   A1 // 2nd right sensor// Right most sensor
#define sensor5   A0 // Right most sensor

#define triggerPinF 34 // Trigger for front ultrasonic sensor 
#define echoPinF 21 // Echo for front ultrasonic sensor, this is an interrupt pin
#define triggerPinL 10 // Trigger for left ultrasonic sensor //Revisar el pin 
#define echoPinL 20 // Echo for left ultrasonic sensor, this is an interrupt pin
#define triggerPinR 11 // Trigger for right ultrasonic sensor //Revisar el pin 
#define echoPinR 19 // Echo for right ultrasonic sensor, this is an interrupt pin

#define buzzerPin 40 // REVISAR PINES
#define ledR1 1
#define ledR2 1
#define ledR3 1
#define ledL1 1
#define ledL2 1
#define ledL3 1

long durationF; // Duration measured by front sensor
bool objectDetectedF = false; // Front object detection flag
long distanceF=0; // Front distance of an object
long durationL; // Duration measured by left sensor
bool objectDetectedL = false; // Left object detection flag
long durationR; // Duration measured by right sensor
bool objectDetectedR = false; // Right object detection flag
int led=0; //Manage led sequence

float Kp = 220.0; //Incrementar si el robot es lento para corregir su trayectoria
float Ki = 0.3; //Incrementar si el robot tiene un error constante al seguir la línea
float Kd = 100.0; //Incrementar si el robot oscila demasiado
float lastError = 0.0;
float integral = 0.0;
int baseSpeed = 120;
int maxSpeed = 230;

//Motor control functions
void FR_fwd(int speed){  //front-right wheel forward turn
  digitalWrite(rightMotorDirPin1,HIGH);
  digitalWrite(rightMotorDirPin2,LOW); 
  analogWrite(speedPinR,speed);
}
void FR_bck(int speed){ // front-right wheel backward turn
  digitalWrite(rightMotorDirPin1,LOW);
  digitalWrite(rightMotorDirPin2,HIGH); 
  analogWrite(speedPinR,speed);
}
void FL_fwd(int speed){ // front-left wheel forward turn
  digitalWrite(leftMotorDirPin1,HIGH);
  digitalWrite(leftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
}
void FL_bck(int speed){ // front-left wheel backward turn
  digitalWrite(leftMotorDirPin1,LOW);
  digitalWrite(leftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
}

void RR_fwd(int speed){  //rear-right wheel forward turn
  digitalWrite(rightMotorDirPin1B, HIGH);
  digitalWrite(rightMotorDirPin2B,LOW); 
  analogWrite(speedPinRB,speed);
}
void RR_bck(int speed){  //rear-right wheel backward turn
  digitalWrite(rightMotorDirPin1B, LOW);
  digitalWrite(rightMotorDirPin2B,HIGH); 
  analogWrite(speedPinRB,speed);
}
void RL_fwd(int speed){  //rear-left wheel forward turn
  digitalWrite(leftMotorDirPin1B,HIGH);
  digitalWrite(leftMotorDirPin2B,LOW);
  analogWrite(speedPinLB,speed);
}
void RL_bck(int speed){    //rear-left wheel backward turn
  digitalWrite(leftMotorDirPin1B,LOW);
  digitalWrite(leftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB,speed);
}

//Movement functions
void forward(int speed_left,int speed_right){
   RL_fwd(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_fwd(speed_left); 
}
void right_turn(int speed_left,int speed_right){
   RL_fwd(speed_left);
   RR_bck(speed_right);
   FR_bck(speed_right);
   FL_fwd(speed_left);
}
void left_turn(int speed_left,int speed_right){
   RL_bck(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_bck(speed_left);
}
void reverse(int speed){
   RL_bck(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_bck(speed); 
}
void right(int speed){
   RL_fwd(speed);
   RR_bck(0);
   FR_bck(0);
   FL_fwd(speed); 
}
void stop_bot(){    //Stop
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
  delay(DELAY_TIME);
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
  pinMode(ledR1,OUTPUT);
  pinMode(ledR2,OUTPUT);
  pinMode(ledR3,OUTPUT);
  pinMode(ledL1,OUTPUT);
  pinMode(ledL2,OUTPUT);
  pinMode(ledL3,OUTPUT);
  
  stop_bot();
}

void setup(){
  init_GPIO();
  attachInterrupt(digitalPinToInterrupt(echoPinF), echoFront, CHANGE); //Interrupt setup for front ultrasonic sensor
  //attachInterrupt(digitalPinToInterrupt(echoPinL), echoLeft, CHANGE); //Interrupt setup for left ultrasonic sensor
  //attachInterrupt(digitalPinToInterrupt(echoPinR), echoRight, CHANGE); //Interrupt setup for right ultrasonic sensor
  Serial.begin(9600);
}

void loop(){
  // Start front ultrasonic sensor lecture
  digitalWrite(triggerPinF, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPinF, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinF, LOW);
  delayMicroseconds(50);
//  // Start left ultrasonic sensor lecture
//  digitalWrite(triggerPinL, LOW);
//  delayMicroseconds(2);
//  digitalWrite(triggerPinL, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(triggerPinL, LOW);
//  delayMicroseconds(50);
//  // Start right ultrasonic sensor lecture
//  digitalWrite(triggerPinR, LOW);
//  delayMicroseconds(2);
//  digitalWrite(triggerPinR, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(triggerPinR, LOW);
//  delayMicroseconds(50);
// 
  // Check object detection flag
  if (!objectDetectedF) {
//    if(objectDetectedR){
//      tone(buzzerPin, 1000); // 1kHz sound signal
//      led=1;
//      rightLights();
//    }
//    else if(objectDetectedL){
//      tone(buzzerPin, 1000); // 1kHz sound signal
//      led=1;
//      leftLights();
//    }
//    else{
//      digitalWrite(ledR1,LOW);
//      digitalWrite(ledR2,LOW);
//      digitalWrite(ledR3,LOW);
//      digitalWrite(ledL1,LOW);
//      digitalWrite(ledL2,LOW);
//      digitalWrite(ledL3,LOW);
//      noTone(buzzerPin);
//    }
    //tracking();
    control(); //USANDO ESTA FUNCIÓN YA NO SE NECESITAN LAS FUNCIONES DE MOVIMIENTO
  } 
  else {
    stop_bot();
  }
  delay(DELAY_TIME);//Revisar si no afecta pwm
}

void tracking(){
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
  
  if ( senstr=="01100" || senstr=="00110" || senstr=="01110" || senstr=="00100" || senstr=="00010" || senstr=="01000" || senstr=="11110" || senstr=="01111" || senstr=="11111"){ //Forward
    forward(HIGH_SPEED,HIGH_SPEED);
    delay(DELAY_TIME);
  }
  if ( senstr=="11100" || senstr=="11000" || senstr=="10000"){ //Left turn
    left_turn(LOW_SPEED,HIGH_SPEED);
    delay(DELAY_TIME);
  }
  if ( senstr=="00111" || senstr=="00011" || senstr=="00001"){ //Right turn
    right_turn(HIGH_SPEED,LOW_SPEED);
    delay(DELAY_TIME);
  }
  if ( senstr=="00000" || senstr=="10001" || senstr=="11011" || senstr=="10011" || senstr=="11001" || senstr=="11101" || senstr=="10111"){//Reverse
    reverse(MID_SPEED);
    delay(DELAY_TIME);
  }
  else{
    stop_bot();
  }
}
void rightLights(){
  if(led==1){
    digitalWrite(ledR1,HIGH);
    led=0;
  }
  else if(led==2){
    digitalWrite(ledR2,HIGH);
    led=3;
  }
  else if(led==3){
    digitalWrite(ledR3,HIGH);
    led=0;
  }
  else if (led==0){
    digitalWrite(ledR1,LOW);
    digitalWrite(ledR2,LOW);
    digitalWrite(ledR3,LOW);
  }
  delay(SHORT_DELAY_TIME);
}
void leftLights(){
  if(led==1){
    digitalWrite(ledL1,HIGH);
    led=0;
  }
  else if(led==2){
    digitalWrite(ledL2,HIGH);
    led=3;
  }
  else if(led==3){
    digitalWrite(ledL3,HIGH);
    led=0;
  }
  else if (led==0){
    digitalWrite(ledL1,LOW);
    digitalWrite(ledL2,LOW);
    digitalWrite(ledL3,LOW);
  }
  delay(SHORT_DELAY_TIME);
}
void echoFront() { // Interrupt function for front ultrasonic sensor
  if (digitalRead(echoPinF) == HIGH) {
    durationF = micros();
  } 
  else {
    durationF = micros() - durationF;
    distanceF = durationF * 0.0343 / 2;
    if (distanceF < DISTANCE_FRONT) {
      objectDetectedF = true;
    } 
    else {
      objectDetectedF = false;
    }
  }
}
void echoLeft() { // Interrupt function for left ultrasonic sensor
  if (digitalRead(echoPinL) == HIGH) {
    durationL = micros();
  } 
  else {
    durationL = micros() - durationL;
    long distance = durationL * 0.0343 / 2;
    if (distance < DISTANCE_SIDE) {
      objectDetectedL = true;
    } 
    else {
      objectDetectedL = false;
    }
  }
}
void echoRight() { // Interrupt function for right ultrasonic sensor
  if (digitalRead(echoPinR) == HIGH) {
    durationR = micros();
  } 
  else {
    durationR = micros() - durationR;
    long distance = durationR * 0.0343 / 2;
    if (distance < DISTANCE_SIDE) {
      objectDetectedR = true;
    } 
    else {
      objectDetectedR = false;
    }
    //Serial.println(distance);
  }
}
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
  else if( s0==0 && s1==0 && s2==0 && s3==0 && s4==0){//No line handle
    RL_bck(baseSpeed);
    RR_bck(baseSpeed);
    FR_bck(baseSpeed);
    FL_bck(baseSpeed); 
  }
  else{
    float error = (s0*-4 + s1*-2 + s2*0 + s3*2 + s4*4);
    integral += error;
    float derivative = error - lastError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    int leftSpeed = constrain(baseSpeed - output, 0, maxSpeed);
    int rightSpeed = constrain(baseSpeed + output, 0, maxSpeed);
  
    Serial.print(leftSpeed);
    Serial.print(" ");
    Serial.println(rightSpeed);
  
    setMotorSpeed(rightSpeed, leftSpeed);
  }
  //delay(50);//REVISAR QUE NO AFECTE EL PWM
}
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motors
  if (leftSpeed > 0) { // Go forward
    FL_fwd(leftSpeed);
    RL_fwd(leftSpeed);
  } else { // Go backwards
    leftSpeed = -leftSpeed; // Turn speed positive for pwm
    FL_bck(leftSpeed);
    RL_bck(leftSpeed);
  }
  // Right motors
  if (rightSpeed > 0) { // Go forward
    FR_fwd(rightSpeed);
    RR_fwd(rightSpeed);
  } else { // Go backwards
    FR_bck(rightSpeed);
    RR_bck(rightSpeed);
    rightSpeed = -rightSpeed; // Turn speed positive for pwm
  }
}
