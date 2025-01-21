int motorR_PWM = 37;
int motorR_phase = 38;
int motorL_PWM = 39 ;
int motorL_phase = 20 ;

int distance_sensor = 16;
int distance = 0;

void stop_motor(){
  analogWrite(motorR_PWM,0);
  analogWrite(motorL_PWM,0); 
  Serial.println("Motors stopped");
  delay(2000);
}

void turn(){
  digitalWrite(motorR_phase,LOW);
  digitalWrite(motorL_phase,HIGH);
  analogWrite(motorR_PWM, 120);  // set speed of motor 
  analogWrite(motorL_PWM,115);
  Serial.println("turning");
  delay(950);
}

void setup() {
  Serial.begin(9600);
  pinMode(motorR_PWM,OUTPUT);
  pinMode(motorL_PWM,OUTPUT);
  pinMode(motorR_phase,OUTPUT);
  pinMode(motorL_phase,OUTPUT);

  pinMode(distance_sensor,INPUT);

}

void loop() {
  if (distance > 2800){
    stop_motor();
    turn();
  }

  digitalWrite(motorR_phase, LOW);  //forward 
  digitalWrite(motorL_phase,LOW);
  analogWrite(motorR_PWM, 150);  // set speed of motor 
  analogWrite(motorL_PWM,148);
  Serial.println("Forward");  // Display motor direction  
   
  /*digitalWrite(motorR_phase, HIGH);  //Backward 
  digitalWrite(motorL_phase, HIGH); 
  analogWrite(motorR_PWM, 100);   // set speed of motor 
  analogWrite(motorL_PWM, 97);    
  Serial.println("Backward");  // Display motor direction 
  delay(5000);   */

  distance = analogRead(distance_sensor);
  Serial.println(distance);
  delay(5);

}
