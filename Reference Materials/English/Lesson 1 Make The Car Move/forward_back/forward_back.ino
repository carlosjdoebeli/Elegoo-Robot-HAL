//www.elegoo.com

//    Left motor truth table
//  ENA         IN1               IN2         Description  
//  LOW   Not Applicable    Not Applicable    Motor is off
//  HIGH        LOW               LOW         Motor is stopped (brakes)
//  HIGH        HIGH              LOW         Motor is on and turning forwards
//  HIGH        LOW               HIGH        Motor is on and turning backwards
//  HIGH        HIGH              HIGH        Motor is stopped (brakes)

//    Right motor truth table
//  ENB         IN3             IN4         Description  
//  LOW   Not Applicable   Not Applicable   Motor is off
//  HIGH        LOW             LOW         Motor is stopped (brakes)
//  HIGH        LOW             HIGH        Motor is on and turning forwards
//  HIGH        HIGH            LOW         Motor is on and turning backwards
//  HIGH        HIGH            HIGH        Motor is stopped (brakes)  

//    The direction of the car's movement
//  Left motor    Right motor     Description
//  stop(off)     stop(off)       Car is stopped
//  forward       forward         Car is running forwards
//  forward       backward        Car is turning right
//  backward      forward         Car is turning left
//  backward      backward        Car is running backwards

//define the L298n IO pin
#define ENB 5
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define ENA 6

void setup() {
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}
void loop() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  //go forward
  delay(1000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);   //stop
  delay(1000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   //go back
  delay(1000);
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);  //stop
  delay(1000);
}
