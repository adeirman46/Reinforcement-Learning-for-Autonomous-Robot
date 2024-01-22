#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM_R 5
#define PWM_L 6
#define ENA_R 7
#define ENA_L 8

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM_R,OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(ENA_R,OUTPUT);
  pinMode(ENA_L,OUTPUT);
  
}

void loop() {

  // SET TARGET 
  //int target = 1200;
  int target = 200*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0;
  float ki = 0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0  ; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = target - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  
}

void setMotor(int dir, int pwmVal){
  
  if(dir == 1){
    analogWrite(PWM_R,pwmVal);
    analogWrite(PWM_L, 0);
    digitalWrite(ENA_R,HIGH);
    digitalWrite(ENA_L,HIGH);
  }
  else if(dir == -1){
    analogWrite(PWM_L,pwmVal);
    analogWrite(PWM_R, 0);
    digitalWrite(ENA_R,HIGH);
    digitalWrite(ENA_L,HIGH);
  }
  else{
    digitalWrite(ENA_R,LOW);
    digitalWrite(ENA_L,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

////// hallo