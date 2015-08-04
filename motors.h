const int m1 = 10;
const int e1 = 9;
const int m2 = 5;
const int e2 = 6;

class motors{
  public:
    void setup_motors(void);
    void go(int dir, int on, int motorspeed);
};

void motors::setup_motors(void){
  pinMode(m1,  OUTPUT);
  pinMode(e1,  OUTPUT);
  pinMode(m2,  OUTPUT);
  pinMode(e2,  OUTPUT);
}

void motors::go(int dir, int on, int motorspeed){
  
  if(!on){
    //stop
    digitalWrite(e1, LOW);
    digitalWrite(e2, LOW);
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
    //Serial.println("stopping");
  }else{
    if(dir){
      //go forward
      analogWrite(e1, motorspeed);
      analogWrite(e2, motorspeed);
      digitalWrite(m1, LOW);
      digitalWrite(m2, LOW);
      //Serial.println("going forward");
    }else{
      analogWrite(e1, motorspeed);
      analogWrite(e2, motorspeed);
      digitalWrite(m1, HIGH);
      digitalWrite(m2, HIGH);      
      //Serial.println("going backward");
    }
  }
    
}
  
