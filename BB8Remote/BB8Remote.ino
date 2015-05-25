int ch1;
int ch2;

void setup() {

  pinMode(2, INPUT); // Set our input pins as such
  pinMode(3, INPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  Serial.begin(115200); 
}

void loop() {

  ch1 = pulseIn(2, HIGH, 25000); // Read the pulse width of 
  ch2 = pulseIn(3, HIGH, 25000); // each channel
  
  if (ch1<1300 && ch1>100){
    //Serial.println("LEFT");
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
  }
  else if (ch1>1600){
    //Serial.println("RIGHT");
    digitalWrite(6, HIGH);
    digitalWrite(5, LOW);
  }
  else {
    digitalWrite(6, LOW);
    digitalWrite(5, LOW);
    }
    
  if (ch2<1300 && ch1>100){
    //Serial.println("BACKWARD");
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
  }
  else if (ch2>1600) {
    //Serial.println("FORWARD");
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
  }
    else {
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    }

  //delay(200);
}
