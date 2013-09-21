int old_state[6];

void setup() {
  Serial.begin(57600);
  for(int i=0;i<6;++i) {
    int j=8+i;
    pinMode(j,INPUT);
    digitalWrite(j,HIGH);
    old_state[i]=-1;
  }
}


void loop() {
  for(int i=0;i<6;++i) {
    int j=8+i;
    int state=digitalRead(j);
    if(old_state[i] != state) {
      old_state[i] = state;
      Serial.print(j);
      Serial.print('=');
      Serial.println(state);
      delay(50);
    }
  }
}
