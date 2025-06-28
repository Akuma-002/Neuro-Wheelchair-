const int eegPin = A0;   
int eegValue = 0;

void setup() {
  Serial.begin(115200);  
  pinMode(eegPin, INPUT);
}

void loop() {
  eegValue = analogRead(eegPin);  
  Serial.println(eegValue);      
  delay(1);                       
}
