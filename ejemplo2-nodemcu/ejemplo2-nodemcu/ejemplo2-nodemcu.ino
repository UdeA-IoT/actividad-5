int ledPin = D7; // LED pin (D7 = GPIO17)
int Pot = A0;
int val;

void setup() {
  initPorts();
  initSerial();
}

void loop() {
  val = analogRead(Pot);
  analogWrite(ledPin, val); 
  Serial.print("Valor potenciometro: ");
  Serial.println(val);
}


/* Funciones de propias de usuario */
void initPorts() {
  pinMode(ledPin, OUTPUT);  
}

void initSerial() {
  Serial.begin(9600);
}
