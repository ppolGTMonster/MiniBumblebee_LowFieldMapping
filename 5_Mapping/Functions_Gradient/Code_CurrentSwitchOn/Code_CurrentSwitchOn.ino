#define PinRelay 1
#define ledpin 13


unsigned long tTotal;
unsigned long t_old = millis();
bool switchFlag = false;
int timeOn = 5000;  // Miliseconds


String inputString = "";
int inVal = 0;



void setup() {
  Serial.begin(9600);
  pinMode(PinRelay, OUTPUT);
  pinMode(ledpin, OUTPUT);
  Serial.setTimeout(1);
}


void loop() {

  while (Serial.available() == false) {
    tTotal = millis();

    if ((tTotal - t_old) > timeOn) {
      switchOff();
    };
  }

  inputString = Serial.readStringUntil('\n');
  inVal = inputString.toInt();

  if (inVal > 1000) {
    timeOn = inVal;
  } else {
    timeOn = 10000;
  }

  if (inVal >= 1) {
    // Gradient relay control
    t_old = millis();
    switchOn();

  } else {
    Serial.flush();
    switchOff();
  }
}



void switchOn() {
  digitalWrite(PinRelay, HIGH);
  digitalWrite(ledpin, HIGH);
  Serial.println("Relay HIGH");
}


void switchOff() {

  digitalWrite(PinRelay, LOW);
  digitalWrite(ledpin, LOW);
  Serial.println("Relay LOW");
}
