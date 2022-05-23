
#include <Adafruit_Fingerprint.h>
SoftwareSerial mySerial(2, 3);
boolean a = false;
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

void setup()
{
  pinMode(8, OUTPUT);
  Serial.begin(9600);
  while (!Serial);
  delay(100);
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    //Serial.println("Found fingerprint sensor!");
  } else {
    //Serial.println("Did not find fingerprint sensor :(");
    while (1) {
      delay(1);
    }
  }

  finger.getTemplateCount();
}

void loop()
{
  getFingerprintIDez();
  delay(50);
}

int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  // found a match!

  int fingerprint = finger.confidence;
  if (fingerprint >= 10) {
    if (a == false) {
      digitalWrite(8, HIGH);
      a = true;
    }
    else if (a == true) {
      digitalWrite(8, LOW);
      a = false;
    }
  }
  return finger.fingerID;
}
