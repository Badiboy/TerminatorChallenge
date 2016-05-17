
#include <SoftwareSerial.h>
#include <Servo.h> 

Servo myservo;
int curPos = 90;
void setup()
{
	Serial.begin(19200);
	while (!Serial) {
		; // wait for serial port to connect. Needed for Leonardo only
	}
	myservo.attach(9);
	myservo.write(curPos);
	pinMode(13, OUTPUT);
	Serial.println("Start!");
}

void loop()
{
	if (Serial.available())
	{
		int k = Serial.read();
		if (k == '1')
		{
			digitalWrite(13, LOW);
			curPos -= 10;
		}
		if (k == '0')
		{
			digitalWrite(13, HIGH);
			curPos += 10;
		}
		if (k == 1)
		{
			digitalWrite(13, LOW);
			curPos -= 10;
		}
		if (k == 0)
		{
			digitalWrite(13, HIGH);
			curPos += 10;
		}
		myservo.write(curPos);
		Serial.println(k);
		delay(500);
	}
	delay(500);
}

