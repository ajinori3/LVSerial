#include <LVSerial.hpp>

LVSerial my_servo(Serial1);

void setup()
{
	Serial.begin(115200);
	while (my_servo.init(0x00) != LVSerial::ErrorStatus::OK)
	{
		Serial.println("connecting...");
		delay(100);
	}
}

void loop()
{
	uint16_t pos;
	if (my_servo.readNowPos(&pos) != LVSerial::ErrorStatus::OK)
	{
		Serial.println(pos);
	}
	else
	{
		Serial.println("Connection Error");
	}
	
	delay(100);
}