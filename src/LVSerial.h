#ifndef LVSERIAL_H_
#define LVSERIAL_H_

#include "SoftwareSerial.h"

class LVSerial
{
public:
	LVSerial(SoftwareSerial serial);
	
	
private:
	SoftwareSerial serial_;
};

#endif // !LVSERIAL_H_
