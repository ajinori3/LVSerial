#ifndef LVSERIAL_H_
#define LVSERIAL_H_

#include "Arduino.h"

class LVSerial
{
public:
	enum class RegName
	{
		SYS_PN,
		SYS_VER,
		SYS_UID,
		SYS_SID,
		SYS_RST,
		SYS_BR,
		SYS_T0,
		SYS_RID,
		SYS_PCT,
		SYS_DCT,
		SYS_ECT,
		SYS_ULK,
		PWM_PSN,
		PWM_PSA,
		PWM_SST,
		PWM_SLP,
		PWM_PSO,
		M_POS,
		M_SPD,
		M_VE,
		M_TEMP,
		M_VI,
		M_IERR,
		FB_TPOS,
		FB_PG,
		FB_DG,
		FB_EG,
		FB_IG,
		FB_ILIM,
		FB_PDB,
		FB_DDB,
		FB_EDB,
		PWM_EN,
		PWM_LIM,
		PWM_PCH,
		PWM_OUT,
		AL_TEMP,
		SD_TEMP,
		AL_VI,
		SD_VI,
		VIB_OTH,
		VIB_STH,
		BST_DUM,
		BST_LEN,
		BST_SYN,
		BST_WA0,
		BST_WA1,
		BST_WA2,
		BST_WA3,
		BST_WA4,
		BST_WA5,
		BST_WA6,
		BST_WA7,
		BST_RA0,
		BST_RA1,
		BST_RA2,
		BST_RA3,
		BST_RA4,
		BST_RA5,
		BST_RA6,
		BST_RA7
	};
	
	explicit LVSerial(HardwareSerial &serial);
	LVSerial(HardwareSerial &serial, const long baud);
	
	bool begin();
	bool begin(const uint8_t servo_id);
	
	bool isConnected();
	void releaseWriteProtection();
	
	void powerOn();
	void powerOff();
	
	void writeTargetPos(const uint16_t raw_pos);
	float readPowerVoltage();
	uint16_t readNowPos();
	float readBackEMF();
	uint16_t readNowSpeed();
	
	uint8_t read1byteData();
	uint16_t read2byteData();
	uint32_t read4byteData();
	
	void write1byteData(const uint8_t data);
	void write2byteData(const uint16_t data);
	void write4byteData(const uint32_t data);
	
	void write(const RegName reg, int data);
	uint32_t read(const RegName reg);
		
	
private:
	HardwareSerial *serial_;
	uint8_t servo_id_;
	
	typedef struct RegElement_t
	{
		int address;
		int size;
		bool is_writeable;
	} RegElement_t;
	
	RegElement_t getRegisterSpecification(const RegName);		
	};

#endif // !LVSERIAL_H_