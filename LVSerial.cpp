#include "LVSerial.hpp"

LVSerial::LVSerial(HardwareSerial &serial)
	: serial_(&serial)
	, servo_id_(0)
{
	serial_->begin(115200);
	serial_->setTimeout(10);
}

LVSerial::LVSerial(HardwareSerial &serial, const long baud)
	: serial_(&serial)
	, servo_id_(0)
{
	serial_->begin(baud);
	serial_->setTimeout(10);
}

bool  LVSerial::begin() {	
	releaseWriteProtection();
	return isConnected();
}
bool LVSerial::begin(const uint8_t servo_id)
{
	this->servo_id_ = servo_id;
	return begin();
}

bool LVSerial::isConnected()
{
	uint16_t sys_pn_val = read(RegName::SYS_PN);
	
	if (sys_pn_val != 0x0000)
	{
		return true;
	}
	else
	{
		return false;
	}	
}

void LVSerial::releaseWriteProtection()
{
	write(RegName::SYS_ULK, 0x55);	
}

void LVSerial::powerOn()
{
	write(RegName::PWM_EN, 0x01);
}

void LVSerial::powerOff()
{
	write(RegName::PWM_EN, 0x00);
}

void LVSerial::writeTargetPos(const uint16_t raw_pos)
{	
	return write(RegName::FB_TPOS, raw_pos);
}

float LVSerial::readPowerVoltage()
{
	uint16_t read_buff = read(RegName::M_VI);	
	return 27.5f * (float)read_buff / 4096.0f;
}

uint16_t LVSerial::readNowPos() {	
	return read(RegName::M_POS);
}

float LVSerial::readBackEMF()
{
	int16_t read_buff = read(RegName::M_VE);	
	if (read_buff > 0x2000) {
		read_buff -= 0x4000;
	}
	
	return 27.5f * (float)read_buff / 4096.f;
}

int16_t LVSerial::readNowSpeed()
{	
	int16_t read_buff = read(RegName::M_SPD);
	if (read_buff > 0x2000) {
		read_buff -= 0x4000;
	}
	return read_buff;
}


uint8_t LVSerial::read1byteData() {
	uint8_t read_buff = 0;
	
	serial_->readBytes(&read_buff, sizeof(read_buff));
	
	return (read_buff & 0x7f);
}
uint16_t LVSerial::read2byteData() {
	uint8_t read_buff[2] = {};
	uint16_t read_value = 0;
	
	serial_->readBytes(read_buff, sizeof(read_buff));
	
	read_value = 
		(read_buff[0] & 0x7f) + 
		(((uint32_t)read_buff[1] & 0x7f) << 7);
	
	return read_value;
}
uint32_t LVSerial::read4byteData() {
	uint8_t read_buff[4] = { };
	uint32_t read_value = 0;
	
	serial_->readBytes(read_buff, sizeof(read_buff));
	
	read_value = 
		(read_buff[0] & 0x7F) +
		(((uint32_t)read_buff[1] & 0x7f) << 7) +
		(((uint32_t)read_buff[2] & 0x7f) << 14) +
		(((uint32_t)read_buff[3] & 0x7f) << 21);
		
	return read_value;
}

	
void LVSerial::write1byteData(const uint8_t data) {
	uint8_t write_data = data & 0x7f;
	serial_->write(write_data);
}

void LVSerial::write2byteData(const uint16_t data) {
	uint8_t write_data[2] = 
	{
		(uint16_t)data & 0x7f,
		((uint16_t)data >> 7) & 0x7f
	};
	
	serial_->write(write_data, sizeof(write_data));
}
void LVSerial::write4byteData(const uint32_t data) {
	uint8_t write_data[4] = 
	{
		(uint16_t)data & 0x7f,
		((uint32_t)data >> 7) & 0x7f,
		((uint32_t)data >> 14) & 0x7f,
		((uint32_t)data >> 21) & 0x7f		
	};
	
	serial_->write(write_data, sizeof(write_data));
}

void LVSerial::write(const RegName reg, const int data)
{
	uint8_t header_data[] = {
		0x80 | servo_id_,
		0x40 | getRegisterSpecification(reg).size,
		getRegisterSpecification(reg).address
	};
	
	serial_->write(header_data, sizeof(header_data));	
	switch (getRegisterSpecification(reg).size)
	{
	case 1:
		write1byteData(data);
		break;
	case 2:
		write2byteData(data);
		break;
	case 4:
		write4byteData(data);
		break;
	}
	
	serial_->flush();
}

uint32_t LVSerial::read(const RegName reg)
{	
	uint32_t read_value = 0;
	constexpr uint32_t DUMMY_DATA = 0x00;
	
	uint8_t header_data[] = {
		0x80 | servo_id_,
		0x20 | getRegisterSpecification(reg).size,
		getRegisterSpecification(reg).address
	};
	
	serial_->write(header_data, sizeof(header_data));
	serial_->flush();
	
	switch (getRegisterSpecification(reg).size)
	{
	case 1:
		write1byteData(DUMMY_DATA);
		read_value = read1byteData();
		break;
	case 2:
		write2byteData(DUMMY_DATA);
		read_value = read2byteData();
		break;
	case 4:
		write4byteData(DUMMY_DATA);
		read_value = read4byteData();
		break;
	}	
	return read_value;
}


LVSerial::RegElement_t LVSerial::getRegisterSpecification(RegName reg)
{
	constexpr RegElement_t REG_SPECIFICATION[] = {
		{ 0x00, 2, false },
		{ 0x02, 2, false },
		{ 0x04, 4, false },
		{ 0x08, 1, true },
		{ 0x09, 1, true },
		{ 0x0a, 2, true },
		{ 0x0c, 2, true },
		{ 0x0e, 1, true },
		{ 0x10, 1, true },
		{ 0x11, 1, true },
		{ 0x12, 1, true },
		{ 0x14, 1, true },
		{ 0x18, 1, true },
		{ 0x19, 1, true },
		{ 0x1a, 2, true },
		{ 0x1c, 2, true },
		{ 0x1E, 1, true },
		{ 0x20, 2, false },
		{ 0x22, 2, false },
		{ 0x24, 2, false },
		{ 0x26, 2, false },
		{ 0x28, 2, false },
		{ 0x2a, 2, false },
		{ 0x30, 2, true },
		{ 0x32, 1, true },
		{ 0x33, 1, true },
		{ 0x34, 1, true },
		{ 0x35, 1, true },
		{ 0x36, 2, true },
		{ 0x38, 1, true },
		{ 0x39, 1, true },
		{ 0x3a, 1, true },
		{ 0x3b, 1, true },
		{ 0x3c, 1, true },
		{ 0x3d, 1, true },
		{ 0x3e, 2, false },
		{ 0x40, 2, true },
		{ 0x42, 2, true },
		{ 0x44, 2, true },
		{ 0x46, 2, true },
		{ 0x48, 1, true },
		{ 0x49, 1, true },
		{ 0x4d, 1, true },
		{ 0x4e, 1, true },
		{ 0x4f, 1, true },
		{ 0x50, 1, true },
		{ 0x51, 1, true },
		{ 0x52, 1, true },
		{ 0x53, 1, true },
		{ 0x54, 1, true },
		{ 0x55, 1, true },
		{ 0x56, 1, true },
		{ 0x57, 1, true },
		{ 0x58, 1, true },
		{ 0x59, 1, true },
		{ 0x5a, 1, true },
		{ 0x5b, 1, true },
		{ 0x5c, 1, true },
		{ 0x5d, 1, true },
		{ 0x5e, 1, true },
		{ 0x5f, 1, true }		
	};
	
	return REG_SPECIFICATION[static_cast<int>(reg)];
}