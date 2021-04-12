#include "LVSerial.h"

LVSerial::LVSerial(SoftwareSerial serial)
	: serial_(serial)
	, servo_id_(0)
{
	serial_.begin(115200);
}

LVSerial::LVSerial(SoftwareSerial serial, long baud)
	: serial_(serial)
	, servo_id_(0)
{
	serial_.begin(baud);
}


bool  LVSerial::readRAM(const RegName reg, uint8_t* read_buff, const size_t buff_size)
{
	//LVSerial protocol need write some data even if only need to read data
	size_t reg_size = getRegisterSpecification(reg).size;
	uint8_t write_dummy[buff_size];
		
	if (reg_size > buff_size)
	{
		return false;
	}		
		
	return transmitReceiveToRAM(reg, write_dummy, read_buff, false);
}

bool LVSerial::writeRAM(RegName reg, uint8_t* write_buff, size_t buff_size) {
	size_t reg_size = getRegisterSpecification(reg).size;	
	bool is_writable = getRegisterSpecification(reg).is_writeable;
	uint8_t dummy_read_buff[buff_size];
		
	if (reg_size > buff_size || !is_writable)
	{
		return false;
	}
		
	return transmitReceiveToRAM(reg, write_buff, dummy_read_buff, true);
}

bool  LVSerial::init() {
	if (!isConnected())
	{
		return false;
	}
	
//	if (!releaseWriteProtection(true))
//	{
//		return false;
//	}
	
	return true;
}
bool LVSerial::init(uint8_t servo_id)
{
	this->servo_id_ = servo_id;
	return init();
}

bool LVSerial::isConnected()
{
	uint16_t sys_pn_val = 0; 
	if (!readRAM(LVSerial::RegName::SYS_PN, reinterpret_cast<uint8_t*>(&sys_pn_val), sizeof(sys_pn_val))) {
		return false;
	}
	return sys_pn_val != 0x0000;
}

bool LVSerial::releaseWriteProtection(const bool is_enable)
{
	const uint8_t unlock_key = 0x55;
	uint8_t write_val;
	
	is_enable ? write_val = unlock_key : write_val = 0x00;
	return writeRAM(LVSerial::RegName::SYS_ULK, &write_val, sizeof(write_val));
}

bool LVSerial::enableServoPower(const bool is_enable)
{
	uint8_t write_val;
	is_enable ? write_val = 0x01 : write_val = 0x00;
	
	return writeRAM(LVSerial::RegName::PWM_EN, &write_val, sizeof(write_val));
}

bool LVSerial::writeTargetPos(const uint16_t raw_pos)
{ 
	uint16_t target_pos_val = raw_pos;
	return writeRAM(LVSerial::RegName::FB_TPOS, reinterpret_cast<uint8_t*>(&target_pos_val), sizeof(target_pos_val));
}

float LVSerial::readPowerVoltage()
{
	uint16_t read_buff;
	if (!readRAM(LVSerial::RegName::M_VI, reinterpret_cast<uint8_t*>(&read_buff), sizeof(read_buff))) 
	{
		return 0.0f;
	}
	return 27.5f * (float)read_buff / 4096.0f;
}

uint16_t LVSerial::readNowPos() {
	uint16_t read_buff;
	if (!readRAM(LVSerial::RegName::M_POS, reinterpret_cast<uint8_t*>(&read_buff), sizeof(read_buff)))
	{
		return 0;
	}
	return read_buff;
}


bool LVSerial::transmitReceiveToRAM(const RegName reg, const uint8_t* write_data, uint8_t* read_data, const bool is_write) {
	size_t data_size = getRegisterSpecification(reg).size;
	uint8_t data_address = getRegisterSpecification(reg).address;
		
	serial_.write(0x80 + servo_id_);
	serial_.write(is_write ? 0x40 + 0x20 + data_size : 0x20 + data_size);
	serial_.write(data_address);
	
	serial_.write(write_data, data_size);
	serial_.flush();
	
	return serial_.readBytes(read_data, data_size) == data_size;
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