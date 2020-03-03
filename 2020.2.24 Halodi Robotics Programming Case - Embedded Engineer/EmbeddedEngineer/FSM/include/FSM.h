#ifndef FSM_H
#define FSM_H
#include <cstdint>
#include "../../motordriver/include/motordriver.h"

class FSM_state {
protected:
	uint32_t request;
	uint32_t response;
	uint16_t encoderValue;

public:
	virtual ~FSM_state() {}
	//virtual FSM_state* immediate(MotorDriver &m) { return this; } //using a switch to facilitate a one time event
	virtual FSM_state* faultFlagSet(MotorDriver &m) { return this; }
	virtual FSM_state* faultFlagClear(MotorDriver &m) { return this; }
	void checkXOR() {
		uint32_t checksum = (request >> 24) ^ ((request >> 16) & 0xFF) ^ ((request >> 8) & 0xFF);
		request = (request & 0xFFFFFF00) | checksum;//substitute last byte with checksum
	}
	uint32_t* getResponse() {
		return &response;
	}
	uint16_t* getEncoderValue() {
		return &encoderValue;
	}
};

/*class statePREOP :public FSM_state {

public:
	virtual ~statePREOP() {}
	FSM_state* immediate(MotorDriver &m);
};*/

class stateSAFEOP : public FSM_state {
protected:
	bool isEnter = true;

public:
	virtual ~stateSAFEOP(){}
	FSM_state* faultFlagClear(MotorDriver &m);
	FSM_state* faultFlagSet(MotorDriver &m);
};

class stateOP : public FSM_state {
protected:
	bool isEnter = true;
public:
	virtual ~stateOP() {}
	FSM_state* faultFlagSet(MotorDriver &m);
	FSM_state* faultFlagClear(MotorDriver& m);

};

#endif

