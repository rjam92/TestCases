#include "FSM.h"
#include <iostream>


FSM_state* stateSAFEOP::faultFlagSet(MotorDriver& m) {
	std::cout << "SafeOp: Set" << std::endl;
	if (isEnter == true) {
		//write zero to encoder value
		request = 0x00000000;
		request = 0x1<<31|MotorDriverRegisters::ENCODER_VALUE << 24;
		checkXOR();
		response = m.transferData(request);
		//set isEnter to false
		isEnter = false;
	}

	//clear fault flag
	request = 0x00000000;
	request = 0x1<<31| MotorDriverRegisters::FAULT << 24;
	checkXOR();
	response = m.transferData(request);
	//read encoder
	request = 0x00000000;
	request = MotorDriverRegisters::ENCODER_VALUE << 24;
	checkXOR();
	response = m.transferData(request);
	encoderValue = (response >> 8) & 0xFFFF;
	//return this
	return this;
}

FSM_state* stateSAFEOP::faultFlagClear(MotorDriver& m)
{
	std::cout << "SafeOp: Clear" << std::endl;
	//request new state to OP
	request = 0x00000000;
	request = 0x1 << 31 | MotorDriverRegisters::CONTROLWORD << 24 | MotorState::STATE_OP << 8;
	checkXOR();
	response = m.transferData(request);
	//delete this
	delete this;
	//set new state OP for system
	return new stateOP;
}

FSM_state* stateOP::faultFlagSet(MotorDriver& m) {
	std::cout << "Op: Set" << std::endl;
	//disable output
	request = 0x00000000;
	request = 0x1 << 31 | MotorDriverRegisters::OUTPUT_ENABLE << 24;
	checkXOR();
	response = m.transferData(request);

	//request new state to SAFE OP
	request = 0x00000000;
	request = 0x1 << 31 | MotorDriverRegisters::CONTROLWORD << 24 | MotorState::STATE_SAFEOP << 8;
	checkXOR();
	response = m.transferData(request);
	//delete this
	delete this;
	//set new state to be safeOP for the system
	return new stateSAFEOP;
}

FSM_state* stateOP::faultFlagClear(MotorDriver& m) {
	std::cout << "Op: Clear" << std::endl;
	if (isEnter == true) {
		//enable output
		request = 0x00000000;
		request = 0x1 << 31 | MotorDriverRegisters::OUTPUT_ENABLE << 24|0x1<<8;
		checkXOR();
		response = m.transferData(request);
		//isEnter=false i.e. no state transition detected
		isEnter = false;
	
	} 	

	//write motor command: assuming a velocity of 5m/s. typically this valuse will correspond to the timer registers but simplifying her
	request = 0x00000000;
	request = 0x1<<31|MotorDriverRegisters::MOTOR_VELOCITY_COMMAND << 24|0x5<<8;
	checkXOR();
	response = m.transferData(request);
	//read encoder
	request = 0x00000000;
	request = MotorDriverRegisters::ENCODER_VALUE << 24;
	checkXOR();
	response = m.transferData(request);
	encoderValue = (response >> 8) & 0xFFFF;

	//return this
	return this;
}