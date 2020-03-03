#include <cstdint>
#include <stdexcept>
#include <iostream>
#include "motordriver.h"



uint16_t Registers[REGISTERS_MAX_VALUE];
uint8_t verifyChecksum(uint8_t& a, uint8_t& b, uint16_t& c) {
    uint8_t sum=((a << 7) | b) ^ ((c >> 8) & 0xFF) ^ (c & 0xFF);
    return sum;
}


MotorDriver::MotorDriver()
{
    // Clear all registers
    for(uint8_t i = 0; i < REGISTERS_MAX_VALUE; ++i)
    {
        Registers[i] = 0;
    }

    // Set the fault flag on boot
    Registers[FAULT] = 1;
}

uint32_t MotorDriver::transferData(uint32_t data)
{
    //unable to process communication until motorstatus is in preop
    if (Registers[STATUSWORD] == MotorState::STATE_BOOT) { return STATUS_ERROR; }
    
    uint8_t writing = (data >> 31);
    uint8_t command = (data >> 24) & 0x7F;
    uint16_t value = (data >> 8) & 0xFFFF;
    uint8_t checksum = data & 0xFF;

    // TODO: Implement checksum verification
    uint8_t cSum= verifyChecksum(writing, command, value);
    
    //valid command check
    if (command > 0x7 || command == 0x0 || command == MotorDriverRegisters::STATUSWORD && writing != 0x0 || command == MotorDriverRegisters::RESET && writing != 0x1)
    {
        return STATUS_ERROR << 8;
    }
    //cheksum check
    else if (cSum != checksum) {
         //checksum sets the fault flag
         Registers[FAULT] = 1;
         //return error code of fault detection
         return STATUS_ERROR<<8;

    }
   
    //read/write check
    else if(writing) //only if checksum matches will it attempt to write or read
    {
        //  also hold off writing and verify all illegal states and issue a warning, if valid state update status register if a state is invalid terminate the test case
        if (Registers[STATUSWORD] == MotorState::STATE_BOOT && command==MotorDriverRegisters::CONTROLWORD && value == MotorState::STATE_SAFEOP || Registers[STATUSWORD] == MotorState::STATE_BOOT && command == MotorDriverRegisters::CONTROLWORD && value == MotorState::STATE_OP || Registers[STATUSWORD] == MotorState::STATE_PREOP && command == MotorDriverRegisters::CONTROLWORD && value == MotorState::STATE_BOOT || Registers[STATUSWORD] == MotorState::STATE_PREOP && command == MotorDriverRegisters::CONTROLWORD &&  value == MotorState::STATE_OP || Registers[STATUSWORD] == MotorState::STATE_SAFEOP && command == MotorDriverRegisters::CONTROLWORD &&  value == MotorState::STATE_BOOT || Registers[STATUSWORD] == MotorState::STATE_OP && command == MotorDriverRegisters::CONTROLWORD &&  value == MotorState::STATE_BOOT)
        {
            Registers[FAULT] = 1;
            throw ("invalid state transition/implementation of state machine");
        }
        // also if the fault flag is set then safeop to op should not be possible and if flag is clear then op to safeop should not be possible invalid state machine implementation
        if (Registers[STATUSWORD] == MotorState::STATE_SAFEOP && Registers[FAULT] == 1 && command == MotorDriverRegisters::CONTROLWORD && value == MotorState::STATE_OP || Registers[STATUSWORD] == MotorState::STATE_OP && Registers[FAULT] == 0 && command == MotorDriverRegisters::CONTROLWORD && value == MotorState::STATE_SAFEOP) {
           throw ("invalid state transition/implementation of state machine");
        }
        // if no invalid command or state then proceed with writing
        Registers[command] = value;
        //if the requested state was valid then update the status of the statusa register as well
        if (command == MotorDriverRegisters::CONTROLWORD) {
            Registers[STATUSWORD] = value;
        }
        return 0;
    }
    else
    {
        //allow for reading in any state 
        // TODO: Implement checksum for response
        return (Registers[command] << 8)|((Registers[command]&0xFF)^(((Registers[command])>>8)&0xFF)^0x00);
    }


}

void MotorDriver::update()
{
    if(Registers[STATUSWORD] == MotorState::STATE_BOOT)
    {
        Registers[STATUSWORD] = MotorState::STATE_PREOP;
        Registers[CONTROLWORD] = MotorState::STATE_PREOP;
    }

    Registers[ENCODER_VALUE] += Registers[MOTOR_VELOCITY_COMMAND];

}
