#include <thread>
#include <chrono>
#include <iostream>

#include "motordriver.h"
#include "FSM.h"



/**
 * @brief main This is a minimal main function.
 * @return
 */

/*
encrypting function
*/
void checkXOR(uint32_t &datas) {
    uint32_t checksum = (datas >> 24) ^ ((datas >> 16) & 0xFF) ^ ((datas >> 8) & 0xFF);
    datas = (datas & 0xFFFFFF00)|checksum;//substitute last byte with checksum
}


int main(void)
{
    std::cout << "Starting simulated motor driver" << std::endl;

    uint8_t sCase = MotorState::STATE_BOOT;
    uint32_t request,response;
    MotorDriver motorDriver;
  
    //polling to the motor driver till status word is preop
    while (sCase != MotorState::STATE_PREOP) {
        request = MotorDriverRegisters::STATUSWORD << 24 | MotorDriverRegisters::STATUSWORD;
        checkXOR(request);
        response = motorDriver.transferData(request);
        sCase = (response >> 8) & 0xFFFF;
        // Sleep for 100ms to simulate cyclic control
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Call update to simulate motor driver doing work
        motorDriver.update();
    }



    // Finite Stat Machine Variables
    uint16_t faultValue = 0x0001; //verify if fault flag is set;
    uint16_t registerValue = 0x0000;
    FSM_state* currentState;

    while(1)
    {

        switch (sCase) {
            case MotorState::STATE_PREOP: //since the only possibility is to move to safeop immediately
                request = 0x00000000;
                request = 0x1 << 31 | MotorDriverRegisters::CONTROLWORD << 24 | MotorState::STATE_SAFEOP << 8;
                checkXOR(request);
                response = motorDriver.transferData(request);
                //new instance of safeop
                sCase = MotorState::STATE_SAFEOP;
                currentState = new stateSAFEOP;
                break;
            default:
                //1. calls a function that checks for fault flag conditions and calls appropriate function based on the present state
                request = 0x00000000;
                request = MotorDriverRegisters::FAULT << 24;
                checkXOR(request);
                response = motorDriver.transferData(request);
                registerValue = (response >> 8) & 0xFFFF;
                try
                {
                    if (registerValue == faultValue) //faultflag is set
                    {
                        currentState = currentState->faultFlagSet(motorDriver);
                        //ensuring we are getting the right response value after reading the encoder
                        std::cout << "response from main flag set : " << *(currentState->getEncoderValue()) << std::endl;
                    }
                    else
                    {
                        currentState = currentState->faultFlagClear(motorDriver);
                        std::cout << "response from main flag clear : " << *(currentState->getEncoderValue()) << std::endl;
                    }

                    break;
                }
                catch (const std::exception& e)
                {
                    std::cout << e.what() << std::endl;
                }
  
         }

        // Call update to simulate motor driver doing work
        motorDriver.update();

        // Sleep for 100ms to simulate cyclic control
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
