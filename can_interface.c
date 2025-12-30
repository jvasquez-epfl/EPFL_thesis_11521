#include "can_interface.h"
#include "stdint.h"

void ManageCAN_rx(uint8_t *canRX, struct can_interface canbus)
{
	uint32_t msg_data_32 = BytesToInt(canRX);
	uint8_t msg_data0_8 = canRX[2];
	uint8_t msg_data1_8 = canRX[3];
	if(canRX[MSG_TYPE] < INSTRUCTION_TYPES && canRX[MSG_CONTENT] < INSTRUCTION_COUNT){	//check whether valid message.
		if(canbus.interface_functions[canRX[MSG_TYPE]][canRX[MSG_CONTENT]] != 0){ // check whether function defined.
			(canbus.interface_functions[canRX[MSG_TYPE]][canRX[MSG_CONTENT]])(msg_data0_8, msg_data1_8, msg_data_32);
		}
	}
}

void CAN_writedata(int adresse, int data , struct can_interface canbus)
{
	uint8_t buffer[8];
	intToBytes(buffer, data);
	(canbus.transmit_function)(adresse, buffer);
}

void intToBytes(uint8_t *buffer, int value){
	buffer[4] = value >> 24;
	buffer[5] = value >> 16;
	buffer[6] = value >> 8;
	buffer[7] = value;
}

uint32_t BytesToInt(uint8_t *buffer){
	uint32_t value;
	value = (buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7];
	return value;
}
