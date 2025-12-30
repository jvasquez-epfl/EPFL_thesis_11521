#include "stdint.h"

#define INSTRUCTION_TYPES 2
#define INSTRUCTION_COUNT 50

#define MSG_TYPE 0
#define MSG_WRITE 0x00
#define MSG_REQ 0x01

#define MSG_CONTENT 1	

// Laser controllers
#define SET_EXPOSURE 0x05
#define SET_INTENSITY 0x06
#define SET_DELAY 0x07
#define SET_STARTIMAGING 0x08
#define SET_PIEZO 0x09
#define SET_SOLOPIEZO 0x10
#define SET_SOLOIMAGING 0x11

struct can_interface{
	void (*interface_functions[INSTRUCTION_TYPES][INSTRUCTION_COUNT])(uint8_t, uint8_t, uint32_t);
	void (*transmit_function)(int, uint8_t*);
};

void ManageCAN_rx(uint8_t *canRX, struct can_interface canbus);
void CAN_writedata(int adresse, int data , struct can_interface canbus);
void intToBytes(uint8_t*, int);
uint32_t BytesToInt(uint8_t*);
