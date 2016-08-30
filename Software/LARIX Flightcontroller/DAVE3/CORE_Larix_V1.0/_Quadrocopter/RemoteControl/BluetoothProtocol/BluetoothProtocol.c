/*
 * BluetoothProtocol.c
 *
 *  Created on: 30.01.2015
 *  modified on: 08.04.2016
 *      Author: SteurerE
 */
#include "BluetoothProtocol.h"

ControlValue control_value;
DataPacket dpacket;
uint8_t* ReadBufBT;

void initBluetoothStorage()
{
	ReadBufBT = malloc(sizeof(uint8_t)*BLUETOOTH_INPUT_BUFFER);
	control_value.header = malloc(sizeof(uint8_t));
	control_value.height_control = malloc(sizeof(uint8_t));
	control_value.speed = malloc(sizeof(uint8_t));
	control_value.x_pitch = malloc(sizeof(float));
	control_value.y_roll = malloc(sizeof(float));
	control_value.z_rotate = malloc(sizeof(float));
	control_value.checksum = malloc(sizeof(uint32_t));
}

status_t maintainBluetoothInputBuffer(
		uint8_t input_buffer[BLUETOOTH_INPUT_BUFFER],
		ControlValue *control_value, DataPacket* packet)
{
	static int packet_counter = 0;
	static int packets_to_be_received = 0;
	uint32_t checksum;
	checksum = input_buffer[0];
	checksum ^= ((input_buffer[1] << 8 | input_buffer[2]) & 0xFFFF);
	for (int i = 3; i < (PACKET_SIZE - PACKET_CHECKSUM) - 1; i += 4)
	{
		checksum ^= (input_buffer[i] << 24 | input_buffer[i + 1] << 16
				| input_buffer[i + 2] << 8 | input_buffer[i + 3]);
	}
	for(int j = 0;j < 4;j++)
	{
		*((uint8_t*)(control_value->checksum) + 3-j) = input_buffer[15+j];
	}
	control_value->header = (uint8_t*)&input_buffer[0];
	if (*(control_value->checksum) == checksum)
	{
		if (*control_value->header)//Got a valid Data-Package
			{
			if (packets_to_be_received == 0)
			{
				packets_to_be_received = input_buffer[0];
			}
			int current_val = packet_counter;
			for (; packet_counter < (current_val + DATA_SIZE);
				packet_counter++)
			{
				packet->cmd[packet_counter] = input_buffer[packet_counter
						- current_val + PACKET_HEADER];
			}
			if ((input_buffer[0]
					<= (PACKET_SIZE - PACKET_HEADER - PACKET_CHECKSUM)))
			{ //all cmd-bytes are within this data-package
				packet->character_count = packets_to_be_received;
				packet_counter = 0;
				packets_to_be_received = 0;
				//Cleanup Buffer//
				memset(input_buffer, 0x00,PACKET_SIZE);
				return RECEIVED_DATA_PACKET; //Got a complete and valid Data-Package
			} else {
				memset(input_buffer, 0x00,PACKET_SIZE);
				return RECEIVED_DATA_PACKET_N_C; //must do another read operation
			}
		} else
		{
			*((uint8_t*)(control_value->height_control)) = input_buffer[1];
			*((uint8_t*)(control_value->speed)) = input_buffer[2];
			for(int k = 0;k < 4;k++)
			{
				*((uint8_t*)(control_value->z_rotate) + 3-k) = input_buffer[3+k];
				*((uint8_t*)(control_value->x_pitch) + 3-k) = input_buffer[7+k];
				*((uint8_t*)(control_value->y_roll) + 3-k) = input_buffer[11+k];
			}
			return RECEIVED_CONTROL_PACKET; //Got a valid Control-Package
		}
	} else {
			packet_counter = 0; //if it happens between data-packages...
			//Cleanup Buffer//
			memset(input_buffer, 0x00,PACKET_SIZE);
			return CHECKSUM_ERROR; //Wrong checksum
	}
	return UNDEFINED_ERROR; //error by default but in fact never reachable
}

