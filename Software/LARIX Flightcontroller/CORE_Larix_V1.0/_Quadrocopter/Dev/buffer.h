#ifndef __BUFFER_H__
#define __BUFFER_H__

#include <stdint.h>		// declare uint8_t
#include <stdbool.h>	// declare bool

#define SIZE 1 // 15

typedef struct{
	uint8_t num_items;
	uint8_t head;
	float buff[SIZE];
} Buffer;

void push_back(Buffer b, float item);
bool is_full(Buffer b);
float front(Buffer b);
float peek(Buffer b, uint8_t position);

#endif
