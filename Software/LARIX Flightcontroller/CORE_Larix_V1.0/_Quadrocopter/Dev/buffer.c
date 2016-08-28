#include "buffer.h"

void push_back(Buffer b, float item)
{
	uint8_t tail = b.head + b.num_items;
	if(tail >= SIZE) {
		tail -= SIZE;
	}

	// add item to buffer
	b.buff[tail] = item;

	// increment number of items
	if(b.num_items < SIZE) {
		b.num_items++;
	} else{
		// no room for new items so drop oldest item
		b.head++;
		if(b.head >= SIZE) {
			b.head = 0;
		}
	}
}

bool is_full(Buffer b)
{
	return b.num_items >= SIZE;
}

float front(Buffer b)
{
	return peek(b,0);
}

float peek(Buffer b, uint8_t position)
{
	uint8_t j = b.head + position;

	// return zero if position is out of range
	if(position >= b.num_items) {
		const static float r = 0;
		return r;
	}

	// wrap around if necessary
	if(j >= SIZE)
		j -= SIZE;

	// return desired value
	return b.buff[j];
}





