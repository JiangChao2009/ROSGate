#ifndef __MSG_H__
#define __MSG_H__
#include "dtype.h"

struct msg_t{
	dtype t;
	char topic[3]; //arbitrary size
	char method[4]; //sub\0 or pub\0
	uint8_t buf[8];
};

#endif
