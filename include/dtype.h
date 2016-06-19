#ifndef __DTYPE_H__
#define __DTYPE_H__

#include <string>

#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>


enum dtype : uint8_t{
	Int16,
	Float32,
	String,
	Bool,
	Byte,
	Char
};

template<typename T>
dtype get_dtype();

extern std::string toString(dtype t);

#endif
