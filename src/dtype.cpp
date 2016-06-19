#include "dtype.h"

template<>
dtype get_dtype<std_msgs::Int16>(){
	return dtype::Int16;
}

template<>
dtype get_dtype<std_msgs::Float32>(){
	return dtype::Float32;
}

template<>
dtype get_dtype<std_msgs::String>(){
	return dtype::String;
}

template<>
dtype get_dtype<std_msgs::Bool>(){
	return dtype::Bool;
}

template<>
dtype get_dtype<std_msgs::Byte>(){
	return dtype::Byte;
}
template<>
dtype get_dtype<std_msgs::Char>(){
	return dtype::Char;
}

std::string toString(dtype t){
	switch(t){
		case dtype::Int16:
			return "Int16";	
		case dtype::Float32:
			return "Float32";	
		case dtype::String:
			return "String";	
		case dtype::Bool:
			return "Bool";	
		case dtype::Byte:
			return "Byte";	
		case dtype::Char:
			return "Char";	
		default:
			return "Invalid";
	}
}
