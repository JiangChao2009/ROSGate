#ifndef __DEFAULT_MAP_H__
#define __DEFAULT_MAP_H__
#include <map>
template<typename K, typename V>
struct default_map : std::map<K,V>{
	V default_value;

	default_map(){
		//initialized as true default
	}

	default_map(V& default_value) : default_value(default_value){

	}

	V& operator[](K& key){
		auto it = this->find(key);
		if (it == this->end()){
			auto entry = std::make_pair(key,default_value);
			auto res = this->insert(entry);
			return res.first->second;
		}else{
			return it->second;
		}
	}
};
#endif
