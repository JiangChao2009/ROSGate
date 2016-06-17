#ifndef __ROS_GATE_H__
#define __ROS_GATE_H__

#include "server.h"
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Char.h>

#include <iostream>
#include <functional>
#include <map>
#include <set>



template<typename T>
void callback(const typename T::ConstPtr& msg){
	std::cout << msg->data << std::endl;
}

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

class Publisher : public ros::Publisher{

};

class ROSGate : public ServerSocket{
	private:
		ros::NodeHandle nh;
		std::list<ros::Publisher> publishers;
		std::list<ros::Subscriber> subscribers;
		default_map<std::string,std::set<ClientSocket*>> sub_map;

	public:
		ROSGate(int port);

		template<typename T>
			void add_subscriber(std::string topic){
				subscribers.push_back(nh.subscribe<T>(topic,5,
						[&](typename T::ConstPtr msg){ // callback
							for(auto client : sub_map[topic]){
								//TODO : check subscription status
								client->send((char*)&msg->data, sizeof(msg->data));
							}	
						}));
				// == if message comes, post to arduino
			}

		template<typename T>
			void add_publisher(std::string topic){
				//publish to ros when data comes from arduino 
				publishers.push_back(nh.advertise<T>(topic,1000));
			}

		void run_once();
		void run();
		bool ok();
};


template<typename T>
T interpret(std::string str_data){
	T data;
	//TODO : implement interpret
	return data;
}

#endif


