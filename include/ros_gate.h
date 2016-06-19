#ifndef __ROS_GATE_H__
#define __ROS_GATE_H__

#include <ros/ros.h>
#include <ros/serialization.h>

#include <iostream>
#include <functional>
#include <set>

#include "default_map.h"
#include "server.h"
#include "dtype.h"
#include "msg_t.h"


namespace ser = ros::serialization;
using dmap = default_map<std::string, std::set<ClientSocket*>>;

template<typename T>
struct Router{

	std::string topic;
	dmap& sub_map;

	Router(std::string topic, dmap& sub_map)
		:topic(topic),sub_map(sub_map){
	}

	void operator()(const typename T::ConstPtr& msg){
		std::cout << "MSG : " << msg->data << std::endl;
		std::cout << "#SUB : " << sub_map[topic].size() << std::endl;
		auto& clients = sub_map[topic];

		for(auto& client : clients){
			std::cout << "SOCKET : " <<  client->sock << std::endl;
			int n = ser::serializationLength(*msg);
			std::cout << "LEN : " << n << std::endl;

			msg_t buf;

			buf.t = get_dtype<T>();

			memcpy(buf.topic, topic.c_str(),3); //arbitrary

			memcpy(buf.method, "pub", 4);

			ser::OStream stream(buf.buf,n);

			ser::Serializer<T>::write(stream,*msg);
			client->send((char*)&buf,sizeof(msg_t));
		}
	}

};

class ROSGate : public ServerSocket{
	private:
		ros::NodeHandle nh;
		std::list<ros::Publisher> publishers;
		std::list<ros::Subscriber> subscribers;

		std::list<std::string> p_topics;
		std::list<std::string> s_topics;

		dmap sub_map;

	public:
		ROSGate(int port);

		template<typename T>
			void add_subscriber(std::string topic){
				std::cout << "ADDING SUBSCRIBER OF : " << topic << std::endl;
				subscribers.emplace_back(nh.subscribe<T>(topic,5,Router<T>(topic,sub_map)));
			}

		void add_publisher(dtype t, std::string topic);
		void add_subscriber(dtype t, std::string topic);
		//publish to ros when data comes from arduino 

		void run_once();
		void run();
		bool ok();
};

#endif
