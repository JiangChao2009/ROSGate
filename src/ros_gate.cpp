#include "ros_gate.h"

#include <string>
#include <vector>
#include <iostream>
#include <sstream>


template<typename T>
T deserialize(uint8_t* buf){
	T msg;
	int n = ser::serializationLength(msg);
	ser::IStream stream(buf,n);
	ser::Serializer<T>::read(stream,msg);
	return msg;
}

void publish(ros::Publisher& p, msg_t& buf){
	switch(buf.t){//differentiate based on dtype
		case dtype::Int16:
			{
				auto msg = deserialize<std_msgs::Int16>(buf.buf);
				p.publish(msg);
			}
			break;
		case dtype::Float32:
			{
				auto msg = deserialize<std_msgs::Float32>(buf.buf);
				p.publish(msg);
			}
		case dtype::String:
			{
				auto msg = deserialize<std_msgs::String>(buf.buf);
				p.publish(msg);
			}
		case dtype::Bool:
			{
				auto msg = deserialize<std_msgs::Bool>(buf.buf);
				p.publish(msg);
			}
		case dtype::Byte:
			{
				auto msg = deserialize<std_msgs::Byte>(buf.buf);
				p.publish(msg);
			}
		case dtype::Char:
			{
				auto msg = deserialize<std_msgs::Char>(buf.buf);
				p.publish(msg);
			}
	}
}

std::vector<std::string> split(std::string s, char delim){
	std::stringstream ss(s);
	std::string item;
	std::vector<std::string> l;
	while(std::getline(ss,item,delim)){
		l.push_back(item);
	}
	return l;
}

ROSGate::ROSGate(int port):ServerSocket(port){

}

void ROSGate::run_once(){
	copy_fd = master_fd;
	if(select(fdmax+1,&copy_fd,NULL,NULL,NULL) == -1){
		//complain...
	}

	if(FD_ISSET(sock, &copy_fd)){
		//accept client
		clients.emplace_back(sock, &master_fd);
		fdmax = fdmax > clients.back().sock? fdmax : clients.back().sock;
	}

	msg_t buf;
	// buf[dtype][topic][(sub|pub),content]

	for(auto it = clients.begin(); it != clients.end();){
		auto& client = *it;
		if(FD_ISSET(client.sock, &copy_fd)){
			int n = client.recv((char*)&buf);
			if(n <= 0){ // closed connection

				//remove from subscription list
				for(auto& e : sub_map){
					//auto& topic = e.first;
					auto& sub_cls = e.second; //list of clients to topic

					//iterate over subscriber sockets
					for(auto sc_it = sub_cls.begin(); sc_it != sub_cls.end(); ++sc_it){
						if(*sc_it == &client){ //comparison of pointers : has to equal.
							//alternatively, can compare socket values
							sub_cls.erase(sc_it);
							break;
						}
					}
				}

				it = clients.erase(it);
				continue;
			}else{
				std::string topic(buf.topic);
				std::cout << "TOPIC : " << topic << std::endl;

				if(strncmp((char*)buf.method,"sub",4) == 0){
					printf("Subscription request : [%s]\n", topic.c_str());
					sub_map[topic].insert(&client); //remember the socket, but by pointer

					//create if there is no router for the topic
					if(std::find(s_topics.begin(), s_topics.end(), topic) == s_topics.end()){
						printf("ADDING NEW SUBSCRIBER\n");
						s_topics.push_back(topic);
						add_subscriber(buf.t, topic);
					}

				}else if (strncmp((char*)buf.method,"pub",4) == 0){
					printf("Publication request : [%s]\n", topic.c_str());

					//create publisher for topic if there's not already one
					if(std::find(p_topics.begin(), p_topics.end(), topic) == p_topics.end()){
						printf("ADDING NEW PUBLISHER\n");
						p_topics.push_back(topic);
						add_publisher(buf.t, topic);
					}

					//then publish the message to ros board
					for(auto& publisher : publishers){
						std::cout << "PTOPIC : " << publisher.getTopic() << std::endl;
						if(publisher.getTopic() == topic || publisher.getTopic() == "/" + topic){
							std::cout << "PUBLISHING..." << std::endl;
							publish(publisher, buf);
							break;
						}
					}
				}
			}
		}
		++it;
	}

	ros::spinOnce();
}

void ROSGate::add_subscriber(dtype t, std::string topic){
	std::cout << "DTYPE : " << toString(t) << std::endl;

	switch(t){
		case dtype::Int16:
			add_subscriber<std_msgs::Int16>(topic);
			break;
		case dtype::Float32:
			add_subscriber<std_msgs::Float32>(topic);
			break;
		case dtype::String:
			add_subscriber<std_msgs::String>(topic);
			break;
		case dtype::Bool:
			add_subscriber<std_msgs::Bool>(topic);
			break;
		case dtype::Byte:
			add_subscriber<std_msgs::Byte>(topic);
			break;
		case dtype::Char:
			add_subscriber<std_msgs::Char>(topic);
			break;
	}
}

void ROSGate::add_publisher(dtype t, std::string topic){
	std::cout << "DTYPE : " << toString(t) << std::endl;

	switch(t){
		case dtype::Int16:
			publishers.push_back(nh.advertise<std_msgs::Int16>(topic,1000));
			break;
		case dtype::Float32:
			publishers.push_back(nh.advertise<std_msgs::Float32>(topic,1000));
			break;
		case dtype::String:
			publishers.push_back(nh.advertise<std_msgs::String>(topic,1000));
			break;
		case dtype::Bool:
			publishers.push_back(nh.advertise<std_msgs::Bool>(topic,1000));
			break;
		case dtype::Byte:
			publishers.push_back(nh.advertise<std_msgs::Byte>(topic,1000));
			break;
		case dtype::Char:
			publishers.push_back(nh.advertise<std_msgs::Char>(topic,1000));
			break;
	}
}


void ROSGate::run(){
	while(ros::ok() && ok()){
		run_once();
	}
}


bool ROSGate::ok(){
	//TODO : implement
	return true;
}
