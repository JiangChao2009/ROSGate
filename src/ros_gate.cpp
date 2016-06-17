#include "ros_gate.h"

#include <sstream>


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

	for(auto it = clients.begin(); it != clients.end();){
		auto& client = *it;
		if(FD_ISSET(client.sock, &copy_fd)){
			int n = client.recv(buf);
			if(n <= 0){ //data
				it = clients.erase(it);
				// TODO : also remove from subscription status
				continue;
			}else{
				std::cout << buf << std::endl;

				std::string msg(buf,n);
				// msg = [sub|pub]|[topic]|[content]
				auto l = split(msg,'|');
				auto& method = l[0];
				auto& topic = l[1];
				auto& content = l[2];

				if(method == "sub"){
					//subscription request -- to have topic forwarded to them
					sub_map[topic].insert(&client);
					add_subscriber<std_msgs::Int16>(topic);
				}else if (method == "pub"){
					//publishing to topic
					for(auto& publisher : publishers){
						if(publisher.getTopic() == topic){
							std_msgs::Int16 ros_msg; //unfortunately only support Int16 right now
							ros_msg.data = (std::atoi(content.c_str()));
							publisher.publish(ros_msg);
							break;
						}
					}
				}
				std::cout << buf << std::endl;
			}
		}
		++it;
	}
	ros::spinOnce();
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
