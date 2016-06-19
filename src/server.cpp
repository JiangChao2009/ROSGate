#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/select.h>

#include <list>
#include <iostream>

#include "server.h"


// SOCKEt
Socket::Socket(int sock):sock(sock){
	memset(&addr,0,sizeof(addr));
}
Socket::Socket(int d, int t, int p)
	: Socket(socket(d,t,p)){

	}
Socket::Socket(Socket&& o){
	sock = o.sock;
	addr = o.addr;
	o.sock = -1; //invalidate
}
Socket::~Socket(){
	printf("CLOSING SOCKET %d\n", sock);
	close(sock);
}

bool Socket::operator<(const Socket& o){
	return sock < o.sock;
}

bool Socket::operator>(const Socket& o){
	return sock > o.sock;
}

bool Socket::operator==(const Socket& o){
	return sock == o.sock;
}
// CLIENT SOCKET
ClientSocket::ClientSocket(int serverSocket, fd_set* master_fd)
	:Socket(0),master_fd(master_fd){
		socklen_t len = sizeof(addr);
		sock = accept(serverSocket,(sockaddr*)&addr,&len);
		FD_SET(sock, master_fd);
	}
ClientSocket::ClientSocket(ClientSocket&& o): Socket(std::move(o)){
	master_fd = o.master_fd;
	o.master_fd = nullptr; //invalidate
}

ClientSocket::~ClientSocket(){
	//do i have to close it first? check...
	FD_CLR(sock, master_fd);
}

int ClientSocket::send(char* buf, int len){
	::send(sock,buf,len,0);
	return 0; 
}

int ClientSocket::recv(char* buf){
	return ::recv(sock,buf,BUF_SIZE,0);
}


// SERVER SOCKET
ServerSocket::ServerSocket(int port):Socket(AF_INET,SOCK_STREAM,0){
	int opt = 1;
	setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);
	bind(sock, (sockaddr*)&addr,sizeof(addr));
	listen(sock,5);

	FD_SET(sock, &master_fd);
	fdmax = sock;
}

void ServerSocket::run_once(){
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
			std::cout << "!!" << std::endl;
			auto& client = *it;
			if(FD_ISSET(client.sock, &copy_fd)){
				int n = client.recv(buf);
				if(n <= 0){ //data
					it = clients.erase(it);
					continue;
				}else{
					std::cout << buf << std::endl;
				}
			}
			++it;
		}
} 

void ServerSocket::run(){

	while(1){
		run_once();
	}


}

/* EXAMPLE
int main(int argc, char* argv[]){
	if(argc != 2){
		printf("USAGE : %s <PORT>\n", argv[0]);
		exit(1);
	}
	int PORT = atoi(argv[1]);

	ServerSocket serverSocket(PORT);

	serverSocket.run();

	return 0;
}
*/
