#ifndef __TCP_SERVER_H__
#define __TCP_SERVER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/select.h>

#include <list>
#include <iostream>


#define BUF_SIZE 512


struct Socket{
	int sock;
	sockaddr_in addr;

	Socket(int sock);
	Socket(int d, int t, int p);
	Socket(Socket&& o);
	~Socket();

	// for map-related function
	bool operator<(const Socket&);
	bool operator>(const Socket&);
	bool operator==(const Socket&);
};

struct ClientSocket : public Socket{
	fd_set* master_fd;
	ClientSocket(int serverSocket, fd_set* master_fd);
	ClientSocket(ClientSocket&& o);
	~ClientSocket();
	int send(char* buf, int len);
	int recv(char* buf);
};

struct ServerSocket : public Socket{
	fd_set master_fd, copy_fd;
	int fdmax;
	char buf[BUF_SIZE];

	std::list<ClientSocket> clients;
	ServerSocket(int port);
	void run_once();
	void run();
};

#endif
