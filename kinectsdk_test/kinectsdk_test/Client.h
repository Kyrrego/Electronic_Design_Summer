#pragma once

#define WIN32_LEAN_AND_MEAN

#include <stdio.h>  
#include <Winsock2.h> 
#include <iostream>
#pragma comment(lib,"ws2_32.lib")
#pragma warning(disable:4996)


#define MAXLINE 4096

class Send {
private:
	SOCKET sockClient;
public:
	char buff[MAXLINE];
	void Initialize();
	void Send_msg(const char* msg);
	void Shutdown();
	bool Recv_msg();
};