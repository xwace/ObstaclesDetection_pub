#ifndef CLIENT_SOCKET_H
#define CLIENT_SOCKET_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>

#ifdef __cplusplus
}
#endif
#include <string>
#include <thread>
#include <atomic>
// #define MYPORT  7000

class Client
{
	public:
		~Client()
		{
			close(sock_cli);
		}
		void init(std::string ip,uint16_t MYPORT)
		{
			///定义sockfd
			connect_ok = false;
			sock_cli = socket(AF_INET,SOCK_STREAM, 0);
			memset(&servaddr, 0, sizeof(servaddr));
			servaddr.sin_family = AF_INET;
			servaddr.sin_port = htons(MYPORT);  //服务器端口
			servaddr.sin_addr.s_addr = inet_addr(ip.c_str());
			conne_handle = std::thread(std::bind(&Client::connect_handle,this));
			conne_handle.detach();
		}
		

		bool sendMsg(uint8_t*data,int size)
		{
			if(connect_ok)
			{
				send(sock_cli,data,size,0);
				return true;
			}
			return false;
		}
	private:
		int sock_cli;
		struct sockaddr_in servaddr;
		std::thread conne_handle;
		std::atomic<bool> connect_ok;
		void connect_handle()
		{
			//连接服务器，成功返回0，错误返回-1
			while (connect(sock_cli, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
			{
				printf("connect failed!\n");
				sleep(1);
			}
			connect_ok = true;
		}
};

#endif