/*
 * tcp_server.c
 *
 *  Created on: 2016年8月16日
 *      Author: root
 *      初始化本机8888端口作为tcp服务端，不断接收tcp客户端的访问，大小限制为1k字节。
 *      凡是来消息的客户端，在客户访问列表里面保持一定时间，并且保存来访相机索引号，
 *      用以确定该相机发送udp数据至客户ip
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
//#include <linux/in.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <fcntl.h>
#include <sys/ioctl.h>
//#include <linux/time.h>
#include "common.h"
#include "client_net.h"
#include <errno.h>
#include <sys/select.h>
//#define BUFFER_MAX 1000
//#define TCP_TIMEOUT 100
int init_skt()
{
		int port=8888;
		int maxqueue=10;
		int fd;
		int value;
		struct timeval timeo;
		socklen_t len = sizeof(timeo);
	//	char ip[]="192.168.1.113";
		memset(&timeo, 0, sizeof(timeo));
		if ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			prt(err, "socket err");
			return -1;
		}

	    value = 1;
	    if(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &value, sizeof(value))<0){
	    //	Log0("LINE: %d FUN: %s\n", __LINE__, __FUNCTION__);
	        close(fd);
	        return -1;
	    }

		struct sockaddr_in sa;
		memset(&sa, 0, sizeof(struct sockaddr_in));
		sa.sin_family = AF_INET;
		sa.sin_addr.s_addr = htonl(INADDR_ANY);
		sa.sin_port = htons(port);
		if (bind(fd, (struct sockaddr *) (&sa), sizeof(struct sockaddr)) < 0) {
			prt(err, "socket err,bind err,fd %d",fd);
			close(fd);
			return -1;
		}

		if (listen(fd, maxqueue) < 0) {
			prt(err, "socket err");
			close(fd);
			return -1;
		}
		return fd;
}

int WaitTcpConnect(int sock, unsigned long sec, char * ip, unsigned short * port)
{
    int cl_fd=0, one;
    fd_set rd_set;
    struct timeval timeout;
    struct sockaddr_in  client_addr;
    int client_len = sizeof(struct sockaddr_in);

    timeout.tv_sec = sec;
    timeout.tv_usec = 0;

    FD_ZERO(&rd_set);
    FD_SET(sock, &rd_set);
	while ((cl_fd = select(sock + 1, &rd_set, NULL, NULL, &timeout)) < 0) {

		if (EINTR == errno) {
			prt(err, "socket err EINTR");

		}
		if (EINVAL == errno) {
			prt(err, "socket err EINVAL");

		}
		if (ENOMEM == errno) {
			prt(err, "socket err ENOMEM");

		}
		if (EBADF == errno) {
			prt(err, "socket err EBADF");

		}
		prt(err, "socket err,need rebind socket");
		cl_fd = -1;
		return cl_fd;
	}
//  	prt(net,"select rst %d",cl_fd);
//	if(0== cl_fd){
//	   	prt(err,"select time out,need select again  fd %d",cl_fd);
	//	return 0;
//	}

    if(FD_ISSET(sock, &rd_set))
    {
  //  	prt(net,"get msg from client");

		if((cl_fd = accept(sock, (struct sockaddr *)&client_addr, (socklen_t *)&client_len))<=0){

		   	prt(err,"accept  err  %d",cl_fd);
			//close_socket(&sock);

		    cl_fd=-1;
		}else{
//		  	prt(net,"accept rst %d",cl_fd);
		}



		if(ip != NULL)
            strcpy(ip, inet_ntoa(client_addr.sin_addr));
		if(port != NULL)
            *port = ntohs(client_addr.sin_port);
       // return cl_fd;
	}else
	{
		prt(net,"no client in 10s ",cl_fd);
	}
    return cl_fd;
 //   return 1;
}
void handle_buf(char *bf)
{
	mCommand *p=(mCommand *)bf;
	prt(info,"%d ",p->version);
	prt(info,"number %d ",ntohs(p->objnumber));
	prt(info,"type %d ",ntohs(p->objtype));
	prt(info,"len %d ",ntohl(p->objlen));
	prt(info,"%d ",p->prottype);


//	mDetectDeviceConfig test;

}
int recv_buf(int fd)
{
	unsigned short port;
	char ip[16];
	unsigned char buf[BUFFER_MAX];
	memset(buf,0,BUFFER_MAX);
	int ret;
	int client;
	prt(server,"waiting for clients ");
	client=WaitTcpConnect(fd,TCP_TIMEOUT,ip,&port);
	if(client>0){

	//	prt(info,"get client , ip %s",ip);
		ret=recv(client,buf,BUFFER_MAX,0);
	//	handle_buf(buf);
	//	prt(info,"fd %d get %d bytes",client,ret );
	//	mDetectDeviceConfig test;
	//	int len =sizeof(mDetectDeviceConfig);
	//	memset(&test,0,len);

		int send_len=handle_buffer(buf,ret,ip);
		if(send_len>0)
		{
			send(client,&buf,send_len,0);
		}
		prt(server,"    clients  done ");
		close(client);
		return 0;
	}else{
		return -1;
	}

}
int recv_param()
{
	return 0;
}
int handle_cmd()
{
	return 0;
}
int send_rst()
{
	return 0;
}
void  init_tcp_server()
{
	int fd;
	fd=init_skt();
//	prt(info,"init server fd %d",fd);
//	if(fd>0){
//		printf("craeted fd");
//	}
	//int client_fd;
	while(1){
		recv_buf(fd);
	}
}
void test_client()
{
	int fd;
    if((fd =socket(AF_INET, SOCK_STREAM, 0))<0)
    {

    }
    int value = 1;
//    if(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &value , sizeof(value))<0){
//        close(fd);
//        return -1;
//    }

   // struct sockaddr addr;
  //  addr.sa_data
	struct sockaddr_in addr;
	int port=5678;
	memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family= AF_INET;
	addr.sin_addr.s_addr = inet_addr("192.168.1.113");
//	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);
	if (connect(fd, (struct sockaddr *) &addr, sizeof(struct sockaddr)) == -1) {
		printf("connect fail\n");
		//return -1;
	}else{
		printf("connect ok\n");
 	}
	unsigned long ul = 1;
	ioctl(fd, FIONBIO, &ul); //设置为非阻塞模式
	char buf[10];
	memset(buf,0,10);
	printf("prepare to reve\n");
	int ret=0;
	ret=recv(fd,buf,10,0);
	ret=printf("rece ok, ret %d,get %d %d %d\n",ret,buf[0],buf[1],buf[2]);
	ret=recv(fd,buf,10,0);
	printf("rece ok,ret %d,get %d %d %d\n",ret,buf[0],buf[1],buf[2]);
	ret=recv(fd,buf,10,0);
	printf("rece ok,ret %d,get %d %d %d\n",ret,buf[0],buf[1],buf[2]);
	while(1) ;

}
//int main(int argc, char **argv)
//{
//	//test_client();
//	init_protocol();
//    init_tcp_server();
//	return 0;
//}
