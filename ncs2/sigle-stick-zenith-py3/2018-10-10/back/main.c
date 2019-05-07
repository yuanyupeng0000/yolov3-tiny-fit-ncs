#include <unistd.h>
#include "camera_service.h"
#include "tcp_server.h"
#include "client_net.h"
#include "common.h"
#include "sig_service.h"
#include "udp_network.h"


#define CAMNUMSTR cameraLOG%d
/* Function Description
 * name:main
 * return:
 * args:
 * comment:entrance
 * todo:
 */
int main(int argc, char **argv)
{

//	void *data;
//	prt(info,"addr %p",data);
//	data=malloc(1000*1000*100);
//	usleep(50*1000000);
//	char *p=(char *)data;
//	p[0]=1;
//	prt(info,"addr %p",data);
//	free(data);
//	prt(info,"addr %p",data);
//	while(1)
//		;
//	prt(camera_log_info,"\CAMNUMSTR",5);
    setup_sig();
    camera_service_init();
	init_config();
	init_udp();
    //init_sig_service();
    init_sig_service2();
//init_sig_service1();
    init_tcp_server();
	
	prt(info,"main function end");
    while(1)
        usleep(10);
	return 0;
}
