#include "common.h"
#include "udp_network.h"
#include "csocket.h"

int fd_udp_car_in_out = 0;

void init_udp()
{
	fd_udp_car_in_out  = UdpCreateSocket(get_random_port());
}

