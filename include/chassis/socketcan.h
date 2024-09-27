#ifndef SOCKET_CAN
#define SOCKET_CAN

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class Socketcan
{
private:
    int s;
	struct sockaddr_can addr;
	struct ifreq ifr;
public:
    Socketcan(char *canX);
    //~Socketcan();

    void can_write(can_frame frame);
};




#endif