#include "socketcan.h"

Socketcan::Socketcan(char *canX)
{
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s == -1) 
    	std::cerr << "Failed to create SocketCAN socket." << std::endl;
    
    std::strcpy(ifr.ifr_name, canX);  // CAN接口名称

	if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) 
	{
	    std::cerr << "Failed to get CAN interface index." << std::endl;
    }

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	// 绑定CAN地址
	if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == -1) 
	{
	    std::cerr << "Failed to bind SocketCAN socket." << std::endl;
    }
}

void Socketcan::can_write(can_frame frame)
{
    if (write(s, &frame, sizeof(struct can_frame)) == -1) 
	{
		std::cerr << "Failed to send CAN frame." << std::endl;
	}
}
