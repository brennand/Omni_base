#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>

#include "UdpSocket.h"

UdpSocket::UdpSocket(const char* remoteHost, const int remotePort, const int localPort, const int timeout) {
	if((udpSocket_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		std::cerr << "ERROR: [UdpSocket][UdpSocket] Failed to open socket!" <<
			strerror(errno) << "! Exiting!" << std::endl;
		exit(-1);
	}

	struct hostent *host = gethostbyname(remoteHost);
	if(host == NULL) {
	  	std::cerr << "ERROR: [UdpSocket][UdpSocket] Unknown host " <<
			remoteHost << "! Exiting!" << std::endl;
		exit(-1);
	}

	//set timeout for socket
	if(timeout != -1) {
		timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = timeout * 1000;

		if(setsockopt(udpSocket_, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv))) {
			std::cerr << "ERROR: [UdpSocket][UdpSocket] Failed to set SO_RCVTIMEO! " <<
				strerror(errno) << "! Exiting!" << std::endl;
			exit(-1);
		}
	}

	bzero((char *) &localAddr_, sizeof(localAddr_));
	localAddr_.sin_family = AF_INET;
	localAddr_.sin_addr.s_addr = INADDR_ANY;
	localAddr_.sin_port = htons(localPort);
	localAddrLen_ = sizeof(localAddr_);

	bzero((char *) &remoteAddr_, sizeof(remoteAddr_));
	remoteAddr_.sin_family = AF_INET;
	bcopy(host->h_addr_list[0], (char *)&remoteAddr_.sin_addr.s_addr, host->h_length);
	remoteAddr_.sin_port = htons(remotePort);

	if(bind(udpSocket_, (sockaddr*) &localAddr_, sizeof(sockaddr_in)) < 0) {
	  	std::cerr << "ERROR: [UdpSocket][UdpSocket] Failed to bind socket!" <<
			strerror(errno) << "! Exiting!" << std::endl;
		exit(-1);
	}

//	std::cout << "Connected to robot on " << remoteHost << ":" << remotePort << std::endl;
}

UdpSocket::~UdpSocket() {
	close(udpSocket_);
}

void UdpSocket::setNonBlock() {
	fcntl(udpSocket_, F_SETFL, O_NONBLOCK);
}

