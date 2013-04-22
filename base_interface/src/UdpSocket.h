/*
 *A simple class for setting up a udp connection between two processes to exchange random data.
 *WARNING: the processes have to take care of a adequate connection and checking of valid
 *datatypes.
 */

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>

class UdpSocket {
	public:
	//connection parameter for remote machine and recvfrom timeout in msecs
                UdpSocket(const char* remoteIP, const int remotePort, const int localPort, const int timeout = 1);
		~UdpSocket();

		template <class T>
			bool recvPacket(T &packet);
		template <class T>
			bool sendPacket(T &packet);

		void setNonBlock();

	private:
		int udpSocket_;
		struct sockaddr_in remoteAddr_, localAddr_;
		socklen_t localAddrLen_;
};


//template functions for sending and receiving data
//WARNING: Make sure you are sending valid datatypes. Check size, 32/64 bit safety, serial data

//TODO return int depending on status of recvfrom
template <class T>
bool UdpSocket::recvPacket(T &packet) {
	int size = recvfrom(udpSocket_, (void*) &packet, sizeof(T), 0,
			(sockaddr*) &localAddr_, &localAddrLen_);

        if(size == sizeof(T))
                return true;
        else if(errno == EAGAIN) {
                //std::cout << "WARNING: [UdpSocket][recvPacket] Receive timed out! " <<
                //	strerror(errno) << std::endl;
                return false;
        }
        else if(size == -1) {
                //std::cout << "WARNING: [UdpSocket][recvPacket] " <<	strerror(errno) << std::endl;
                //return false;
        }
        else {
                //std::cerr << "ERROR: [UdpSocket][recvPacket] Bad packet length " << size  <<
                //	" (expected: " << sizeof(T) << ")! "  << strerror(errno) << std::endl;
                return false;
        }

            return false;
}

template <class T>
bool UdpSocket::sendPacket(T &packet) {
	if(sendto(udpSocket_, (void*) &packet, sizeof(T), 0,
				(sockaddr*) &remoteAddr_, sizeof(remoteAddr_)) == -1) {

        //	std::cerr << "ERROR: [UdpSocket][sendPacket] Failed to send packet! " <<
//			strerror(errno) << std::endl;

		return false;
	}
	else
		return true;

}

#endif

