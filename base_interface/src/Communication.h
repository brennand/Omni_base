/*
 *  Communication.h
 *  UDP
 *
 *  Created by Bren on 2/17/10.
 *  Copyright 2010 TUM. All rights reserved.
 *
 */

#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_


//My header files
#include "UdpSocket.h"
#include "baseClass.h"


class UDP {

	public:
		//method

		UDP(char* const ,int , int, int );

		//members

		// This two methods puts the vars into a array to be sent.
    void Unzip_packet(Base *base);
    void Zip_packet(Base *base);

		//function to Send and Receive a packet	
		bool Send();
		bool Received();

		//Adds a single 8bits to the array at the index location
		void Set_packet(int index,uint8_t value);
		uint8_t Get_packet(int index);

    //Base *base;

	private:
		//set up udp connection
		const char* remoteIP;
		int			remotePort;
		int			localPort;

		uint8_t		send_packet[148];
		uint8_t		rec_packet[148];

		int packet_index;

		UdpSocket	udpSocket;
};

#endif //_COMMUNICATION_H_
