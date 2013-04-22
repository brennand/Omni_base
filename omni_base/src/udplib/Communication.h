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
#include "headClass.h"

union type_swap {
        int16_t i16;
        struct hi_lo {
                uint8_t hi;
                uint8_t lo;
        } ui8;
};
union type_swap_unsigned {
        uint16_t i16;
        struct hi_lo {
                uint8_t hi;
                uint8_t lo;
        } ui8;
};

union type_swap_unsigned32{
				uint32_t i32;
				struct hi_lo {
				        uint8_t hi1;
				        uint8_t hi2;
				        uint8_t lo1;
				        uint8_t lo2;
				} ui8;
};

union type_swap_float{
				float i32;
				struct hi_lo {
				        uint8_t hi1;
				        uint8_t hi2;
				        uint8_t lo1;
				        uint8_t lo2;
				} ui8;
};

class UDP {

	public:
		//method
                UDP(Head *,char* const ,int , int );

		//members
		bool Send();
		bool Received();

		void Set_packet(int index,uint8_t value);
		uint8_t Get_packet(int index);

                void Unzip_packet();
                void Zip_packet();

                int set_int(uint8_t low,uint8_t high);
                unsigned int set_uint(uint8_t low,uint8_t high);
                unsigned int set_uint32(uint8_t low1,uint8_t low2, uint8_t high1,uint8_t high2);
                float set_float(uint8_t low1,uint8_t low2, uint8_t high1,uint8_t high2);

                void add_8_package(int *Current_index, int value,int *crc);
                void add_16_package(int *Current_index, int value,int *crc);
                void add_32_package(int *Current_index, int value,int *crc);
                void add_16_float_package(int *Current_index, int value,int *crc);

                int get_16_int_package(int *Current_index);
                int get_32_int_package(int *Current_index);
                int get_16_uint_package(int *Current_index);
                double get_16_double_package(int *Current_index);
                double get_32_double_package(int *Current_index);




//                void SetupSocket(int i);
                Head *head;

	private:
		//set up udp connection
		const char* remoteIP;
		int			remotePort;
		int			localPort;

		uint8_t		send_packet[148];
		uint8_t		rec_packet[148];

		UdpSocket	udpSocket;
};

#endif //_COMMUNICATION_H_
