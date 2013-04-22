/*
 *  Communication.cpp
 *  UDP
 *
 *  Created by Bren on 2/17/10.
 *  Copyright 2010 TUM. All rights reserved.
 *
 */
#include <iostream>
#include "Communication.h"
#include "baseClass.h"

UDP::UDP(char* const IP,int rmPort, int locPort, int index) :

        remoteIP(IP),//remoteIP("127.0.0.1"), //remoteIP("10.0.200.168"),
        remotePort(rmPort),//1023
        localPort(locPort),//8000
        udpSocket(remoteIP, remotePort, localPort),
				packet_index(index){

}

void UDP::Unzip_packet(Base *base){

    uint8_t *pArray;
		pArray = &rec_packet[1];

    switch (Get_packet(0)) {

        case off: break;

        case full_Controller:
            
    				for (int i = packet_index; i < (packet_index+2); i++) {
							base->raw_pos[i] 							= * (int32_t*) pArray;		pArray = pArray + 4;
            	base->raw_pos_revolutions[i] 	= * (int32_t*) pArray;		pArray = pArray + 4;
							base->raw_vel[i] 							= * (int16_t*) pArray;		pArray = pArray + 2;
							base->vel_filtered[i]					= * (float*)   pArray;		pArray = pArray + 4;
							base->pwm[i] 									= * (int16_t*) pArray;		pArray = pArray + 2;
							base->error_code[i]						= * (uint8_t*) pArray;		pArray = pArray + 1;
							base->error_code[i]						=  (uint8_t)packet_index+1;
            }
            break;

        case LED_switch: break;


        default: break;
    
    }
}

void UDP::Zip_packet(Base *base){

    for (int i = 1; i < 148; i++) {
        Set_packet(i,00);
    }
    
    uint8_t *pArray;
		pArray = &send_packet[1];
    
    switch (base->get_control){

        case off:				
            Set_packet(0,off);
            break;

        case full_Controller:
  
            Set_packet(0,full_Controller); // Make sure the packet has the correct control type

            for (int i = packet_index; i < (packet_index+2); i++) {
            	* (int16_t*) pArray = (int16_t) base->ffc[i];  						pArray = pArray + 2;         	
            	* (uint8_t*) pArray = (uint8_t) base->mosfet_config[i];  	pArray = pArray + 1; 
            }
            * (uint8_t*) pArray = (uint8_t) base->LED;
						break;

        case LED_switch:	
            Set_packet(0,LED_switch);
            * (uint8_t*) pArray = (uint8_t) base->LED;
            break;

        default:
            Set_packet(0,0);
            break;
        }


}


/*********************************************************************************
*
*					simple functions after this
*
*********************************************************************************/


bool UDP::Send(){
    if (!udpSocket.sendPacket(send_packet)) {

        return false;
    }else {
        return true;
    }

}

bool UDP::Received(){
    if(udpSocket.recvPacket(rec_packet)) {
        return true;
    }else {

        return false;
    }

}


void UDP::Set_packet(int index,uint8_t value){
    send_packet[index]= value;
}


uint8_t UDP::Get_packet(int index){
    return rec_packet[index];
}
