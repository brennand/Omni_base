/*
 *  Communication.cpp
 *  UDP
 *
 *  Created by Bren on 2/17/10.
 *  Copyright 2010 TUM. All rights reserved.
 *
 */
#include "Communication.h"
#include "headClass.h"

UDP::UDP(Head *head,char* const IP,int rmPort, int locPort) :

        remoteIP(IP),//remoteIP("127.0.0.1"), //remoteIP("10.0.200.168"),
        remotePort(rmPort),//1023
        localPort(locPort),//8000
        udpSocket(remoteIP, remotePort, localPort),
        head(head){

}


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

int UDP::set_int(uint8_t low,uint8_t high){
    type_swap temp;

    temp.ui8.hi = high;
    temp.ui8.lo = low;

    return temp.i16;
}

unsigned int UDP::set_uint(uint8_t low,uint8_t high){
    type_swap_unsigned uint_temp;

    uint_temp.ui8.hi = high;
    uint_temp.ui8.lo =low;
    return  uint_temp.i16;
}

unsigned int UDP::set_uint32(uint8_t low1,uint8_t low2, uint8_t high1,uint8_t high2){
    type_swap_unsigned32 temp;
    temp.ui8.hi1 = high2;
    temp.ui8.hi2 = high1;
    temp.ui8.lo1 = low2;
    temp.ui8.lo2 = low1;

    return temp.i32;
}

float UDP::set_float(uint8_t low1,uint8_t low2, uint8_t high1,uint8_t high2){
    type_swap_float temp;
    temp.ui8.hi1 = high2;
    temp.ui8.hi2 = high1;
    temp.ui8.lo1 = low2;
    temp.ui8.lo2 = low1;

    return temp.i32;
}



void UDP::add_8_package(int *Current_index, int value,int *crc){
    type_swap temp;

    temp.i16 = value;

    Set_packet(*Current_index, temp.ui8.hi);    *Current_index = *Current_index + 1;
    *crc = *crc ^ temp.ui8.hi;

}

void UDP::add_16_package(int *Current_index, int value,int *crc){
    type_swap temp;

    temp.i16 = value;

    Set_packet(*Current_index , temp.ui8.lo);   *Current_index = *Current_index + 1;
    Set_packet(*Current_index , temp.ui8.hi);   *Current_index = *Current_index + 1;
    *crc = *crc ^ temp.ui8.hi ^ temp.ui8.lo;

}

void UDP::add_32_package(int *Current_index, int value,int *crc){
    type_swap_unsigned32 temp32;

    temp32.i32 = value;
    Set_packet(*Current_index , temp32.ui8.lo2);      *Current_index = *Current_index + 1;
    Set_packet(*Current_index , temp32.ui8.lo1);      *Current_index = *Current_index + 1;
    Set_packet(*Current_index , temp32.ui8.hi2);      *Current_index = *Current_index + 1;
    Set_packet(*Current_index , temp32.ui8.hi1);      *Current_index = *Current_index + 1;
    *crc = *crc ^ temp32.ui8.lo2 ^ temp32.ui8.lo1 ^ temp32.ui8.hi2 ^ temp32.ui8.hi1;
}

void UDP::add_16_float_package(int *Current_index, int value,int *crc){
    type_swap temp;
    temp.i16 = value*1024;

    Set_packet(*Current_index, temp.ui8.lo);       *Current_index = *Current_index + 1;
    Set_packet(*Current_index, temp.ui8.hi);       *Current_index = *Current_index + 1;;
    *crc = *crc ^ temp.ui8.hi ^ temp.ui8.lo;
}

double UDP::get_16_double_package(int *Current_index){
    double temp = double(set_int(Get_packet(*Current_index),Get_packet(*Current_index+1))) / 1024;
    *Current_index = *Current_index + 2;
    return temp;
}

int UDP::get_16_int_package(int *Current_index){
    int temp = set_int(Get_packet(*Current_index),Get_packet(*Current_index+1));
    *Current_index = *Current_index + 2;
    return temp;
}
int UDP::get_16_uint_package(int *Current_index){
    int temp = set_uint(Get_packet(*Current_index),Get_packet(*Current_index+1));
    *Current_index = *Current_index + 2;
    return temp;
}

double UDP::get_32_double_package(int *Current_index){
    float temp = (set_float(Get_packet(*Current_index),Get_packet(*Current_index+1),Get_packet(*Current_index+2),Get_packet(*Current_index+3)));
    *Current_index = *Current_index + 4;
    return (double)temp;
}


int UDP::get_32_int_package(int *Current_index){
    int temp = int(set_uint32(Get_packet(*Current_index),Get_packet(*Current_index+1),Get_packet(*Current_index+2),Get_packet(*Current_index+3)));
    *Current_index = *Current_index + 4;

    return temp;
}

void UDP::Unzip_packet(){

    int index = 1;
    bool failed = false;
    
    switch (Get_packet(0)) {

        case off: break;

        case full_Controller:
         
            head->raw_encoder.clear();
            head->raw_vel.clear();
            head->pwm.clear();
            head->vel_filtered.clear();
            for (int i = 0; i < (int)head->name.size(); i++) {
               head->raw_encoder.push_back(get_16_uint_package(&index));
               head->raw_vel.push_back(get_16_int_package(&index));
               head->vel_filtered.push_back(get_32_double_package(&index));
               head->pwm.push_back(get_16_int_package(&index));               
            }


			case update_Values: break;

			case Test_UDP: 

            for (int i = 1; i < 148; i++) {
                if(Get_packet(i) != i){
                    failed = true;
                }
            }

            if(failed){

            }

        case LED_switch: break;


        default: break;
    
    }
}

void UDP::Zip_packet(){

    for (int i = 1; i < 148; i++) {
        Set_packet(i,00);
    }

    int index = 1;
    int crc = 0;
    switch (head->get_control){

        case off:				
            Set_packet(0,off);
            break;

        case full_Controller:
            Set_packet(0,full_Controller);

            for (int i = 0; i < (int)head->name.size(); i++) {
            	add_16_package(&index,head->ffc[i],&crc);
            	add_8_package(&index,head->mosfet_config[i],&crc);
            }

            break;

        case update_Values:	
            Set_packet(0,update_Values);
            break;

        case Test_UDP:	
            for (int i = 1; i < 148; i++) {
                Set_packet(i,i);
            }
        
            Set_packet(0,Test_UDP);
            break;

        case LED_switch:	
            Set_packet(0,LED_switch);
            add_8_package(&index,head->LED,&crc);
            break;




        default:
            Set_packet(0,0);
            break;
        }


}
