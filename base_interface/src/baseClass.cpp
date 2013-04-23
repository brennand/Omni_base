/*
 *  Head
 *
 *  Created by Bren on 6/1/11.
 *  Copyright 2011 TUM. All rights reserved.
 *
 */

#include "baseClass.h"

Base::Base() {}

double Base::omnidrive_limit(double x, double max, double min){
	if(x>max) {
		return max;
	}
	
	if(x<min){
		return min;
	}
	
	return x;
}