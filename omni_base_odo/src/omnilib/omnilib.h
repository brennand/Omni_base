/*
 * This file is part of the libomnidrive project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *                    Ingo Kresse <kresse@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#ifndef OMNIDRIVE_H 
#define OMNIDRIVE_H 

void omnidrive_set_correction(double drift);
int omnidrive_odometry(double Vel1,double Vel2,double Vel3,double Vel4,double Pos1,double Pos2,double Pos3,double Pos4,double *x, double *y, double *a);

double omnidrive_limit(double x, double l);


#endif