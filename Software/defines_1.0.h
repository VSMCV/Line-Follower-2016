/*
 * defines.h
 *
 *  Created on: 7 februarie 2016
 *      Author: Vlad
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define rc_power 16752735 //3785549847 remote control code
#define port0_in_location 0x40040024 //memory location for line sensor input register
#define p1 15 //line sensor pin numbers
#define p2 14
#define p3 9
#define p4 8
#define p5 7
#define p6 6
#define p7 5
#define p8 0

#define initial_delay 10000 //may cause robot to fail to recognize initial color if lowered
#define avoid_wheelie_time 10000 //set to 1000 for instant max speed
#define samples_out_limit 1000 //number of samples allowed outside the line before the robot stops
#define sampling_time 1000
#define bump_time 1000000 //microseconds spent at slow speed for bump crossing
#define bridge_time 1000000 //microseconds spent at slow speed for bridge crossing
#define color_switch_time 100000. //microseconds from magnet detection until color switch

#define i1 (-3) //sensor coefficients. Calin, don't!
#define i2 (-2)
#define i3 (-1)
#define i4 0
#define i5 0
#define i6 1
#define i7 2
#define i8 3

#define maxi 10 //limits the value of i component

#define initial_vref 2000 //initial power level
#define max_vref 10000 //10000 mandatory!!!
#define bump_vref 5000 //bump speed - please adjust!
#define bridge_vref 5000 //bridge speed - please adjust!
#define decrease_vref 2000

#define kp 5000 //PID constants
#define ki 2000
#define kd 7000

#endif /* DEFINES_H_ */
