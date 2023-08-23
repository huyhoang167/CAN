/*
 * timer.c
 *
 *  Created on: Aug 21, 2023
 *      Author: Hoang
 */

#include "timer.h"
// Timer1
int timer1_flag = 0;
int timer1_counter = 0;
// Timer2
int timer2_flag = 0;
int timer2_counter = 0;

void setTimer1(int duration) {
	timer1_counter = duration;
	timer1_flag = 0;
}
void setTimer2(int duration) {
	timer2_counter = duration;
	timer2_flag = 0;
}
void timerRun1() {
		timer1_counter--;
		if(timer1_counter <= 0) {
			timer1_flag = 1;
		}
}
void timerRun2(){
			timer2_counter--;
			if(timer2_counter <= 0) {
				timer2_flag = 1;
			}
}
void clearTimer1(){
	timer1_counter =0;
	timer1_flag = 0;
}
void clearTimer2(){
	timer2_counter =0;
	timer2_flag = 0;
}
