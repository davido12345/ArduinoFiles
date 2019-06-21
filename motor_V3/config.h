#include <Arduino.h>
// slow start time setting in ms. After SStime milliseconds, the ventilator will work in full speed.
int SStime=3000; //3s

// time setting for the ventilator to slow down between the state change of clean and cool
int slowdowntime=1000; //0.5s

// time setting for clean
int cleantime=30000; //30s

// time setting for cool
int cooltime=15000; //15s 

//set enableTem to 0 then the ventilator will always on. set enableTem to 1 then Ventilator starts at a temperature of tem_H 
// when temperature is higher than tem_H, the ventilator start running, it stops when temperture is lower than tem_H
// The range of the temperature sensor is 40℃ - 120℃
int enableTem=1;   
int tem_L=32;
int tem_H=35;
