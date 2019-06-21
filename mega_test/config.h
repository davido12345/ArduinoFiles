#include <Arduino.h>
// set to 1 if the desired output is 0-12V with safty range in 3-9V and use the green terminal blocks 
// set to 0 if the desired output range from outputL to outputH  (0<= outputL < outputH <=5 ) 
const int mode = 1 ;
float outputH = 4 ;   // max output voltage: outputH V
float outputL = 3 ;   // min output voltage: outputL V
