/************************************************************************/
// File:			functions.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Various functions needed
//
/************************************************************************/

#include <avr/io.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Take any angle and put it inside -pi,pi */
void vFunc_Inf2pi(float *angle_in_radians){
    do{
        if (*angle_in_radians > M_PI) *angle_in_radians -= 2*M_PI;
        else if (*angle_in_radians < -M_PI) *angle_in_radians += 2*M_PI;
    } while (fabs(*angle_in_radians) > M_PI);
}

// parse the update message from uart
void vFunc_ParseUpdate(char *cin, float *theta, int16_t *radius){
    char *token;
    const char delimiters[] = "{U,}\n";
    token = strtok(cin, delimiters);
    uint8_t i = 0;
    while (token != NULL){
        switch (i)
        {
            case 0:
                *theta = atoi(token);
            break;
            
            case 1:{
                *radius = atoi(token);
            break;}
            
            default:
            break;
        }
        token = strtok(NULL, delimiters);
        i++;
    }
}

// reverses a string 'str' of length 'len'
void vFunc_reverse(char* p, char* q)
{
    char c;
    for(; p < q; ++p, --q){
        c = *p;
        *p = *q;
        *q = c;
    }
}

// increase function for decimal parsing
char* vFunc_inc(char* s, char* e)
{
    int co = 1;
    char* t = e;
    
    //increase from end to start
    for(; t >= s; --t){
        if(*t == '.') continue;
        *t += co;
        if(*t > '9'){
            *t = '0';
            co = 1;
        }
        else{
            co = 0;
            break;
        }
    }
    //check if there's still carry out
    if(co){
        for(t = ++e; t > s; --t) *t = *(t - 1);
        *s = '1';
    }
    return e;
}

// C program for implementation of ftoa()
char* vFunc_ftoa(double num, char* dest, int afterPoint)
{
    char* p = dest;
    int integer = (int)num;
    double decimal = num - integer;
    
    //parse sign
    if(num < 0){
        *p++ = '-';
        integer = -integer;
        decimal = -decimal;
    }
    //parse integer
    if(integer){
        char* q = p;
        for(; integer; integer /= 10){
            *q++ = '0' + integer % 10;
        }
        vFunc_reverse(p, q - 1);
        p = q;
    }
    else *p++ ='0';
    //parse decimal
    if(afterPoint > 0){
        *p++ ='.';
        for(; afterPoint; --afterPoint){
            decimal *= 10;
            *p++ = '0' + (int)decimal;
            decimal -= (int)decimal;
        }
        if((int)(decimal + 0.5)) p = vFunc_inc(dest, p - 1) + 1;
    }
    
    *p = '\0';
    return dest;
}

/* Legacy functions that may become useful for future work */
/**
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Radicaly changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed and moved to own file by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @file position.c
 * @brief position routines
 */
/**
 *  calculates the x-position of a measured object. 
 */
 int16_t objectPosX(int16_t i16_sensorT, uint8_t ui8_sensorDelta, int8_t i8_servoAngle, float f_robotTheta, float f_robotX){	
	ui8_sensorDelta+=2;
	//i16_servoAngle negative because -90 is on left side and +90 on right side
	float f_theta   = ((float)(i16_sensorT+i8_servoAngle))/180*M_PI + f_robotTheta;
	float f_objectX = f_robotX - (32*cos(f_robotTheta)) + ( ((float)ui8_sensorDelta*10) * cos(f_theta));
	return((int16_t)f_objectX);
}

/**
 *  calculates the y-position of a measured object.
 */
 int16_t objectPosY(int16_t i16_sensorT, uint8_t ui8_sensorDelta, int8_t i8_servoAngle, float f_robotTheta, float f_robotY){
	ui8_sensorDelta+=2;
	//i16_servoAngle negative because -90 is on left side and +90 on right side
	float f_theta   = ((float)(i16_sensorT+i8_servoAngle))*M_PI/180 + f_robotTheta;
	float f_objectY = f_robotY - (32*sin(f_robotTheta)) + ( ((float)ui8_sensorDelta*10) * sin(f_theta));
	
	return((int16_t)f_objectY);
}

/**
 * Calculates a heading and a distance to move to a position
 */
void goRobotPos(int16_t i16_goX, int16_t i16_goY, float f_robotX, float f_robotY, uint8_t *ui8_targetHeading, int8_t *i8_targetDistance){
	float f_goX           = (float) i16_goX;
	float f_goY           = (float) i16_goY;
	float diffXSquareInCm = ((f_goX-f_robotX)*(f_goX-f_robotX))/100;
	float diffYSquareInCm = ((f_goY-f_robotY)*(f_goY-f_robotY))/100;
	int16_t i16_angle       = (int16_t) (atan2(f_goY-f_robotY,f_goX-f_robotX)*180/M_PI);
	if (i16_angle<0){
		i16_angle+=360;
	}
	*ui8_targetHeading      = i16_angle/2;
	*i8_targetDistance     = (int8_t)sqrt(diffXSquareInCm+diffYSquareInCm);
}