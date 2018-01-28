#ifndef COMMON_H_
#define COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>


#define MIN(x,y)                 ((x) < (y) ? (x) : (y))
#define MAX(x,y)                 ((x) > (y) ? (x) : (y))
#define ABS(x)                 ((x) < 0 ? -(x) : (x))
#define CLAMP(x,mn,mx)       {if (x <= (mn)) x = (mn); else if (x >= (mx)) x = (mx);}
#define CORE(x,t)              {if (ABS(x) <= (t)) x = 0;}
#define MCORE(x,t)              {if (x > (t)) x -= (t); else if (x < -(t)) x += (t); else x = 0;}
#define CORRECT(x,mx,mn)  		(((float)(x-mn)/(float)(mx-mn)) - 0.5f)

//#define M_PI 		  3.1415927f
#define RAD2DEG(r)   ((r)*57.29577951f)
#define DEG2RAD(d)   ((d)*0.017453292f)

#define _180_DIV_PI         57.295779f
#define PI_DIV_180          0.017453292f

#endif
