#ifndef SHAPE_H_INCLUDED
#define SHAPE_H_INCLUDED

#include <stdlib.h>
struct shape{
	float x,y,z;  //position
	float vx,vy,vz;  //current velocity
	float nx, ny, nz; //calculated new position
	float nvx, nvy, nvz; //calculated new velocity
	//int r, g, b;	//color, 1 number
};

#endif // SHAPE_H_INCLUDED
