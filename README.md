# cloth_wind_ball
A simulation of a cloth held from the top corners with wind, gravity, and collision (shape of a ball) all affecting the cloth. This was done for a computer graphics algorithm class. The physics was done using Verlet integration.

# Requirements
* FreeGLUT

# Controls
* esc: exit
* space: pause/unpase
* R: reset simulation
* Q,W: decrease/increase dampening by .01
* A,S: decrease/increase rest length by .1
* Z,X: decrease/increase diagonal rest length by .1
* D,F: decrease/increase gravity by .1
 	(will not take affect until reset)
* C,V: decrease/increase number of iterations by 1
 	(reseted to default at reset)
* T,Y: decrease/increase x direction wind by .01
* G,H: decrease/increase y direction wind by .01
* B,N: decrease/increase z direction wind by .01 (This starts at 1.00)
