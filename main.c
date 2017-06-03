//Derik Vega
#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "shape.h" //SHOULD BE REMOVED
#include "particlesystem.h"

#define BALL_POSITION_X 6
#define BALL_POSITION_Y -2
#define BALL_POSITION_Z 0
#define BALL_RADIUS 0.5
#define BIG_RADIUS 5
#define TRUE 1
#define FALSE 0
//#define SPRING_MIN 2.4
//#define SPRING_REST 2.7
//#define SPRING_MAX 3.4
#define SPRING_MIN 2.0
#define SPRING_REST 2.3
#define SPRING_MAX 2.6
//#define DIAG_SPRING_MIN 2.83
//#define DIAG_SPRING_REST 3.25
//#define DIAG_SPRING_MAX 3.68
#define SPRING_K 400
#define FAR_SPRING_K 500
#define TIMESTEP 10 //100 increment per second, default: 10
//#define DAMPENING .7
#define MASS 1
//#define RESTL 3
//#define DRESTL 4.24264
//#define DRESTL 3

// *************************************************************************************
// * GLOBAL variables. Not ideal but necessary to get around limitations of GLUT API... *
// *************************************************************************************
int pause = FALSE;
struct shape ball, nodes[NODEAMOUNT][NODEAMOUNT];
int length = 3;
int oldtime, newtime; //Delta-time
float gravity = 10; // round to 10
float DIAG_SPRING_MIN, DIAG_SPRING_REST, DIAG_SPRING_MAX;
float DIAG_SPRING_K, DIAG_FAR_SPRING_K;
float fTimeStep = .1;
float DAMPENING = .8;
float RESTL = 1.5, DRESTL = 1.9; //REST DISTANCE
int NUM_CONSTRAINTS;
struct Vector3 wind, movingball;
float wx = 0, wy = 0, wz = 1;
float ballpi = 0;
int NUM_BALL_ITERATIONS;
int windhappens;

float getdistance3d (struct shape s1, struct shape s2);
float hookes (struct shape s1, struct shape s2);
float diaghookes (struct shape s1, struct shape s2);
float getunitvectx (struct shape s1, struct shape s2);
float getunitvecty (struct shape s1, struct shape s2);
float getunitvectz (struct shape s1, struct shape s2);
void getangles (struct shape s1, struct shape s2, float *theta, float *phi);
void handlesprings (struct shape *s1, struct shape *s2, float *fx, float *fy, float *fz);
void diaghandlesprings (struct shape *s1, struct shape *s2, float *fx, float *fy, float *fz);
void satisfyconstraints (struct shape *s1, struct shape *s2);
void diagsatisfyconstraints (struct shape *s1, struct shape *s2);
void resetsim();
struct Vector3 crossproduct(struct Vector3 v1, struct Vector3 v2);
void addwindf(int xy, float windf);

void init (void)
{
	glShadeModel (GL_SMOOTH);
	glClearColor (0.2f, 0.2f, 0.4f, 0.5f);
	glClearDepth (1.0f);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);
	glEnable (GL_COLOR_MATERIAL);
	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable (GL_LIGHTING);
	glEnable (GL_LIGHT0);
	GLfloat lightPos[4] = {-1.0, 1.0, 0.5, 0.0};
	glLightfv (GL_LIGHT0, GL_POSITION, (GLfloat *) &lightPos);
	glEnable (GL_LIGHT1);
	GLfloat lightAmbient1[4] = {0.0, 0.0,  0.0, 0.0};
	GLfloat lightPos1[4]     = {1.0, 0.0, -0.2, 0.0};
	GLfloat lightDiffuse1[4] = {0.5, 0.5,  0.3, 0.0};
	glLightfv (GL_LIGHT1,GL_POSITION, (GLfloat *) &lightPos1);
	glLightfv (GL_LIGHT1,GL_AMBIENT, (GLfloat *) &lightAmbient1);
	glLightfv (GL_LIGHT1,GL_DIFFUSE, (GLfloat *) &lightDiffuse1);
	glLightModeli (GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
}

void display (void)
{
	int dumi = 0, n, m; //int dummy
	float storedist = 0, springv; //float dummy
	float theta, phi;
	float dumf = 0, dumf2, vmag, reangle;
	float fx=0, fy=0, fz=0;
	float ax=0, ay=0, az=0;
	float x=0, y=0, z=0; //TEMP COORDINATE

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity ();
	glDisable (GL_LIGHTING);
	glBegin (GL_POLYGON);
	glColor3f (0.8f, 0.8f, 1.0f);
	glVertex3f (-200.0f, -100.0f, -100.0f);
	glVertex3f (200.0f, -100.0f, -100.0f);
	glColor3f (0.4f, 0.4f, 0.8f);
	glVertex3f (200.0f, 100.0f, -100.0f);
	glVertex3f (-200.0f, 100.0f, -100.0f);
	glEnd ();
	glEnable (GL_LIGHTING);
	glTranslatef (-6.5, 6, -9.0f); // move camera out and center on the rope
	//glTranslatef (70, 60, -100.0f); //own camera

	/*glPushMatrix (); //Draw big ball
	glTranslatef (ball.x, ball.y, ball.z);
	glColor3f (1.0f, 0.0f, 0.0f);
	glutSolidSphere (BIG_RADIUS - 0.1, 16, 16); // draw the ball, but with a slightly lower radius, otherwise we could get ugly visual artifacts of rope penetrating the ball slightly
	glPopMatrix ();*/

	/*for(n = 888; n < NODEAMOUNT; n++) //render the ROPE
	{
		for(m = 0; m < NODEAMOUNT; m++)
		{
			glPushMatrix (); //Draw next sphere
			glTranslatef (nodes[clothxy(n,m)].x, nodes[clothxy(n,m)].y, nodes[clothxy(n,m)].z);
			//printf("n: %d  x: %f y: %f z: %f\n", n, nodes[clothxy(n,m)].x, nodes[clothxy(n,m)].y, nodes[clothxy(n,m)].z);
			glColor3f (1.0f, 1.0f, 0.0f);
			glutSolidSphere (BALL_RADIUS - 0.1, 4, 4); // draw the ball, but with a slightly lower radius, otherwise we could get ugly visual artifacts of rope penetrating the ball slightly
			glPopMatrix ();
		}
	}*/
	glPushMatrix (); //Draw moving ball
	glTranslatef (movingball.x, movingball.y, movingball.z);
	glColor3f (1.0f, 0.0f, 0.0f);
	glutSolidSphere (BIG_RADIUS - 0.1, 8, 8); // draw the ball, but with a slightly lower radius, otherwise we could get ugly visual artifacts of rope penetrating the ball slightly
	glPopMatrix ();
	/*movingball.x = 20 * sinf(ballpi);
	movingball.z = -60 + 20 * cosf(ballpi);
	ballpi += .01;
	if(ballpi >= 2 * M_PI){
		ballpi = 0;
	}*/

	for(n = 0; n < NODEAMOUNT-1; n++) //render the CLOTH
	{
		for(m = 0; m < NODEAMOUNT-1; m++)
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glBegin (GL_POLYGON);
			glColor3f ( (n+m) % 2 /2, (n+m) % 2, 1);
			glVertex3f(m_x[clothxy(n,   m)].x, m_x[clothxy(n,m)].y, m_x[clothxy(n,m)].z);
			glVertex3f(m_x[clothxy(n,   m+1)].x, m_x[clothxy(n,m+1)].y, m_x[clothxy(n,m+1)].z);
			glVertex3f(m_x[clothxy(n+1, m+1)].x, m_x[clothxy(n+1, m+1)].y, m_x[clothxy(n+1, m+1)].z);
			glVertex3f(m_x[clothxy(n+1, m)].x, m_x[clothxy(n+1, m)].y, m_x[clothxy(n+1, m)].z);
			glEnd();
		}
	}
	glFlush();

	glutSwapBuffers();
	glutPostRedisplay();

	//if(((newtime - oldtime) > TIMESTEP - 1) && pause) //Once timestep complete
	if(pause) //REMOVED STEPS
	{
		wind.x = wx; //WIND VECTOR
		wind.y = wy;
		wind.z = wz;
		m_vGravity.y = -gravity; //SET GRAVITY
		movingball.x = 20 * sinf(ballpi); //MOVE BALL
		movingball.z = -60 + 20 * cosf(ballpi);
		ballpi += .01;
		if(ballpi >= 2 * M_PI){ //CONSTRAIN TO < 2 PI
			ballpi = 0;
		}
		TimeStep();
		oldtime = newtime; //start another count

		for(n = 0; n < NUM_PARTICLES; n++){
			//printf("n: %d x: %f y: %f z: %f\n", n, m_x[n].x, m_x[n].y, m_x[n].z);
		}
	}

	//printf("%d, %d\n", oldtime, newtime);
	newtime = glutGet(GLUT_ELAPSED_TIME); //Get new time
}

void handlesprings (struct shape *s1, struct shape *s2, float *fx, float *fy, float *fz)
{
	float storedist, dumf, theta, phi;
	storedist = getdistance3d (*s1, *s2);
	if((SPRING_REST - storedist) != 0) //HANDLE SPRING
	{
		dumf = hookes(*s1, *s2); //velocity from spring
		*fx += dumf * (s2->x - s1->x)/storedist;
		*fy += dumf * (s2->y - s1->y)/storedist;
		*fz += dumf * (s2->z - s1->z)/storedist;
	}
}

void diaghandlesprings (struct shape *s1, struct shape *s2, float *fx, float *fy, float *fz)
{
	float storedist, dumf, theta, phi;
	storedist = getdistance3d (*s1, *s2);
	if((SPRING_REST - storedist) != 0) //HANDLE SPRING
	{
		dumf = diaghookes(*s1, *s2); //velocity from spring
		*fx += dumf * (s2->x - s1->x)/storedist;
		*fy += dumf * (s2->y - s1->y)/storedist;
		*fz += dumf * (s2->z - s1->z)/storedist;
	}
}

void satisfyconstraints (struct shape *s1, struct shape *s2)
{
	float delta, deltalength, diff;
	delta = s2->x - s1->x;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - SPRING_REST)/(deltalength * (2/MASS));
	s1->nx -= MASS*delta*diff;
	s2->nx += MASS*delta*diff;

	delta = s2->y - s1->y;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - SPRING_REST)/(deltalength * (2/MASS));
	s1->ny -= MASS*delta*diff;
	s2->ny += MASS*delta*diff;

	delta = s2->z - s1->z;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - SPRING_REST)/(deltalength * (2/MASS));
	s1->nz -= MASS*delta*diff;
	s2->nz += MASS*delta*diff;
}

void diagsatisfyconstraints (struct shape *s1, struct shape *s2)
{
	float delta, deltalength, diff;
	delta = s2->x - s1->x;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - DIAG_SPRING_REST)/(deltalength * (2/MASS));
	s1->nx -= MASS * delta * diff;
	s2->nx += MASS * delta * diff;

	delta = s2->y - s1->y;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - DIAG_SPRING_REST)/(deltalength * (2/MASS));
	s1->ny -= MASS*delta*diff;
	s2->ny += MASS*delta*diff;

	delta = s2->z - s1->z;
	deltalength = sqrt(delta * delta);
	diff = (deltalength - DIAG_SPRING_REST)/(deltalength * (2/MASS));
	s1->nz -= MASS*delta*diff;
	s2->nz += MASS*delta*diff;
}

float getdistance3d (struct shape s1, struct shape s2) //xyz coordinate
{
	float x = s2.x - s1.x;
	float y = s2.y - s1.y;
	float z = s2.z - s1.z;
	return sqrt((x * x) + (y * y) + (z * z));
}

float hookes (struct shape s1, struct shape s2) //returns the force
{
    float a, v, dist;
    dist = getdistance3d(s1, s2);
	dist = SPRING_REST - dist;
	//printf("dist from spring: %f\n", dist);
	if((dist > SPRING_MAX) || (dist < SPRING_MIN)) //EXCEEDING SPRING THRESHOLD
	{
		return FAR_SPRING_K * dist/2;
	}
	return SPRING_K * dist / 2;
}

float diaghookes (struct shape s1, struct shape s2) //returns the force
{
    float a, v, dist;
    dist = getdistance3d(s1, s2);
	dist = SPRING_REST - dist;
	//printf("dist from spring: %f\n", dist);
	if((dist > DIAG_SPRING_MAX) || (dist < DIAG_SPRING_MIN)) //EXCEEDING SPRING THRESHOLD
	{
		return DIAG_FAR_SPRING_K * dist/2;
	}
	return DIAG_SPRING_K * dist / 2;
}

float getunitvectx (struct shape s1, struct shape s2)
{
	float dumf = getdistance3d (s1, s2);
	float x = s2.x - s1.x;
	return x / dumf;
}

float getunitvecty (struct shape s1, struct shape s2)
{
	float dumf = getdistance3d (s1, s2);
	float y = s2.y - s1.y;
	return y / dumf;
}

float getunitvectz (struct shape s1, struct shape s2)
{
	float dumf = getdistance3d (s1, s2);
	float z = s2.z - s1.z;
	return z / dumf;
}

void getangles (struct shape s1, struct shape s2, float *theta, float *phi) //s2: current node
{
	float x, y, z;
	float r, p;
	x = s2.x - s1.x;
	y = s2.y - s1.y;
	z = s2.z - s1.z;
	r = sqrtf((x*x) + (y*y) + (z*z));
	p = sqrtf((x*x) + (y*y));
	*theta = acosf(z/r);
	*phi = atanf(y/x);
	if(x < 0) //IF VECTOR IS IN 4th QUADRANT
	{
		*phi += M_PI;
	}
	if(*phi < 0) //MAKE PHI: 0 <= phi <= 2 * M_PI
	{
		*phi += 2 * M_PI;
	}
}

//*****************
//NEW FUNCTIONS
//SET CONSTRAINT
void SetConstraint(int A, int B, float rest, int *id){
	m_constraints[*id].particleA = A;
	m_constraints[*id].particleB = B;
	m_constraints[*id].restlength = rest;
	*id+=1;
}

//GET XY OF CLOTH and VICE VERSA
int clothxy(int x, int y){
	return x + (y*50); //(0,0) -> (x,y) standard array procedure
}
int getclothx(int particle){
	return particle % 50;
}
int getclothy(int particle){
	return particle / 50;
}

void TimeStep() {
	AccumulateForces();
	Verlet();
	SatisfyConstraints();
}

// This function should accumulate forces for each particle
void AccumulateForces(){
	int i, n, m;
	struct Vector3 a, b;
	float windf, magn;
	// All particles are influenced by gravity
	for(i=0; i<NUM_PARTICLES; i++){
		m_a[i].y += m_vGravity.y;
	}
	// Wind
	windhappens++;
	if(windhappens >= 150){
		windhappens = -150;
	}
	//printf("windhappens: %d\n", windhappens);
	for(n = 0; n<NODEAMOUNT-1; n++) //0-48  (n,m) = (x,y)
	{
		for(m = 0; m<NODEAMOUNT-1; m++)
		{
			a.x = m_x[clothxy(n+1,m)].x - m_x[clothxy(n,m)].x;
			a.y = m_x[clothxy(n+1,m)].y - m_x[clothxy(n,m)].y;
			a.z = m_x[clothxy(n+1,m)].z - m_x[clothxy(n,m)].z;

			b.x = m_x[clothxy(n,m+1)].x - m_x[clothxy(n,m)].x;
			b.y = m_x[clothxy(n,m+1)].y - m_x[clothxy(n,m)].y;
			b.z = m_x[clothxy(n,m+1)].z - m_x[clothxy(n,m)].z;
			a = crossproduct(a,b);
			magn = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
			a.x = a.x / magn;
			a.y = a.z / magn;
			a.y = a.z / magn;
			windf = (a.x * wind.x) + (a.y * wind.y) + (a.z * wind.z);
			windf = windf * (150-abs(windhappens))/50;
			/*if((windhappens >= 50) && (windhappens < 60)){
				windf *= 1.5;
			}
			else if((windhappens >= 60) && (windhappens < 70)){
				windf *= 2;
			}
			else if((windhappens >= 70) && (windhappens < 80)){
				windf *= 1.5;
			}*/
			addwindf(clothxy(n,m), windf);
			addwindf(clothxy(n+1,m), windf);
			addwindf(clothxy(n,m+1), windf);
		}
	}
}

// Verlet integration step
void Verlet() {
	struct Vector3 x, temp, oldx, a;
	int i;
	for(i=0; i<NUM_PARTICLES; i++) {
		x = m_x[i];
		temp = x;
		oldx = m_oldx[i];
		a = m_a[i];
		x.x += (x.x - oldx.x)*(DAMPENING) + a.x * fTimeStep*fTimeStep;  //<-- a*t^2
		x.y += (x.y - oldx.y)*(DAMPENING) + a.y * fTimeStep*fTimeStep;  //<-- a*t^2
		x.z += (x.z - oldx.z)*(DAMPENING) + a.z * fTimeStep*fTimeStep;  //<-- a*t^2
		m_x[i] = x; //set new position
		oldx = temp;
		m_oldx[i] = oldx; //set old to previous position
		m_a[i].x = 0;
		m_a[i].y = 0;
		m_a[i].z = 0;
		//printf("i: %d x: %f y: %f z: %f\n", i, m_x[i].x, m_x[i].y, m_x[i].z);
	}
}

// Assume that an array of constraints, m_constraints, exists
void SatisfyConstraints() {
	struct Vector3 delta;
	struct Vector3 x1, x2;
	struct Constraint c;
	float deltalength, diff;
	int i, j, a;
	for(j=0; j<NUM_ITERATIONS; j++) {
		//HANDLE BALL COLISSION
		for(a=0; a<NUM_BALL_ITERATIONS; a++){ //MULTIPLE ITERATIONS
			for(i=0; i<NUM_PARTICLES; i++){
				delta.x = m_x[i].x - movingball.x; //GET DISTANCE
				delta.y = m_x[i].y - movingball.y;
				delta.z = m_x[i].z - movingball.z;
				deltalength = sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
				if(deltalength < BIG_RADIUS){
					delta.x = delta.x / deltalength; //NORMALIZE
					delta.y = delta.y / deltalength;
					delta.z = delta.z / deltalength;
					delta.x *= (BIG_RADIUS - deltalength); //CALCULATE HOW MUCH TO MOVE
					delta.y *= (BIG_RADIUS - deltalength);
					delta.z *= (BIG_RADIUS - deltalength);
					m_x[i].x += delta.x; //APPLY CHANGES
					m_x[i].y += delta.y;
					m_x[i].z += delta.z;
				}
			}
		}

		for(i=0; i<NUM_CONSTRAINTS; i++) {
		//for(i=0; i<4900; i++) { //4-DIRECTIONS ONLY
		//for(i=4900; i<NUM_CONSTRAINTS; i++) { //DIAGONALS ONLY
		//for(i=NUM_CONSTRAINTS-1; i>=0; i--) { //REVERSE
			c = m_constraints[i];
			x1 = m_x[c.particleA];
			x2 = m_x[c.particleB];
			//GET DELTA
			delta.x = x2.x - x1.x;
			delta.y = x2.y - x1.y;
			delta.z = x2.z - x1.z;
			deltalength = sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
			diff = (deltalength - c.restlength) / deltalength;
			x1.x += delta.x * 0.5*diff;
			x1.y += delta.y * 0.5*diff;
			x1.z += delta.z * 0.5*diff;

			x2.x -= delta.x * 0.5*diff;
			x2.y -= delta.y * 0.5*diff;
			x2.z -= delta.z * 0.5*diff;
			m_x[c.particleA] = x1;
			m_x[c.particleB] = x2;
			//printf("i: %d x1.x: %f x1.y: %f x1.z: %f\n", i, x1.x, x1.y, x1.z);
		}
		// Constrain one particle of the cloth to origo
		//m_x[0] = Vector3(0,0,0);
		m_x[clothxy(0,49)] = anchorl;
		m_x[clothxy(49,49)] = anchorr;
		m_x[clothxy(1,49)] = anchorlr;
		m_x[clothxy(0,48)] = anchorld;
		m_x[clothxy(48,49)] = anchorrl;
		m_x[clothxy(49,48)] = anchorrd;
	}
}

void resetsim(){
	int n, m;
	int constrcounter = 0; //CONSTRAINT COUNTER
	for(n = 0; n<NODEAMOUNT; n++) //0-50
	{
		for(m = 0; m<NODEAMOUNT; m++)
		{
			m_x[clothxy(n,m)].x = -37 + (1.5 * n); //COORDINATES
			m_x[clothxy(n,m)].y = -37 + (1.5 * m);
			m_x[clothxy(n,m)].z = -50;
			m_oldx[clothxy(n,m)].x = -37 + (1.5 * n); //COPY COORDINATES
			m_oldx[clothxy(n,m)].y = -37 + (1.5 * m);
			m_oldx[clothxy(n,m)].z = -50;
			m_a[clothxy(n,m)].x = 0; //ACCELERATION
			m_a[clothxy(n,m)].y = 0;
			m_a[clothxy(n,m)].z = 0;
		}
	}
	movingball.x = 0; //MOVING BALL  center of cloth
	movingball.y = 0;
	movingball.z = -60;
	m_vGravity.x = 0; //GRAVITY VECTOR
	m_vGravity.y = -gravity;
	m_vGravity.z = 0;
	wind.x = wx; //WIND VECTOR
	wind.y = wy;
	wind.z = wz;
	//SETUP ANCHORS
	anchorl = m_x[clothxy(0,49)]; //top-left
	anchorlr = m_x[clothxy(1,49)];
	anchorld = m_x[clothxy(0,48)];
	anchorr = m_x[clothxy(49,49)]; //top-right
	anchorrl = m_x[clothxy(48,49)];
	anchorrd = m_x[clothxy(49,48)];
	pause = FALSE;
	windhappens = 0; //determine when wind occurs
}

//  i    j    k
//v1.x v1.y v1.z
//v2.x v2.y v2.z
struct Vector3 crossproduct(struct Vector3 v1, struct Vector3 v2){
	struct Vector3 a;
	a.x = (v1.y * v2.z) - (v2.y * v1.z);
	a.y = -((v1.x * v2.z) - (v2.x * v1.z));
	a.z = (v1.x * v2.y) - (v2.x * v1.y);
	return a;
}

void addwindf(int xy, float windf){
	m_a[xy].x += windf;
	m_a[xy].y += windf;
	m_a[xy].z += windf;
}
//END FUNCTIONS
//*******************

void reshape (int w, int h)
{
	glViewport (0, 0, w, h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	if (h == 0)
	{
		gluPerspective (80, (float) w, 1.0, 5000.0);
	}
	else
	{
		gluPerspective (80, (float) w / (float) h, 1.0, 5000.0);
	}
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
}

void keyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:
			exit (0);
		break;
		case 32: //SPACE KEY
			pause = 1 - pause;
		break;
		case ('q'): //CHANGE DAMPENING
			DAMPENING -= .01;
			printf("DAMPENING: %f\n", DAMPENING);
		break;
		case ('w'):
			DAMPENING += .01;
			printf("DAMPENING: %f\n", DAMPENING);
		break;
		case ('a'): //CHANGE RESTL
			RESTL -= .1;
			printf("RESTL: %f\n", RESTL);
		break;
		case ('s'):
			RESTL += .1;
			printf("RESTL: %f\n", RESTL);
		break;
		case ('z'): //CHANGE DRESTL
			DRESTL -= .1;
			printf("DRESTL: %f\n", DRESTL);
		break;
		case ('x'):
			DRESTL += .1;
			printf("DRESTL: %f\n", DRESTL);
		break;
		case ('c'): //CHANGE ITERATIONS
			if(NUM_ITERATIONS > 1){
				NUM_ITERATIONS--;
			}
			printf("NUM_ITERATIONS: %d\n", NUM_ITERATIONS);
		break;
		case ('v'):
			NUM_ITERATIONS++;
			printf("NUM_ITERATIONS: %d\n", NUM_ITERATIONS);
		break;
		case ('d'): //GRAVITY
			gravity -= .1;
			printf("gravity: %f\n", m_vGravity.y);
		break;
		case ('f'):
			gravity += .1;
			printf("gravity: %f\n", m_vGravity.y);
		break;
		case ('t'): //WIND X
			wx -= .1;
			printf("wind x: %f\n", wx);
		break;
		case ('y'):
			wx += .1;
			printf("wind x: %f\n", wx);
		break;
		case ('g'): //WIND Y
			wy -= .1;
			printf("wind y: %f\n", wy);
		break;
		case ('h'):
			wy += .1;
			printf("wind y: %f\n", wy);
		break;
		case ('b'): //WIND Z
			wz -= .1;
			printf("wind z: %f\n", wz);
		break;
		case ('n'):
			wz += .1;
			printf("wind z: %f\n", wz);
		break;
		case ('u'): //CHANGE BALL COLLISION ITERATIONS
			//NUM_BALL_ITERATIONS--;
			if(NUM_BALL_ITERATIONS > 0){
				NUM_BALL_ITERATIONS--;
			}
			if(NUM_BALL_ITERATIONS == 0){
				printf("NUM_BALL_ITERATIONS: %d  collision disabled\n", NUM_BALL_ITERATIONS);
			}
			else{
				printf("NUM_BALL_ITERATIONS: %d  collision enabled\n", NUM_BALL_ITERATIONS);
			}
		break;
		case ('i'):
			NUM_BALL_ITERATIONS++;
			printf("NUM_BALL_ITERATIONS: %d  collision enabled\n", NUM_BALL_ITERATIONS);
		break;
		case ('r'): //reset
			resetsim();
		break;
		default:
		break;
	}
}

void arrow_keys (int a_keys, int x, int y)
{
	switch(a_keys)
	{
		case GLUT_KEY_UP:
			glutFullScreen();
		break;
		case GLUT_KEY_DOWN:
			glutReshapeWindow (960, 540 );
		break;
		default:
		break;
	}
}

int main (int argc, char *argv[])
{
	int n, m;
	int constrcounter = 0; //CONSTRAINT COUNTER
	oldtime = glutGet(GLUT_ELAPSED_TIME);
	oldtime = 0;
	/*for(n = 0; n<NODEAMOUNT; n++) //0-49  SETUP NODES
	{
		for(m = 0; m<NODEAMOUNT; m++)
		{
			m_x[clothxy(n,m)].x = -37 + (1.5 * n); //COORDINATES
			m_x[clothxy(n,m)].y = -37 + (1.5 * m);
			m_x[clothxy(n,m)].z = -50;
			m_oldx[clothxy(n,m)].x = -37 + (1.5 * n); //COPY COORDINATES
			m_oldx[clothxy(n,m)].y = -37 + (1.5 * m);
			m_oldx[clothxy(n,m)].z = -50;
			m_a[clothxy(n,m)].x = 0; //ACCELERATION
			m_a[clothxy(n,m)].y = 0;
			m_a[clothxy(n,m)].z = 0;
		}
	}
	movingball.x = 0; //MOVING BALL  center of cloth
	movingball.y = 0;
	movingball.z = -60;
	m_vGravity.x = 0; //GRAVITY VECTOR
	m_vGravity.y = -gravity;
	m_vGravity.z = 0;
	wind.x = wx; //WIND VECTOR
	wind.y = wy;
	wind.z = wz;
	//SETUP ANCHORS
	anchorl = m_x[clothxy(0,49)]; //top-left
	anchorlr = m_x[clothxy(1,49)];
	anchorld = m_x[clothxy(0,48)];
	anchorr = m_x[clothxy(49,49)]; //top-right
	anchorrl = m_x[clothxy(48,49)];
	anchorrd = m_x[clothxy(49,48)];*/
	resetsim();
	//SET CONSTRAINTS
	for(n = 0; n<NODEAMOUNT; n++){ //0-49      n=X
		for(m = 0; m<NODEAMOUNT-1; m++){ //m=Y  0-48
			SetConstraint(clothxy(n,m+1), clothxy(n,m), RESTL, &constrcounter); //UP
			SetConstraint(clothxy(m+1,n), clothxy(m,n), RESTL, &constrcounter); //RIGHT
		}
	}
	printf("constraint counter: %d\n", constrcounter);
	//DIAGONALS
	for(n = 0; n<NODEAMOUNT-1; n++){ //0-48    n=X
		for(m = 0; m<NODEAMOUNT-1; m++){ //m=Y  0-48
			SetConstraint(clothxy(n+1,m+1), clothxy(n,m), DRESTL, &constrcounter); //UP-RIGHT
			SetConstraint(clothxy(n+1,m), clothxy(n,m+1), DRESTL, &constrcounter); //DOWN-RIGHT
		}
	}
	//SET LONG CONSTRAINTS
	for(n = 0; n<NODEAMOUNT-1; n++){ //0-48    n=X
		for(m = 0; m<NODEAMOUNT-2; m++){ //m=Y  0-47
			SetConstraint(clothxy(n,m+2), clothxy(n,m), 2*RESTL, &constrcounter); //UP
			SetConstraint(clothxy(m+2,n), clothxy(m,n), 2*RESTL, &constrcounter); //RIGHT
		}
	}
	printf("constraint counter: %d\n", constrcounter);
	//DIAGONALS
	for(n = 0; n<NODEAMOUNT-2; n++){ //0-48    n=X
		for(m = 0; m<NODEAMOUNT-2; m++){ //m=Y  0-48
			SetConstraint(clothxy(n+2,m+2), clothxy(n,m), 2*DRESTL, &constrcounter); //UP-RIGHT
			SetConstraint(clothxy(n+2,m), clothxy(n,m+2), 2*DRESTL, &constrcounter); //DOWN-RIGHT
		}
	}
	NUM_ITERATIONS = 13;
	NUM_BALL_ITERATIONS = 5;
	NUM_CONSTRAINTS = constrcounter;
	printf("constraint counter: %d\n", constrcounter);
	printf("DAMPENING: %f\n", DAMPENING);
	printf("RESTL: %f\n", RESTL);
	printf("DRESTL: %f\n", DRESTL);
	printf("NUM_ITERATIONS: %d\n", NUM_ITERATIONS);
	printf("NUM_BALL_ITERATIONS: %d\n", NUM_BALL_ITERATIONS);
	printf("gravity: %f\n", gravity);

	ball.x = -110;
	ball.x = -80;
	ball.y = -4;
	ball.z = 0;
	glutInit (&argc, argv);
	glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize (960, 540 );
	//glutInitWindowSize (1600, 900 );
	//glutInitWindowSize (1280, 720 );
	glutCreateWindow ("cloth simulator");
	init ();
	glutDisplayFunc (display);
	glutReshapeFunc (reshape);
	glutKeyboardFunc (keyboard);
	glutSpecialFunc (arrow_keys);
	glutMainLoop ();
}
