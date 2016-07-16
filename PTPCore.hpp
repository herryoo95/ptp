/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the PTP(Plastic Terramechanics Particle).
 *
 *  Author: Dr. -Ing. Yong-Ho Yoo 
 *
 */

#ifndef __PTPCORE_H__
#define __PTPCORE_H__
#include <stdio.h>
#include "PTPUtil.hpp"
#include <math.h>
#include <stdlib.h>


#define MM2M 0.001
#define M2MM 1000
#define SPHERE 0
//#define CYLINDER 1
//#define CUBE 2
#define NOT 0
//#define GOLINE 1024
//#define FEATURE_POINT 512
#define ERR -1
#define FINE 1
#define FAST 2
#define START 1
#define END 2
#define PTP_START 4
#define PTP_END 8

//#define MOST_UPPER 0
//#define MOST_CONTAIN 1

///SPHARE
#define IS_ON 1
#define IS_IN 256

///COLLISION
#define NOT_COLLISION	0
#define COLLISION		-1


namespace mars {
  namespace sim {
	
typedef struct FOOT{
	vecD loc;					///[mm*MAG]
	//	vecD acc;					///[mm*MAG/s^2]
	vecD vel;					///[mm*MAG/s]
	
//	vecD avel;					///[rad/s]
			
	int type;					///SPHERE
	double r;					///[mm*MAG]
//	double l;					///[mm*MAG]
//	double d;					///[mm*MAG]
//	double mass;
	bool contact;
} FOOT;

typedef struct PTP{
	vecD start;					///[mm*MAG]
	vecD end;					///[mm*MAG]
	vecN n;						///[]
	vecD cellScale;
	vecD test;
	vecD start_loc;				/// to calculate dip. of lateral	
	int lateral;
	double plane_a,plane_b,plane_c,plane_d;
} PTP;

typedef struct CELL {
	vecD start;					///[mm*MAG]
	vecD end;					///[mm*MAG]
	vecD center;
	vecD averageLoc;			/** average location of particles collided with object */
	unsigned int CollisionParticleNum;
    unsigned int particleNum;
} CELL;

typedef struct PARTICLE {
	vecD loc;					///[mm*MAG]
	vecN cloc;					///[]
	int isCollision;
} PARTICLE;

class PTPCore{

public:
		
	PTPCore(int x, int y, int z);
	~PTPCore();
	
	bool AABBCollision(CELL* pAABB1, CELL* pAABB2);

	bool calcForce(FOOT* foot, const int type, vecD* forceR);
	bool initializePTP(const vecN *n,const vecD *s,const vecD *e);	
	int particleWorks(const vecN* particleN,const FOOT* foot, const FOOT* lastFootLoc, const int pLocData);

	int ptpWorks(const FOOT* foot,const FOOT* lastFoot);
	vecD particleMovement(const vecD* point,const FOOT* foot);
	
	vecN getComputeNeededCellNum(const FOOT* foot, const int type);

	int getSumOfParticlesNum();
	int particleCollisionDetect(const FOOT* foot, const PARTICLE* particle);
	bool PTPCollisionDetectOne(const FOOT* foot);
	int cellCollisionDetectVertex(const FOOT* foot, const vecN* cellN);
	int cellCollisionDetectLine(const FOOT* foot, const vecN* cellN);
	double lookupTable(const CELL* cell);
	double shearStress(const double disp, const double pressure);  
	double getArea(const CELL* cell,const int direction);
	vecD getCellSize();

	double determinePlanePointLoc(const vecD n, const double d, const vecD p);

	vecD getFeaturePoint(const FOOT* foot, const vecD cellcenter);

	int isInASphere(const FOOT* foot, const vecD* point);

	vecD getOriginalParticlePosition(const vecN n);
	int getIntersectionNumLineSphere(const vecD* LineStart, const vecD* LineEnd, const vecD* SphereCenter, const double* SphereRadius);
	int cellCollisionFeaturePoint(const FOOT* foot, const vecN* cellN);

	vecD getFootAABBBound(const FOOT* foot, const int type);
	vecN judgeWhichCell(const vecD* loc);

	double getVolume(const CELL* cell);

	double getLength(const CELL* cell, const int type);

	PTP ptp; 

	PTPUtil* ptputil; 
	PARTICLE*** particle; 
	CELL*** cell; 
	char filenameCALC[150];
	
	vecN LOOPSTART;
	vecN LOOPEND;
	
	FILE *outCALC;
	vecN ptpsize;

private:


};	
	
  } // end of namespace sim
} // end of namespace mars

#endif // __PTPCORE_H__
