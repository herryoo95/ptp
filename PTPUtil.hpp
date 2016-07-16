/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the PTP(Plastic Terramechanics Particle).
 *
 *  Author: Dr. -Ing. Yong-Ho Yoo 
 *
 */

#ifndef __PTPUTIL_H__
#define __PTPUTIL_H__
#include <math.h>
#include "PTPparameters.hpp"
#include <stdio.h>

#define FLOAT_EQ(x,v) (((v - EPSILON_1) < x) && (x <( v + EPSILON_1)))
//#define round_x(x) ((x)>=0?(int)((x)+0.5-EPSILON):(int)((x)-0.5+EPSILON))

#define ALL 0
#define XX_AXIS 1
#define YY_AXIS 2
#define ZZ_AXIS 4

#define SQR(n) (n)*(n)
#define SQR2(X_AXIS,Y_AXIS) ((X_AXIS)*(X_AXIS))+((Y_AXIS)*(Y_AXIS))+((2*(X_AXIS)*(Y_AXIS)))
#define MAX(A, B)	((A) > (B) ? (A) : (B))
#define ABS(x) ((x) < 0 ? (-(x)) : (x))
typedef enum {False=0, True=1} Bool;

namespace mars {
  namespace sim {

typedef struct {
double x,y,z;
} vecD;

typedef struct {
int x,y,z;
} vecN;

class PTPUtil{
	
public:
	
	PTPUtil();
    ~PTPUtil();
    
	vecD setVecD(const double x,const double y,const double z);
	vecN setVecN(const int x,const int y,const int z);
	double getCosTheta(const vecD* a,const vecD* b);
	vecD minusVecD(const vecD* i1,const vecD* i2);
	vecD plusVecD(const vecD* i1,const vecD* i2);
	vecN minusVecN(const vecN* i1,const vecN* i2);
	vecN plusVecN(const vecN* i1,const vecN* i2);
	vecD multipleVecD(const vecD* i1,const vecD* i2);
	vecD getMaxVec(const vecD* i1,const vecD* i2);
	vecD getMinVec(const vecD* i1,const vecD* i2);
	vecN getMaxVecN(const vecN* i1,const vecN* i2);
	vecN getMinVecN(const vecN* i1,const vecN* i2);
	vecD getMiddlePoint(const vecD* p1, const vecD* p2);
	double getVecDSizeSQR(const vecD* i, const int type);

	double getLengthSQR(const vecD* p1, const vecD* p2, const int type);

	double InnerProduct(const vecD* p,const vecD* q);
	vecD CrossProduct(const vecD* p,const vecD* q);


///EXPENSIVE

	double sqrtVecDP(const vecD *i);
	double quad_eqn(const double a, const double b, const double c);
	int quad_eqn_new(const double a, const double b, const double c, double *t0, double *t1);
	vecD projVector(const vecD* P, const vecD* n);
	vecD transformSpherical(const vecD* point);
	vecD transformRectangular(const vecD* point);
	vecD normalizeVector(const vecD* v);
	char *intToBinary(int i);          
	char *uintToBinary(unsigned int i); 
	int binaryToInt(char *s);           
	unsigned int binaryToUint(char *s); 

private:
	
};

  } // end of namespace sim
} // end of namespace mars

#endif // __PTPUTIL_H__
