/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the PTP(Plastic Terramechanics Particle).
 *
 *  Author: Dr. -Ing. Yong-Ho Yoo 
 *
 */
 
#include "PTPUtil.hpp"


namespace mars {
  namespace sim {

PTPUtil::PTPUtil(){
}
PTPUtil::~PTPUtil(){

}
vecD PTPUtil::setVecD(const double x,const double y,const double z){
	vecD r;

	r.x = x;
	r.y = y;
	r.z = z;
	return r;
}
vecN PTPUtil::setVecN(const int x,const int y,const int z){
	vecN r;

	r.x = x;
	r.y = y;
	r.z = z;
	return r;
}
vecD PTPUtil::minusVecD(const vecD* i1,const vecD* i2){
	return setVecD(i1->x-i2->x, i1->y-i2->y, i1->z-i2->z);
}
vecD PTPUtil::plusVecD(const vecD* i1,const vecD* i2){
	return setVecD(i1->x+i2->x, i1->y+i2->y, i1->z+i2->z);
}
vecN PTPUtil::minusVecN(const vecN* i1,const vecN* i2){
	return setVecN(i1->x-i2->x, i1->y-i2->y, i1->z-i2->z);
}
vecN PTPUtil::plusVecN(const vecN* i1,const vecN* i2){
	return setVecN(i1->x+i2->x, i1->y+i2->y, i1->z+i2->z);
}
vecD PTPUtil::multipleVecD(const vecD* i1,const vecD* i2){
	return setVecD(i1->x*i2->x, i1->y*i2->y, i1->z*i2->z);
}
vecD PTPUtil::getMaxVec(const vecD* i1,const vecD* i2){
	vecD max;
	if(i1->x < i2->x){
		max.x = i2->x;
	}else{
		max.x = i1->x;
	}

	if(i1->y < i2->y){
		max.y = i2->y;
	}else{
		max.y = i1->y;
	}

	if(i1->z < i2->z){
		max.z = i2->z;
	}else{
		max.z = i1->z;
	}
	return max;
}
vecD PTPUtil::getMinVec(const vecD* i1,const vecD* i2){
	vecD min;
	if(i1->x > i2->x){
		min.x = i2->x;
	}else{
		min.x = i1->x;
	}

	if(i1->y > i2->y){
		min.y = i2->y;
	}else{
		min.y = i1->y;
	}

	if(i1->z > i2->z){
		min.z = i2->z;
	}else{
		min.z = i1->z;
	}
	return min;
}
vecN PTPUtil::getMaxVecN(const vecN* i1,const vecN* i2){
	vecN max;
	if(i1->x < i2->x){
		max.x = i2->x;
	}else{
		max.x = i1->x;
	}

	if(i1->y < i2->y){
		max.y = i2->y;
	}else{
		max.y = i1->y;
	}

	if(i1->z < i2->z){
		max.z = i2->z;
	}else{
		max.z = i1->z;
	}
	return max;
}
vecN PTPUtil::getMinVecN(const vecN* i1,const vecN* i2){
	vecN min;
	if(i1->x > i2->x){
		min.x = i2->x;
	}else{
		min.x = i1->x;
	}

	if(i1->y > i2->y){
		min.y = i2->y;
	}else{
		min.y = i1->y;
	}

	if(i1->z > i2->z){
		min.z = i2->z;
	}else{
		min.z = i1->z;
	}
	return min;
}
vecD PTPUtil::getMiddlePoint(const vecD* p1, const vecD* p2){
	vecD temp;
	temp.x = (p1->x+p2->x)*0.5f;
	temp.y = (p1->y+p2->y)*0.5f;
	temp.z = (p1->z+p2->z)*0.5f;

	return temp;
}

double PTPUtil::getVecDSizeSQR(const vecD* i, const int type){
	double r;
	switch(type){
	case ZZ_AXIS:
		r = SQR(i->x)+SQR(i->y);
		break;
	case YY_AXIS:
		r = SQR(i->x)+SQR(i->z);
		break;
	case XX_AXIS:
		r = SQR(i->y)+SQR(i->z);
		break;
	case ALL:
		r = SQR(i->x)+SQR(i->y)+SQR(i->z);
		break;
	}
	return r;
}

double PTPUtil::getLengthSQR(const vecD* p1, const vecD* p2, const int type){
	vecD temp;

	temp.x = p1->x - p2->x;
	temp.y = p1->y - p2->y;
	temp.z = p1->z - p2->z;

	return getVecDSizeSQR(&temp,type);
}
double PTPUtil::getCosTheta(const vecD* a,const vecD* b){
	double r;
	
	r = ((a->x*b->x)+(a->y*b->y)+(a->z*b->z))/(sqrtVecDP(a)*sqrtVecDP(b));
		
	return r;
}

double PTPUtil::InnerProduct(const vecD* p,const vecD* q){
	return (p->x*q->x)+(p->y*q->y)+(p->z*q->z);
}
vecD PTPUtil::CrossProduct(const vecD* p,const vecD* q){
	vecD r;

	r.x = ((p->y*q->z) - (p->z*q->y));
	r.y = -((p->x*q->z) - (p->z*q->x));
	r.z = ((p->x*q->y) - (p->y*q->x));

	return r;
}


///EXPENSIVE

double PTPUtil::sqrtVecDP(const vecD *i){
	return sqrt((i->x*i->x)+(i->y*i->y)+(i->z*i->z));
}
vecD PTPUtil::normalizeVector(const vecD* v){
	vecD r=setVecD(0,0,0);
	double size =sqrtVecDP(v); 
	if(size){
		r.x = v->x/size;
		r.y = v->y/size;
		r.z = v->z/size;
	}
	return r;
}
 double PTPUtil::quad_eqn(const double a, const double b, const double c){
	double d, x, y;

	d = b*b-4.0*a*c;
	if(d>0)
	{
		x = (-b-sqrt(b*b-4*a*c))/2*a;
		y = (-b+sqrt(b*b-4*a*c))/2*a;
		return y;
	}else if(FLOAT_EQ(d,0.f)){
		x = b/(-2.0*a);
		return x;
	}else{
		return 0;
	}
}

 int PTPUtil::quad_eqn_new(const double a, const double b, const double c, double *t0, double *t1){
	double d;

	d = b*b-4*a*c;
	if(d>0)
	{
		*t0 = (-b-sqrt(b*b-4*a*c))/2*a;
		*t1 = (-b+sqrt(b*b-4*a*c))/2*a;
		return 2;
	}else if(FLOAT_EQ(d,0.f)){
		*t0 = b/(-2.0*a);
		*t1 = b/(-2.0*a);
		return 1;
	}else{
		*t0 = -1.0;
		*t1 = -1.0;
		return 0;
	}
}


 vecD PTPUtil::projVector(const vecD* P, const vecD* n){
	vecD result;
	double inner = InnerProduct(P,n);
	result = *P;
	result.x -= inner*n->x;
	result.y *= inner*n->y;
	result.z *= inner*n->z;

	return result;
}


 vecD PTPUtil::transformSpherical(const vecD* point){
	double r = sqrtVecDP(point);
	double phi=0.0;
	double theta=0.0;

	theta = acos(point->z/sqrt(point->x*point->x+point->y*point->y+point->z*point->z));  

	if (point->y > 0) {
		if (point->x > 0) {phi = atan(point->y/point->x); }
		if (point->x < 0) {phi = atan(point->y/point->x) + 180*DEG2RAD; } 
		if (point->x == 0) { phi = 90.0*DEG2RAD; }}
	if (point->y < 0) {
		if (point->x > 0) {phi = atan(point->y/point->x) + 360*DEG2RAD; }
		if (point->x < 0) {phi = atan(point->y/point->x) + 180*DEG2RAD; } 
		if (point->x == 0) { phi = 270.0*DEG2RAD; }}
	if (point->y == 0) {
		if (point->x > 0) {phi = 0; }
		if (point->x < 0) {phi = 180.0*DEG2RAD; }}


	return setVecD(r,phi,theta);
}

 vecD PTPUtil::transformRectangular(const vecD* point){
	double r = point->x;
	double phi = point->y;
	double th = point->z;

	double x = r*sin(th)*cos(phi);
	double y = r*sin(th)*sin(phi);
	double z = r*cos(th);
	/*
	double x = point->x*sin(point->y)*sin(point->z+PI/2);
	double y = point->x*cos(point->z +PI/2);
	double z = point->x*cos(point->y)*sin(point->z+PI/2);
	*/
	return setVecD(x,y,z);
}

char* PTPUtil::intToBinary(int i) {
	static char s[32 + 1] = { '0', };
	int count = 32;

	do { s[--count] = '0' + (char) (i & 1);
	i = i >> 1;
	} while (count);

	return s;
}


char* PTPUtil::uintToBinary(unsigned int i) {
	static char s[32 + 1] = { '0', };
	int count = 32;

	do { s[--count] = '0' + (char) (i & 1);
	i = i >> 1;
	} while (count);

	return s;
}



int PTPUtil::binaryToInt(char *s) {
	int i = 0;
	int count = 0;

	while (s[count])
		i = (i << 1) | (s[count++] - '0');

	return i;
}
/*
Fixed intToFixed(int iValue){
return (Fixed)(iValue << 16);
}
Fixed doubleToFixed(double dValue){
return (Fixed)(dValue * 65536.0);
}

int FixedToInt(Fixed fValue){
return fValue >> 16;
}

double FixedToDouble(Fixed fValue){
return ((double)fix) / 65536.0;
}
*/

unsigned int PTPUtil::binaryToUint(char *s) {
	unsigned int i = 0;
	int count = 0;

	while (s[count])
		i = (i << 1) | (s[count++] - '0');

	return i;
}

  } // end of namespace sim
} // end of namespace mars
