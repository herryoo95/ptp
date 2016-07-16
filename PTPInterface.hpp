/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the PTP(Plastic Terramechanics Particle).
 *
 *  Author: Dr. -Ing. Yong-Ho Yoo 
 *
 */

#ifndef __PTPINTERFACE_H__
#define __PTPINTERFACE_H__
#include "PTPCore.hpp"
#include <Eigen/Core>
#include <mars/interfaces/sim/NodeInterface.h>

namespace mars {
  namespace sim {
		
typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector;		
typedef Eigen::Matrix<int, 3, 1, Eigen::DontAlign> VectorN;	

typedef struct vFOOT{
	Vector loc;					///[mm*MAG]
	Vector vel;					///[mm*MAG/s]
			
	int type;					///SPHERE
	double r;					///[mm*MAG]
	bool contact;
} vFOOT;

class PTPInterface : public PTPCore{
public:
	
	PTPInterface(int x, int y, int z);
    ~PTPInterface();
    
	bool buildPTP(const VectorN *num,const Vector *start,const Vector *end,interfaces::NodeInterface* nodeInterface);
    bool collidePTP(bool isMaxForceNow);  
    
vFOOT vfoot;
Vector contactForce;
//FOOT footLast;
//Vector forceR;
//Vector last_outF;
//Vector last_pos;
double minPosZ;  
    
private:

    
};

  } // end of namespace sim
} // end of namespace mars

#endif // __PTPINTERFACE_H__
