/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the PTP(Plastic Terramechanics Particle).
 *
 *  Author: Dr. -Ing. Yong-Ho Yoo 
 *
 */

#include "PTPInterface.hpp"


namespace mars {
  namespace sim {

PTPInterface::PTPInterface(int x, int y, int z) : PTPCore(x,y,z){

}
PTPInterface::~PTPInterface()
{

}

bool PTPInterface::buildPTP(const VectorN *num,const Vector *start,const Vector *end, interfaces::NodeInterface* nodeInterface){
	vecN n;
	vecD s,e;
	n.x = num->x();
	n.y = num->y();
	n.z = num->z();
	s.x = start->x();
	s.y = start->y();
	s.z = start->z();
	e.x = end->x();
	e.y = end->y();
	e.z = end->z();	
	
          	//printf("(%f %f %f)...##(%d %d)\n", 
          	    //MM2M*start->x(),MM2M*start->y(), nodeInterface->getHeightmapHeight(128+(int)(MM2M*start->x()*256/26), 
          	    //128+(int)(MM2M*start->y()*256/26)), 128-76+(int)(MM2M*start->x()*256/26), 128+(int)(MM2M*start->y()*256/26));
          	    
          	//printf("(%f %f %f)...##(%d %d)\n", 
          	    //MM2M*end->x(),MM2M*end->y(), nodeInterface->getHeightmapHeight(128-76+(int)(MM2M*end->x()*256/26), 
          	    //128+(int)(MM2M*end->y()*256/26)), 128-76+(int)(MM2M*end->x()*256/26), 128+(int)(MM2M*end->y()*256/26));          	    
          	    
			double x1,x2,x3,y1,y2,y3,z1,z2,z3,x,y,z,z4;
			x1 = start->x()*MM2M;
			y1 = start->y()*MM2M;
			z1 = nodeInterface->getHeightmapHeight(128-76-(int)(start->x()*MM2M*256/26),128+(int)(start->y()*MM2M*256/26));			
			x2 = end->x()*MM2M;
			y2 = end->y()*MM2M;	
			z2 = nodeInterface->getHeightmapHeight(128-76-(int)(end->x()*MM2M*256/26),128+(int)(end->y()*MM2M*256/26));						
			x3 = end->x()*MM2M;
			y3 = start->y()*MM2M;
			z3 = nodeInterface->getHeightmapHeight(128-76-(int)(end->x()*MM2M*256/26),128+(int)(start->y()*MM2M*256/26));			
							
				ptp.plane_a = y1*(z2-z3)+y2*(z3-z1)+y3*(z1-z2);
				ptp.plane_b = z1*(x2-x3)+z2*(x3-x1)+z3*(x1-x2);
				ptp.plane_c = x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2);
				ptp.plane_d = -x1*(y2*z3-y3*z2)-x2*(y3*z1-y1*z3)-x3*(y1*z2-y2*z1);

			//x = x1;
			//y = y1;	
			//z = -(ptp.plane_a*x+ptp.plane_b*y+ptp.plane_d)/ptp.plane_c; 	
			
			//printf("....(%f %f %f)..(%f)\n", x1,y1,z1,z);
			
			x = start->x()*MM2M;
			y = end->y()*MM2M;
			z4 = nodeInterface->getHeightmapHeight(128-76-(int)(start->x()*MM2M*256/26),128+(int)(end->y()*MM2M*256/26));					
			z = -(ptp.plane_a*x+ptp.plane_b*y+ptp.plane_d)/ptp.plane_c; 	
			
			//printf("....(%f %f %f)..(%f)\n", start->x()*MM2M,end->y()*MM2M,z4,z);
			
			
				 
	return 	initializePTP(&n,&s,&e);	
}	

bool PTPInterface::collidePTP(bool isMaxForceNow){
	
	static FOOT foot, foot_last;
	//static int last_flag=0;
	static vecD f;
	
	//foot <- vfoot
	foot.loc.x = vfoot.loc.x();
	foot.loc.y = vfoot.loc.y();	
	foot.loc.z = vfoot.loc.z();
	foot.vel.x = vfoot.vel.x();
	foot.vel.y = vfoot.vel.y();	
	foot.vel.z = vfoot.vel.z();
		
	foot.type = vfoot.type;
	foot.r 	  = vfoot.r;	
	
	//ptpWorks(&foot,&foot_last);
//	if(!isMaxForceNow) {
		ptpWorks(&foot,&foot_last);	
//	}
//	else printf(" max force now................\n");
		
	calcForce(&foot, FAST,&f);	//have a look of FINE
	
	vfoot.contact = foot.contact;		
		
	foot_last = foot;
		//last_flag=1;
	contactForce.x() = f.x;
	contactForce.y() = f.y;
	contactForce.z() = f.z;
	//		printf("(%f %f %f) in foot vel \n", foot.vel.x,foot.vel.y,foot.vel.z);	
	//		printf("(%f %f %f) in contacforce \n", f.x,f.y,f.z);
		
	return true;//vfoot.contact;

}
 
  } // end of namespace sim
} // end of namespace mars
