
/**
 * \file PtpGraphics.cpp
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief soil
 *
 * Version 0.1
 */


#include "PTPGraphics.hpp"
#include <mars/utils/Vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <time.h>


namespace mars {
  namespace plugins {

      using namespace utils;
      using namespace interfaces;
      //using namespace sim;     

//PTPGraphics::PTPGraphics() {
	//prev_drawing = false;
//}
PTPGraphics::PTPGraphics(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "PTPGraphics") {
	prev_drawing = false;			
}

PTPGraphics::~PTPGraphics() {
}
  
void PTPGraphics::drawSoil(ControlCenter *control, sim::PTPInterface* soil){

	 if(prev_drawing) control->graphics->removeOSGNode(geode_pt.get()); 
		  
     osg::ref_ptr<osg::Geode> geode (new osg::Geode());
     geode_pt = geode;
     
     osg::ref_ptr<osg::Geometry> particleGeom (new osg::Geometry());
     osg::ref_ptr<osg::Vec3Array> particleVertices (new osg::Vec3Array());
     osg::ref_ptr<osg::Vec4Array> particleColors (new osg::Vec4Array());
     
     Vector ptp_pos;
  
       for(int i = 0;i<soil->ptp.n.x;i++){
		for(int j = 0;j<soil->ptp.n.y;j++){
		  for(int k = 0;k<soil->ptp.n.z;k++){
			  
       	////Only Collision Particle
		//if(soil->particle[i][j][k].isCollision){
		  //ptp_pos.x() = soil->particle[i][j][k].loc.x;
		  //ptp_pos.y() = soil->particle[i][j][k].loc.y;
		  //ptp_pos.z() = soil->particle[i][j][k].loc.z; 			
	   
	   //vertices->push_back (osg::Vec3 (ptp_pos.x(), ptp_pos.y(), ptp_pos.z()));
       //colors->push_back (osg::Vec4f (1.0f,0.0f, 0.0f,1.0f));
		//}else{			  
		  ptp_pos.x() = soil->particle[i][j][k].loc.x*MM2M;
		  ptp_pos.y() = soil->particle[i][j][k].loc.y*MM2M;
		  ptp_pos.z() = soil->particle[i][j][k].loc.z*MM2M; 
  
       particleVertices->push_back (osg::Vec3 (ptp_pos.x(), ptp_pos.y(), ptp_pos.z()));
       particleColors->push_back (osg::Vec4f (1.0f,1.0f, 1.0f,1.0f));
		//}
	     if(soil->cell[i][j][k].CollisionParticleNum){
			Vector cellPos,cellSize; 
			cellPos.x() = soil->cell[i][j][k].start.x*MM2M;
			cellPos.y() = soil->cell[i][j][k].start.y*MM2M;
			cellPos.z() = soil->cell[i][j][k].start.z*MM2M;
			 
			cellSize.x() = (soil->cell[i][j][k].start.x-soil->cell[i][j][k].end.x)*MM2M/MAG;
			cellSize.y() = (soil->cell[i][j][k].start.y-soil->cell[i][j][k].end.y)*MM2M/MAG;
			cellSize.z() = (soil->cell[i][j][k].start.z-soil->cell[i][j][k].end.z)*MM2M/MAG;

          //  drawCell(&cellPos, &cellSize);
		}
       
		  }
		}
	  }
	  

	  
    particleGeom->setVertexArray(particleVertices.get());
    particleGeom->setColorArray(particleColors.get());

	particleGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    particleGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,particleVertices->size()));
    geode_pt->addDrawable(particleGeom.get());


    control->graphics->addOSGNode(geode.get());  
    prev_drawing = true;

}


  } // end of namespace sim
} // end of namespace mars

DESTROY_LIB(mars::plugins::PTPGraphics::PTPGraphics);
CREATE_LIB(mars::plugins::PTPGraphics::PTPGraphics);
