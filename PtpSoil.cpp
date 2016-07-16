/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * \file PtpSoil.cpp
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief soil
 *
 * Version 0.1
 */


#include "PtpSoil.hpp"
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/utils/Vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <time.h>


namespace mars {
  namespace plugins {
    namespace ptpSoil {

      using namespace mars::utils;
      using namespace mars::interfaces;

PtpSoil::PtpSoil(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "PtpSoil") {

		readyNextPlane = true;
		nextPlane = false;  
}
 
void PtpSoil::drawCell(Vector* cellPos, Vector* cellSize){
	osg::ref_ptr<osg::Geometry> cellGeom (new osg::Geometry());
 	osg::ref_ptr<osg::Vec3Array> cellVertices (new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array> cellColors (new osg::Vec4Array());
    
    cellVertices->push_back( osg::Vec3(cellPos->x(), cellPos->y(), cellPos->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x()+cellSize->x(), cellPos->y(), cellPos->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x()+cellSize->x(), cellPos->y()+cellSize->y(), cellPos->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x(), cellPos->y()+cellSize->y(), cellPos->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x(), cellPos->y(), cellPos->z()+cellSize->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x()+cellSize->x(), cellPos->y(), cellPos->z()+cellSize->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x()+cellSize->x(), cellPos->y()+cellSize->y(), cellPos->z()+cellSize->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x(), cellPos->y()+cellSize->y(), cellPos->z()+cellSize->z()));
    
	//cellVertices->push_back( osg::Vec3(0, 0, 0));
	//cellVertices->push_back( osg::Vec3(1, 0, 0));
	//cellVertices->push_back( osg::Vec3(1, 1, 0));
	//cellVertices->push_back( osg::Vec3(0,1,0));
	//cellVertices->push_back( osg::Vec3(0, 0, 1));
	//cellVertices->push_back( osg::Vec3(1, 0, 1));
	//cellVertices->push_back( osg::Vec3(1, 1, 1));
	//cellVertices->push_back( osg::Vec3(0,1,1));
	
	cellGeom->setVertexArray(cellVertices.get());

	osg::ref_ptr<osg::DrawElementsUInt> cellBottom (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellBottom->push_back(3);
	cellBottom->push_back(2);
	cellBottom->push_back(1);
	cellBottom->push_back(0);
	cellGeom->addPrimitiveSet(cellBottom.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellFront (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellFront->push_back(4);
	cellFront->push_back(5);
	cellFront->push_back(1);
	cellFront->push_back(0);
	cellGeom->addPrimitiveSet(cellFront.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellLeft (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));	
	cellLeft->push_back(6);
	cellLeft->push_back(2);
	cellLeft->push_back(1);
	cellLeft->push_back(5);
	cellGeom->addPrimitiveSet(cellLeft.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellRight (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellRight->push_back(4);
	cellRight->push_back(7);
	cellRight->push_back(3);
	cellRight->push_back(0);
	cellGeom->addPrimitiveSet(cellRight.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellBack (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellBack->push_back(7);
	cellBack->push_back(6);
	cellBack->push_back(2);
	cellBack->push_back(3);
	cellGeom->addPrimitiveSet(cellBack.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellTop (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellTop->push_back(7);
	cellTop->push_back(6);
	cellTop->push_back(5);
	cellTop->push_back(4);
	cellGeom->addPrimitiveSet(cellTop.get());
	
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    
    cellGeom->setColorArray(cellColors.get());
    cellGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    //cellGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geode_pt->addDrawable(cellGeom.get());
    
}

  
void PtpSoil::drawSoil(PTPInterface* soil){

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

}


PtpSoil::~PtpSoil() {
		for(int i=0;i<6;i++) delete  soil[i];
}
  
void PtpSoil::update(sReal time_ms) {
	
		const int num_foot[6]={34,48,61,73,87,99};
	    Vector foot_pos[6], foot_vel[6];

	  for(int i=0;i<6;i++){
		foot_pos[i] = control->nodes->getPosition(num_foot[i]);
		foot[i].loc.x() =foot_pos[i].x()*M2MM;
		foot[i].loc.y() =foot_pos[i].y()*M2MM;
		foot[i].loc.z() =foot_pos[i].z()*M2MM;	
		
		foot_vel[i] = control->nodes->getLinearVelocity(num_foot[i]);   //going down in z-axis is minus velocity
		foot[i].vel.x() =foot_vel[i].x()*M2MM;
		foot[i].vel.y() =foot_vel[i].y()*M2MM;
		foot[i].vel.z() =foot_vel[i].z()*M2MM;	
		
		//printf(" %d (%f %f %f)\n",num_foot[i], foot[i].loc.x,foot[i].loc.y,foot[i].loc.z);	
		last_pos[i] = foot_pos[i]; 
	}

     // 	if(readyNextPlane == true ) {
		record_time += time_ms;

		Vector outF[6];
		double wasFirstPlane = false;
	for(int i=0;i<6;i++) {
	if(foot[i].contact == true) {		
//	 if((foot_pos[i].x() > start0.x) && (foot_pos[i].x() < end0.x)) {
		 if(!soil[i]->getPTPForce(foot[i],&forceR[i])){printf("ERRRROR");}
		 else {
			 wasFirstPlane = true;
			// if(foot[i].contact == true) {		     
			 //	printf("foot collision\n");
			 outF[i].x() = forceR[i].x();	
			 outF[i].y() = forceR[i].y();	
				if(foot_pos[i].z() > -0.2) outF[i].z() = forceR[i].z();
				else {
				outF[i].z() = forceR[i].z() + (foot_pos[i].z()+0.2)*2200 + foot_vel[i].z()*100;	
//				printf(",,,,,%f  -- %f\n", foot_pos[i].z(), outF[i].z());
				}
				if(ABS(outF[i].z()) <= 150) control->nodes->applyForce(num_foot[i], -outF[i]);
				else if(outF[i].z() > 150 ) { 
				outF[i].z() = 150;
				control->nodes->applyForce(num_foot[i], -outF[i]);
				}
				else if(outF[i].z() < -150) { 
				outF[i].z() = -150;				
				control->nodes->applyForce(num_foot[i], -outF[i]);
				} 
			}
	//	 }
	 } 

	}  //for

	    disp_time += time_ms;
	    if(disp_time > 200.0f){
		  control->graphics->removeOSGNode(geode_pt.get()); 
		//drawSoil();
		  drawSoil(soil[0]);
		 // if(nextPlane == true) drawSoil(soil1);
		//	  printf(" (%f %f %f)\n",foot[2].loc.x,foot[2].loc.y,foot[2].loc.z);
		disp_time = 0;
		}
		
		
}

    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::ptpSoil::PtpSoil);
CREATE_LIB(mars::plugins::ptpSoil::PtpSoil);
