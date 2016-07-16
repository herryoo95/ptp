/**
 * \file PTPGraphics.h
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief soil
 *
 * Version 0.1
 */

#ifndef MARS_PTPGRAPHICS_H
#define MARS_PTPGRAPHICS_H

#ifdef _PRINT_HEADER_
  #warning "PTPGraphics.hpp"
#endif

#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/utils/Vector.h>
#include <string>
#include "PTPInterface.hpp"
#include <time.h>

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile> 
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

namespace mars {
  namespace plugins {
	  
	  //using namespace mars::utils;

       class PTPGraphics: public mars::interfaces::MarsPluginTemplate {

      public:
        PTPGraphics(lib_manager::LibManager *theManager);
        ~PTPGraphics();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("soilNode"); }
        CREATE_MODULE_INFO();        

//		void drawCell(Vector* cellPos, Vector* cellSize);
		void drawSoil(interfaces::ControlCenter *control, sim::PTPInterface* soil);

        
		osg::ref_ptr<osg::Geode> geode_pt;

		double msec;
		bool prev_drawing;
		
		private:
	};


  } // end of namespace sim
} // end of namespace mars

#endif // MARS_PTPGRAPHICS_H
