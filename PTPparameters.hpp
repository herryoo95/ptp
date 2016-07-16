/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the PTP(Plastic Terramechanics Particle).
 *
 *  Author: Dr. -Ing. Yong-Ho Yoo 
 *
 */

#ifndef __PTPPARAMETERS_H__
#define __PTPPARAMETERS_H__

///Essentional Option(FIXED)
#define FAST_CALC
#define USING_FR		//!
//#define USE_MOVEMENT_FUNCTION
//#define FILEPRINT		 

///Movement Function Option
//#define USE_MOVEMENT_TOP
#define USE_GRAVITY
///Failed Option
#ifndef USING_FR		
#endif

#define PI 3.1415926536
#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)

#define SPREAD 10
#define DEL_SOIL_THRESHOLD -0.04  // at this height (meter)PTP should be deleted
#define FRICTION 0
#define GRAVITY -0.1 //-9.80665			 
#define EPSILON_1 0.0001   // Define your own tolerance
#define TOTALMASS 25
#define FOOTRADIUS 25

#define COHESION 1100   //[Pa]   granular:600-900, fine: 1000-1195
#define INT_ANGLE 40*DEG2RAD  
#define DEFORMATION_MODULE 0.05
#define SHEAR_THRESHOLD 0.1f  //if more than this lateral force (forceXY), it is started to calculate the shearForce

#define MAG 1.0
#define MAX_X 600					/// X�� �ִ� �� ����
#define MAX_Y 600				/// Y�� �ִ� �� ����
#define MAX_Z 200					/// Z�� �ִ� �� ����

///FILER
#define FILTER_Q 0.022
#define FILTER_R 0.617

///Laplace Equation Parameters
#define PANEL_Y	(-8e-6)				///Gravity
#define PANEL_ZX (3e-3)				///

///Soil Visualization Parameters
#define SOIL_PANEL_ZX 8e-4			///�ҵ�¡ ����Ʈ �Ķ����(����κ����� �����ϴ���) �ӵ��� ����������
#define SOIL_DEGREE	1.5				///�ҵ�¡ ����Ʈ �Ķ����(��ŭ �����ϴ���..) XZ �� �������
#define SOIL_DEGREE_Y	0.8

///Lookup Table Parameters
#define LOOKUP_K	895980
#define LOOKUP_N	0.785

#endif
