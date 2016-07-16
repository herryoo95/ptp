/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the PTP(Plastic Terramechanics Particle).
 *
 *  Author: Dr. -Ing. Yong-Ho Yoo 
 *
 */

#include "PTPCore.hpp"

namespace mars {
  namespace sim {

PTPCore::PTPCore(int x, int y, int z){
	ptputil = new PTPUtil();
	ptp.lateral = 0;
	
	ptpsize = ptputil->setVecN(x,y,z);
	particle = new PARTICLE**[x];
	for(int i=0; i<x; i++) {
	    particle[i] = new PARTICLE*[y];
		for(int j=0; j<y; j++) particle[i][j] = new PARTICLE[z];
	}
	cell = new CELL**[x];
	for(int i=0; i<x; i++) {
	    cell[i] = new CELL*[y];
		for(int j=0; j<y; j++) cell[i][j] = new CELL[z];
	}
}
PTPCore::~PTPCore()
{
	delete ptputil;

   	for(int i=0; i<ptpsize.x; i++){
	for(int j=0; j<ptpsize.y; j++) delete [] particle[i][j]; 
	delete [] particle[i]; }
	delete [] particle;
	
	for(int i=0; i<ptpsize.x; i++){
	for(int j=0; j<ptpsize.y; j++) delete [] cell[i][j]; 
	delete [] cell[i]; }
	delete [] cell;
}

int PTPCore::getSumOfParticlesNum(){
	return	ptp.n.x*ptp.n.y*ptp.n.z;
}



vecD PTPCore::getOriginalParticlePosition(const vecN n){
	vecD r;
	vecD d=ptp.cellScale;
	double surface_top;

	r.x = ptp.start.x + (n.x)*d.x;
	r.y = ptp.end.y - (n.y)*d.y;
	r.z = ptp.start.z + (n.z)*d.z;
	surface_top = -(ptp.plane_a*r.x*MM2M + ptp.plane_b*r.y*MM2M + ptp.plane_d)/(ptp.plane_c); 
	surface_top = surface_top - 8.14;
	//printf("----(%f)----\n", r.z*MM2M);	
	if(r.z > surface_top*M2MM) {
		r.z = surface_top*M2MM;
	//printf("+++(%f   %f)++++\n", r.z*MM2M, surface_top);
	}
	return r;
}

vecD PTPCore::getFootAABBBound(const FOOT* foot, const int type){
	vecD start;
	vecD end;


	switch(foot->type){
	case SPHERE:
		start.x = foot->loc.x - foot->r;
		start.z = foot->loc.z - foot->r;
		start.y = foot->loc.y - foot->r;

		end.x = foot->loc.x + foot->r;
		end.z = foot->loc.z + foot->r;
		end.y = foot->loc.y + foot->r;


		break;
	}
#ifdef USE_COLLISION_DEBUG
	DEBUG_GLOBAL.boundPointEnd = start;
	DEBUG_GLOBAL.boundPointStart = end;
#endif
	switch(type){
	case END:
		return end;
		break;
	case START:
		return start;
		break;
	default:
		return start;
		break;

	}
}

vecN PTPCore::judgeWhichCell(const vecD* loc){
		vecD d=ptp.cellScale;
		vecN r;
		vecD temp;
		temp.x = (loc->x-cell[0][0][ptp.n.z-1].start.x);  //yz
		temp.y = (loc->y-cell[0][0][ptp.n.z-1].start.y);
		temp.z = (loc->z-cell[0][0][ptp.n.z-1].start.z);
		//	printf("Point(%f,%f,%f)\n",temp.x,temp.y,temp.z);


		temp.x = (temp.x/d.x);
		temp.z = ptp.n.z - (temp.z/d.z);  //yz
		temp.y = (temp.y/d.y);			

		r.x = (int)temp.x;
		r.y = (int)temp.y;
		r.z = (int)temp.z;

		return r;
}


vecD PTPCore::getCellSize(){
		vecD discreteScale;

		vecD maxVec=ptputil->getMaxVec(&ptp.end,&ptp.start);
		vecD minVec=ptputil->getMinVec(&ptp.end,&ptp.start);

		if(ptp.n.x-1==0){
			discreteScale.x=(maxVec.x-minVec.x); 
		}else{
			discreteScale.x = (maxVec.x-minVec.x)/(ptp.n.x-1);
		}
		if(ptp.n.y-1==0){

			discreteScale.y = (maxVec.y-minVec.y);
			//	printf("...discreteScale.y = %f",discreteScale.y);
						
		}else{
			discreteScale.y = (maxVec.y-minVec.y)/(ptp.n.y-1);
		}

		if(ptp.n.z-1==0){

			discreteScale.z = (maxVec.z-minVec.z);
		}else{
			discreteScale.z = (maxVec.z-minVec.z)/(ptp.n.z-1);

		}
		return discreteScale;									///RETURN VAL[mm*MAG]
}


double PTPCore::getArea(const CELL* cell,const int direction){
		vecD cellSize = ptputil->minusVecD(&cell->end,&cell->start);	/// CELLSIZE[mm*MAG]
		double r=0;

		///cellSize[mm*MAG]
		switch (direction){
		case ZZ_AXIS: //x-y
			r=cellSize.x*cellSize.y;
			break;
		case YY_AXIS: //x-z
			r=cellSize.x*cellSize.z;
			break;
		case XX_AXIS: //y-z				
			r=cellSize.y*cellSize.z;
			break;
		default:
			break;
		}
		//r *= MM2M*MM2M;
		///RETURN VAL[mm^2*MAG^2]
		return r;
}



double PTPCore::getVolume(const CELL* cell){
		vecD cellSize = ptputil->minusVecD(&cell->end,&cell->start);	/// CELLSIZE[mm*MAG]
		double r=0;

		///cellSize[mm*MAG]

		r = cellSize.x*cellSize.y*cellSize.z;
		///RETURN VAL[mm^3*MAG^3]
		return r;
}

double PTPCore::getLength(const CELL* cell, const int type){
		vecD cellSize = ptputil->minusVecD(&cell->end,&cell->start);	/// CELLSIZE[mm*MAG]
		double r=0;
		switch(type){
		case XX_AXIS:
			r = cellSize.x;
			break;
		case YY_AXIS:
			r = cellSize.y;
			break;
		case ZZ_AXIS:
			r = cellSize.z;
			break;
		}
		///RETURN VAL[mm*MAG]
		return r;
}

vecN PTPCore::getComputeNeededCellNum(const FOOT* foot, const int type){
		vecN loopStartN=ptputil->setVecN(0,0,0);
		vecN loopEndN=ptputil->setVecN(0,0,0);
		vecN bigN = ptputil->setVecN(0,0,0);
		vecN smallN = ptputil->setVecN(0,0,0);

		vecN rN=ptputil->setVecN(0,0,0);

		vecD loopStart= 	getFootAABBBound(foot,START);
		vecD loopEnd =		getFootAABBBound(foot,END);


		loopStartN = judgeWhichCell(&loopStart);
		loopEndN = judgeWhichCell(&loopEnd);

		bigN=ptputil->getMaxVecN(&loopStartN,&loopEndN);
		smallN=ptputil->getMinVecN(&loopStartN,&loopEndN);

		loopStartN = smallN;
		loopEndN = bigN;

		loopEndN.x++;
		loopEndN.y++;
		loopEndN.z++;

		if(loopStartN.x<0) loopStartN.x = 0;
		if(loopStartN.y<0) loopStartN.y = 0;
		if(loopStartN.z<0) loopStartN.z = 0;
		if(loopStartN.x>ptp.n.x) loopStartN.x = ptp.n.x;
		if(loopStartN.y>ptp.n.y) loopStartN.y = ptp.n.y;
		if(loopStartN.z>ptp.n.z) loopStartN.z = ptp.n.z;

		if(loopEndN.x<0) loopEndN.x = 0;
		if(loopEndN.y<0) loopEndN.y = 0;
		if(loopEndN.z<0) loopEndN.z = 0;
		if(loopEndN.x>ptp.n.x) loopEndN.x = ptp.n.x;
		if(loopEndN.y>ptp.n.y) loopEndN.y = ptp.n.y;
		if(loopEndN.z>ptp.n.z) loopEndN.z = ptp.n.z;


		switch(type){
		case START:
		case PTP_START:
			rN = loopStartN;
			break;
		case END:
		case PTP_END:
			rN = loopEndN;
			break;
		default:
			printf("error");

			break;

		}
		return rN;
}

bool PTPCore::initializePTP(const vecN *n,const vecD *s,const vecD *e){	

	register int i,j,k;		
	ptp.start = ptputil->setVecD( MAG*s->x, MAG*s->y, MAG*s->z);    /** *soil surface size = start*end */
	ptp.end = ptputil->setVecD(MAG*e->x,MAG*e->y,MAG*e->z);
	ptp.n = ptputil->setVecN(n->x,n->y,n->z);
	ptp.cellScale = getCellSize();
	
/** *build cells*/
	for(i=0;i<n->x;i++){
		for(j=0;j<n->y;j++){
			for(k=0;k<n->z;k++){
				cell[i][j][k].start = ptputil->setVecD(
					ptp.start.x-(ptp.cellScale.x*0.5f)+ (i*ptp.cellScale.x), 
					ptp.start.y-(ptp.cellScale.y*0.5f)+ (j*ptp.cellScale.y),  //-+
					ptp.end.z-(ptp.cellScale.z*0.5f)- (k*ptp.cellScale.z) //+- 
					);
				cell[i][j][k].end = ptputil->setVecD(
					ptp.start.x+(ptp.cellScale.x*0.5f)+((i)*ptp.cellScale.x),
					ptp.start.y +(ptp.cellScale.y*0.5f)+((j)*ptp.cellScale.y),  //-+
					ptp.end.z+(ptp.cellScale.z*0.5f)-((k)*ptp.cellScale.z)  //+-
					);
				cell[i][j][k].center = ptputil->getMiddlePoint(&cell[i][j][k].start,&cell[i][j][k].end);

				cell[i][j][k].CollisionParticleNum =0;
				cell[i][j][k].particleNum = 1;
			
				cell[i][j][k].averageLoc = ptputil->setVecD(0,0,0);
			}
		}
	}

/** build particles*/
	for(i=0;i<n->x;i++){
		for(j=0;j<n->y;j++){
			for(k=0;k<n->z;k++){

				particle[i][j][k].loc = getOriginalParticlePosition(ptputil->setVecN(i,j,k));
				particle[i][j][k].cloc = ptputil->setVecN(i,j,k);
				
					judgeWhichCell(&particle[i][j][k].loc);
			}
		} 
	}

	return true;
}
vecD PTPCore::particleMovement(const vecD* point,const FOOT* lastFoot){
	vecD E = ptputil->setVecD(0.0,0.0,0.0);
	vecD R= ptputil->setVecD(0.0,0.0,0.0);
	vecD movement = ptputil->normalizeVector(&lastFoot->vel);

	double Rs=0.0;
	R.x = ((point->x/MAG)-lastFoot->loc.x);

	R.y = ((point->y/MAG)-(lastFoot->loc.y));
	R.z = ((point->z/MAG)-lastFoot->loc.z);
	Rs = sqrt(ptputil->getVecDSizeSQR(&R,ALL));
	E = ptputil->setVecD( (1./(Rs*Rs*Rs)),  (1./(Rs*Rs*Rs)),  (1./(Rs*Rs*Rs)));
	E = ptputil->multipleVecD(&E,&R);
	E = ptputil->normalizeVector(&E);

#ifdef USE_GRAVITY
	E.z -= GRAVITY;	//yz	///FIXED
#endif

	movement.x *= PANEL_ZX;
	movement.y *= PANEL_ZX; //0 PANEL_ZX
	movement.z *= 0;		//PANEL_ZX 0
	//	if(lastFoot->vel.x>0) {Sx = -PANEL_Y; }else if(lastFoot->vel.x<0){ Sx = PANEL_Y; }
	//	if(lastFoot->vel.y>0) {Sy = 0; }
	//	if(lastFoot->vel.z>0) {Sz = -PANEL_Y; }else if(lastFoot->vel.z<0){ Sz = PANEL_Y; }
	E.x -=movement.x;
	E.y -=movement.y;  //zy

	//zE = normalizeVector(&E);


	return E;

}

bool PTPCore::AABBCollision(CELL* pAABB1, CELL* pAABB2)
{
	if(FLOAT_EQ(pAABB1->end.x,pAABB2->start.x))
		return true;
	if(FLOAT_EQ(pAABB1->end.y,pAABB2->start.y))
		return true;
	if(FLOAT_EQ(pAABB1->end.z,pAABB2->start.z))
		return true;
	if( pAABB1->end.x < pAABB2->start.x || 
		pAABB1->start.x > pAABB2->end.x )
		return false;
	if( pAABB1->end.y < pAABB2->start.y ||
		pAABB1->start.y > pAABB2->end.y )
		return false;
	if( pAABB1->end.z < pAABB2->start.z ||
		pAABB1->start.z > pAABB2->end.z )
		return false;


	return true;
}

int PTPCore::particleWorks(const vecN* particleN,const FOOT* foot, const FOOT* lastFoot, const int pLocData){		///PARTICLE
	double a=0.0, b=0.0, c=0.0;
	double k=0.0;

	PARTICLE* pp = &particle[particleN->x][particleN->y][particleN->z];
	vecN lastCLoc = pp->cloc;
	vecN tempCloc = ptputil->setVecN(-1,-1,-1);
	vecD tempLoc = ptputil->setVecD(0.f,0.f,0.f);
	vecD ppDirection = ptputil->setVecD(0.f,0.f,0.f);
	vecD contactDirection = ptputil->setVecD(0.f,0.f,0.f);

	vecD NfV;

#ifdef USE_MOVEMENT_FUNCTION
	NfV = particleMovement(&pp->loc,lastFoot);
//	NfV =ptputil->setVecD(10.f,0.f,10.f);  //!!
	NfV = ptputil->normalizeVector(&NfV); //normalizeVector(foot->vel);
//printf("..(%f %f %f)....(%f %f %f)....\n", NfV.x, NfV.y, NfV.z, lastFoot->loc.x, lastFoot->loc.y,lastFoot->loc.z);

			ppDirection = ptputil->minusVecD(&pp->loc,&foot->loc);
			ppDirection = ptputil->normalizeVector(&ppDirection);
			contactDirection = ptputil->normalizeVector(&foot->vel);
		//	contactDirection = ptputil->plusVecD(&contactDirection, &GDirection);
		//	contactDirection = ptputil->normalizeVector(&contactDirection);
			double cosTheta, collReact;
			cosTheta = ptputil->getCosTheta(&ppDirection,&contactDirection);
			
//	printf("(%f %f)..(%f %f %f)..(%f, %f, %f)\n", contactDirection.y,cosTheta, pp->loc.x, pp->loc.y, pp->loc.z,foot->vel.x, foot->vel.y, foot->vel.z);		
	
#endif
#ifndef USE_MOVEMENT_FUNCTION
	NfV = ptputil->normalizeVector(&foot->vel);
	
			ppDirection = ptputil->minusVecD(&pp->loc,&foot->loc);
			ppDirection = ptputil->normalizeVector(&ppDirection);
			contactDirection = ptputil->normalizeVector(&foot->vel);
		//	contactDirection = ptputil->plusVecD(&contactDirection, &GDirection);
		//	contactDirection = ptputil->normalizeVector(&contactDirection);
			double cosTheta, collReact;
			cosTheta = ptputil->getCosTheta(&ppDirection,&contactDirection);
#endif
	//	a = (NfV.x*NfV.x)+(NfV.y*NfV.y)+(NfV.z*NfV.z);
		//a = 1;//(NfV.x*NfV.x)+(NfV.y*NfV.y)+(NfV.z*NfV.z);
		//b = 2.0*((pp->loc.x*NfV.x)+(pp->loc.y*NfV.y)+(pp->loc.z*NfV.z)-((foot->loc.x*NfV.x)+(foot->loc.y*NfV.y)+(foot->loc.z*NfV.z)));
		//c = (SQR(pp->loc.x)+SQR(foot->loc.x)-(2*pp->loc.x*foot->loc.x))+(SQR(pp->loc.y)+SQR(foot->loc.y)-(2*pp->loc.y*foot->loc.y))+SQR(pp->loc.z)+SQR(foot->loc.z)-(2*pp->loc.z*foot->loc.z)-SQR(foot->r);
		//k = ptputil->quad_eqn(a,b,c);
		
		vecD foot_pp = ptputil->setVecD(0.f,0.f,0.f);
		foot_pp = ptputil->minusVecD(&pp->loc, &foot->loc);
		vecD unit_foot_pp = ptputil->setVecD(0.f,0.f,0.f); 
		unit_foot_pp = ptputil->normalizeVector(&foot_pp);
		double scala_foot_pp = sqrt(SQR(foot_pp.x)+SQR(foot_pp.y)+SQR(foot_pp.z));
		if(scala_foot_pp < foot->r) {
			tempLoc.x = foot->r*unit_foot_pp.x + foot->loc.x;
			tempLoc.y = foot->r*unit_foot_pp.y + foot->loc.y;
			tempLoc.z = foot->r*unit_foot_pp.z + foot->loc.z;									
			} 
		else{
				
		tempLoc.x = pp->loc.x;// + (k*NfV.x)*collReact;  /** tempLoc is new location vector of particle, but still not updated*/
		tempLoc.y = pp->loc.y;// + (k*NfV.y)*collReact;  /**..............for test..20141218  */
		tempLoc.z = pp->loc.z;// + (k*NfV.z)*collReact;  
				
			}
		
		////collReact = SPREAD*cosTheta; //TBD
		//collReact = (1 + SPREAD*sqrt(1-cosTheta*cosTheta));
		////collReact = 1;
		
		//tempLoc.x = pp->loc.x + (k*NfV.x)*collReact;  /** tempLoc is new location vector of particle, but still not updated*/
		//tempLoc.y = pp->loc.y + (k*NfV.y)*collReact;  /**..............for test..20141218  */
		//tempLoc.z = pp->loc.z + (k*NfV.z)*collReact;  

	if((lastCLoc.x >=0 && lastCLoc.x < ptp.n.x) &&
		(lastCLoc.y >=0 && lastCLoc.y < ptp.n.y) &&
		(lastCLoc.z >=0 && lastCLoc.z < ptp.n.z) ){

			//printf("P %d,%d,%d : %d \n",lastCLoc.x,lastCLoc.y,lastCLoc.z,cell[lastCLoc.x][lastCLoc.y][lastCLoc.z].particleNum); 
			cell[lastCLoc.x][lastCLoc.y][lastCLoc.z].particleNum--;
			/*
			if(cell[lastCLoc.x][lastCLoc.y][lastCLoc.z].particleNum==0){
			cell[lastCLoc.x][lastCLoc.y][lastCLoc.z].CollisionType=NOT_COLLISION;
			}
			*/
			tempCloc = judgeWhichCell(&tempLoc);   /** find cell ID with collision of Nth particle 'tempLoc'  */
//			printf("(%d,%d,%d),[%d,%d,%d]\n",particleN->x,particleN->y,particleN->z,tempCloc.x,tempCloc.y,tempCloc.z);
			if((tempCloc.x>=0 &&tempCloc.x<ptp.n.x) &&
				(tempCloc.y>=0 && tempCloc.y<ptp.n.y) &&
				(tempCloc.z>=0 && tempCloc.z<ptp.n.z) ){
					cell[tempCloc.x][tempCloc.y][tempCloc.z].particleNum++;
					if(cell[tempCloc.x][tempCloc.y][tempCloc.z].particleNum == 1)    
					/// particleNum must be higher than 0 because of collision
					{
						cell[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc = tempLoc;  //
					} 
					else if(cell[tempCloc.x][tempCloc.y][tempCloc.z].particleNum == 2) 
					{
						cell[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc = ptputil->getMiddlePoint(&pp->loc,&tempLoc);  
						
					}
					else if(cell[tempCloc.x][tempCloc.y][tempCloc.z].particleNum > 2)
					{
						cell[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc = 
						ptputil->getMiddlePoint(&cell[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc,&tempLoc);
					}
     		//		printf("(%d,%d,%d),[%d,%d,%d], {%d, %f %f}\n",particleN->x,particleN->y,particleN->z,tempCloc.x,tempCloc.y,tempCloc.z,
     		//		         cell[tempCloc.x][tempCloc.y][tempCloc.z].particleNum, cell[tempCloc.x][tempCloc.y][tempCloc.z].averageLoc.z, tempLoc.z);   //yz
   	        	     				
					cell[tempCloc.x][tempCloc.y][tempCloc.z].CollisionParticleNum++;

			}
	}

	pp->cloc =tempCloc;      /** particle ID/location will be updated */
	pp->loc = tempLoc;

	if(!FLOAT_EQ(lastCLoc.x,tempCloc.x)|| 
		!FLOAT_EQ(lastCLoc.y,tempCloc.y)||
		!FLOAT_EQ(lastCLoc.z,tempCloc.z)
		){
			return COLLISION;
	}else{
		return NOT_COLLISION;
	}

}

int PTPCore::particleCollisionDetect(const FOOT* foot, const PARTICLE* particle){
	int r = NOT_COLLISION;
	if(foot->type==SPHERE){
		r = isInASphere(foot,&particle->loc);
	}else{
		r = NOT_COLLISION;
		printf("Particle Collision Error\n");
	}
	return r;
}

bool PTPCore::PTPCollisionDetectOne(const FOOT* foot){
	vecD ptpCenter = ptputil->getMiddlePoint(&ptp.start,&ptp.end);
	double rptp = sqrt(ptputil->getLengthSQR(&cell[0][0][ptp.n.z-1].start,&ptpCenter,ALL));    //yz
	bool r=false;

	switch(foot->type){
	case SPHERE:
		if((ptputil->getLengthSQR(&foot->loc,&ptpCenter,ALL)<=SQR2(foot->r,rptp))){
			r=true;
		}
		break;
	default:
		printf("PTPCollision ERROR\n");
	}
	return r;

}

int PTPCore::ptpWorks(const FOOT* foot,const FOOT* lastFoot){
	register int i,j,k;
	vecN n;
	bool flagPTP;
	int pLocData;

	vecN loopStart;
	vecN loopEnd;
	
	flagPTP = PTPCollisionDetectOne(foot);

#ifdef FAST_CALC
	loopStart = getComputeNeededCellNum(foot,PTP_START);
	loopEnd= getComputeNeededCellNum(foot,PTP_END);
	if((loopStart.x>loopEnd.x) || (loopStart.y>loopEnd.y) || (loopStart.z>loopEnd.z)){
		printf("ERROR FAST CALC RANGE\n");
	}
#else
	loopStart = ptputil->setVecN(0,0,0);
	loopEnd = ptp.n;
#endif
	//printf("1(%f,%f,%f),(%f,%f,%f) \n",lastFoot->loc.x,lastFoot->loc.y,lastFoot->loc.z,lastFoot->vel.x,lastFoot->vel.y,lastFoot->vel.z);
	//printf("2(%f,%f,%f),(%f,%f,%f) \n\n",foot->loc.x,foot->loc.y,foot->loc.z,foot->vel.x,foot->vel.y,foot->vel.z);



	if(flagPTP==true){
		for(i=0;i<ptp.n.x;i++){
			for(j=0;j<ptp.n.y;j++){
				for(k=0;k<ptp.n.z;k++){
					cell[i][j][k].CollisionParticleNum=0;
				}
			}
		}
		///PARTICLES
		for(i=0;i<ptp.n.x;i++){
			for(j=0;j<ptp.n.y;j++){
				for(k=0;k<ptp.n.z;k++){
					n=ptputil->setVecN(i,j,k);
					//CellWorks(foot,&n);
					///Move Particles	
					pLocData = particleCollisionDetect(foot,&particle[i][j][k]);
					if(pLocData!=NOT_COLLISION){
						particleWorks(&n,foot,lastFoot,pLocData);
						particle[i][j][k].isCollision = COLLISION;   //draw only collied particles
											//	printf("####collision### (%d %d %d(\n", i,j,k);
					}else{
						particle[i][j][k].isCollision = NOT_COLLISION;
					}
					//for(l=0;l<6;l++){
						//pLocData = particleCollisionDetect(foot,&particle[i][j][k]);
						//if(pLocData!=NOT_COLLISION){
						//particleWorks(&n,foot,lastFoot,pLocData);
						//particle[i][j][k].isCollision = COLLISION;
						//printf("####collision### (%d %d %d(\n", i,j,k);
						//break;    //it is supposed to exist just one collision among one particle and six legs 
					    //}else{
						//particle[i][j][k].isCollision = NOT_COLLISION;
						//}

					//}

				}
			}
		}
		return 1;
	}
	return 0;
}	
	
double PTPCore::shearStress(const double disp, const double pressure){  
	// scalar shear stress in the direction of velocity 
	double sStress;
	sStress = (COHESION + pressure*tan(INT_ANGLE))*(1-exp(-ABS(disp)/DEFORMATION_MODULE));
	return sStress;
}  
bool PTPCore::calcForce(FOOT* foot, const int type, vecD* forceR){
	register int i,j,k;
	vecN loopStart;
	vecN loopEnd;

	vecD force = ptputil->setVecD(0.f,0.f,0.f);
	vecD fCell=ptputil->setVecD(0.f,0.f,0.f);
	double pressure = 0.f;
	vecD normalDirection= ptputil->setVecD(0.f,0.f,0.f);
	vecD cellSize = ptputil->setVecD(0.f,0.f,0.f);
	double areaX=0,areaY=0,areaZ=0;
	double fZCellSize = 0.f;

	double PrR_Size = 0.f;	
	vecD GDirection = ptputil->setVecD(0.f,0.f,0.f);
	vecD normalCellForce = ptputil->setVecD(0.f,0.f,0.f);
	vecD shearCellForce = ptputil->setVecD(0.f,0.f,0.f);
	vecD PrG = ptputil->setVecD(0.f,0.f,0.f);
	vecD fRDirection = ptputil->setVecD(0.f,0.f,0.f);
	vecD forceXY = ptputil->setVecD(0.f,0.f,0.f);
	double forceXY_Size = 0.f;
	vecD lateral_loc = ptputil->setVecD(0.f,0.f,0.f);
	double 	shearStress_Size;
	
	foot->contact = false;
	

	if(type==FAST){
		loopStart = getComputeNeededCellNum(foot,START);
		loopEnd= getComputeNeededCellNum(foot,END);
	}else{
		loopStart = ptputil->setVecN(0,0,0);
		loopEnd = ptp.n;
	}	
	if((loopStart.x>loopEnd.x) || (loopStart.y>loopEnd.y) || (loopStart.z>loopEnd.z)){
		printf("ERROR FAST CALC RANGE\n");
	}

	//printf("Loop (%d,%d,%d) (%d,%d,%d)\n",loopStart.x,loopStart.y,loopStart.z,loopEnd.x,loopEnd.y,loopEnd.z);



//printf("###########\n");

	for(i=loopStart.x;i<loopEnd.x;i++){    /** loopStart and loopEnd are contact area with object  */
		for(k=loopStart.z;k<loopEnd.z;k++){
			for(j=loopStart.y;j<loopEnd.y;j++){
				/// F = P*A?
				foot->contact = true; 
				cellSize = ptputil->setVecD(0.f,0.f,0.f);	/// CELLSIZE[mm*MAG]

				if(cell[i][j][k].CollisionParticleNum){
					pressure = lookupTable(&cell[i][j][k]);	///[Pa]
					cellSize = ptputil->minusVecD(&cell[i][j][k].end,&cell[i][j][k].start);	/// CELLSIZE[mm*MAG]

					areaX = cellSize.y*cellSize.z*MM2M*MM2M;///[m^2*MAG^2]
					areaY = cellSize.x*cellSize.z*MM2M*MM2M;
					areaZ = cellSize.x*cellSize.y*MM2M*MM2M;
					
				//	printf("(%f %f %f %f) #%f#\n", cellSize.x, cellSize.y, cellSize.z, pressure, (foot->loc.z-25)*MM2M);
					//printf("<%d %d %d>\n", i, j, k);
					///Get Normal Vector(Normalized)
					normalDirection = ptputil->minusVecD(&cell[i][j][k].center,&foot->loc);
					normalDirection = ptputil->normalizeVector(&normalDirection);

					fZCellSize = ABS(areaZ*pressure);     //yz
					vecD moveDirection = ptputil->setVecD(0.f,0.f,0.f); 
					vecD schearCellForce = ptputil->setVecD(0.f,0.f,0.f); 
	
					if (ptp.lateral == 1 ) {
					lateral_loc = ptputil->setVecD(foot->loc.x,foot->loc.y,0.f);     //zy 
					lateral_loc = ptputil->minusVecD(&lateral_loc,&ptp.start_loc);
					moveDirection = ptputil->normalizeVector(&foot->vel);
    				shearStress_Size = areaZ*shearStress(ptputil->sqrtVecDP(&lateral_loc)*MM2M, pressure);   //yz
    				shearCellForce.x = moveDirection.x*shearStress_Size;
    				shearCellForce.y = moveDirection.y*shearStress_Size;
     				shearCellForce.z = moveDirection.z*shearStress_Size;
     				   				
					}		
		//printf("(%d) cfz=%f sss=%f p=%f a=%f sloc=%f, \n",ptp.lateral, fZCellSize, shearStress_Size, pressure, areaY, ptputil->sqrtVecDP(&lateral_loc)*MM2M);
					
					///Get Fr,FR Direction (Normalized)
					if(foot->type == SPHERE){
					
					fRDirection.x = normalDirection.x;
					fRDirection.y = normalDirection.y;
					fRDirection.z = normalDirection.z;
						
					GDirection = ptputil->setVecD(0.f,0.f,-1.f);   //yz     //gravity direction
					
					PrG.x = GDirection.x*fZCellSize;
					PrG.y = GDirection.y*fZCellSize;
					PrG.z = GDirection.z*fZCellSize;
					
					PrR_Size = ABS(ptputil->InnerProduct(&PrG,&fRDirection));		
					//velocity drection pressure					
					
					normalCellForce.x = fRDirection.x*PrR_Size;//*0.02f;
					normalCellForce.y = fRDirection.y*PrR_Size;//*0.02f;
					normalCellForce.z = fRDirection.z*PrR_Size;
					} else { printf(" foot type ERROR! \n"); }
		
 					//printf("(%f %f %f)  (%f %f %f)\n",force.x, force.y, force.z, shearCellForce.x, shearCellForce.y, shearCellForce.z);
					fCell = ptputil->plusVecD(&normalCellForce, &shearCellForce);
					
				
					force.x += fCell.x;
					force.y += fCell.y;    
					force.z += fCell.z;
					

					///F=P*A[(Pa)*(mm^2*MAG^2)]=[N*1000*MAG^2]
				}
			}
		}
	}


	//printf("Cell : %d ,Particle : %d \n",collisionCellNum,collisionParticleNum);
	//		printf("Cell : %d-%dOverlap(%d) ,Particle : %d Area(%e,%e,%e)\n",validCellNum,overlapNum,collisionCellNum,collisionParticleNum,areaDebug.x,areaDebug.y,areaDebug.z);

	force.x /= MAG*MAG; //*M2MM*M2MM;
	force.y /= MAG*MAG; //*M2MM*M2MM;
	force.z /= MAG*MAG; //*M2MM*M2MM;

	
	forceR->x = force.x;
	forceR->y = force.y;
	forceR->z = force.z;
	

	forceXY = ptputil->setVecD(forceR->x,forceR->y, 0.f);   //yz
	forceXY_Size = ptputil->sqrtVecDP(&forceXY);
	if(forceXY_Size >=  SHEAR_THRESHOLD && ptp.lateral == 0){
	ptp.lateral = 1; //on
	ptp.start_loc = ptputil->setVecD(foot->loc.x,foot->loc.y, 0.f); }  //yz
	else if(forceXY_Size < SHEAR_THRESHOLD && ptp.lateral == 1)
	{ ptp.lateral = 0; } //off
	
//	printf("(contact %d)..forceXY = %f (%f %f %f)\n",ptp.lateral,forceXY_Size,forceR->x,forceR->y,forceR->z);
		
	
	return true;
	///F=-A+k*abs(pow((z-z0/1000),n))
	///RETURN VALUE : FORCE(N)
}

double PTPCore::lookupTable(const CELL* cell){
	double k,n,r,temp;

	k = LOOKUP_K;	
	n = LOOKUP_N;


	if(ptp.end.z  >=  cell->averageLoc.z) { 
		temp = (ptp.end.z - cell->averageLoc.z)*MM2M; }  
	else {   //when bulldozing
	    temp = 0;}//ptp.end.z*MM2M; }  
	//if(temp > 0) temp=0;
	r = k*pow(ABS(temp),n);
	
	//printf("##  (%f %f) %f %f \n", ptp.end.z, cell->averageLoc.z, r, temp);
	
	return r;
	/// RETURN VALUE : PRESSURE[Pa]
}

int PTPCore::isInASphere(const FOOT* foot, const vecD* point){
	int r = NOT_COLLISION;
	vecD vec = ptputil->minusVecD(&foot->loc,point);
	if(ptputil->getVecDSizeSQR(&vec,ALL)<SQR(foot->r)){
		r = IS_IN;
	}else if(FLOAT_EQ(ptputil->getVecDSizeSQR(&vec,ALL),SQR(foot->r))){
		r = IS_ON;
	}
	return r;
}

  } // end of namespace sim
} // end of namespace mars
