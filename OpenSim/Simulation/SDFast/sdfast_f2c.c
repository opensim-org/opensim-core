// sdfast_wrapper.c
// The routines in this file provide wrappers for the C versions of sdfast
// routines when called fortran code.  On UNIX systems, a trailing underscore
// is often appended to subroutines or functions called within a fortran
// file.  The methods here convert the fortran calls to the correct C calls.

#include "sdfast.h"

// INITIALIZE
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdinit_()
{
	sdinit();
}

// GRAVITY
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdgrav_(double aGrav[3])
{
	sdgrav(aGrav);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdgetgrav_(double rGrav[3])
{
	sdgetgrav(rGrav);
}

// SPECIFY SYSTEM STATE
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdstate_(double *aT,double *aQ,double *aU)
{
	double t = *aT;
	sdstate(t,aQ,aU);
}

// APPLY LOADS AND PRESCRIBE MOTIONS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdbodyt_(int *aBody,double aTorque[3])
{
	int body = *aBody - 1;
	sdbodyt(body,aTorque);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdpointf_(int *aBody,double aPoint[3],double aForce[3])
{
	int body = *aBody - 1;
	sdpointf(body,aPoint,aForce);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdhinget_(int *aJoint,int *aAxis,double *aTorque)
{
	int joint = *aJoint - 1;
	int axis = *aAxis - 1;
	double torque = *aTorque;
	sdhinget(joint,axis,torque);
}

// CALCULATE DERIVATIVES
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdderiv_(double *dqdt,double *dudt)
{
	sdderiv(dqdt,dudt);
}

// LOAD ACCESS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdgetht_(int *aJoint,int *aAxis,double *aForce)
{
	int joint = *aJoint - 1;
	int axis = *aAxis - 1;
	sdgetht(joint,axis,aForce);
}

// EQUATIONS OF MOTION
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdrel2cart_(int *aCoord,int *aBody,double pt[3],double lin[3],double rot[3])
{
	int coord = *aCoord - 1;
	int body = *aBody - 1;
	sdrel2cart(coord,body,pt,lin,rot);
}

// KINEMATICS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdpos_(int *aBody,double aPoint[3],double rPos[3])
{
	int body = *aBody - 1;
	sdpos(body,aPoint,rPos);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdvel_(int *aBody,double aPoint[3],double rVel[3])
{
	int body = *aBody - 1;
	sdvel(body,aPoint,rVel);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdacc_(int *aBody,double aPoint[3],double rAcc[3])
{
	int body = *aBody - 1;
	sdacc(body,aPoint,rAcc);
}

// BODY KINEMATICS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdorient_(int *aBody,double rDirCos[3][3])
{
	int body = *aBody - 1;
	sdorient(body,rDirCos);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdangvel_(int *aBody,double rAngVel[3])
{
	int body = *aBody - 1;
	sdangvel(body,rAngVel);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdangacc_(int *aBody,double rAngAcc[3])
{
	int body = *aBody - 1;
	sdangacc(body,rAngAcc);
}

// COORDINATE FRAME TRANSFORMATIONS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdtrans_(int *aBody1,double aVec1[3],int *aBody2,double rVec2[3])
{
	int body1 = *aBody1 - 1;
	int body2 = *aBody2 - 1;
	sdtrans(body1,aVec1,body2,rVec2);
}

// OBTAIN ACCELERATION AND LOAD INFORMATION
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdcomptrq_(double *aUDot,double *rF)
{
	sdcomptrq(aUDot,rF);
}

// ANGLE CONVERSION

// States / Euler Angles
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdang2st_(double *aQAng,double *rQ)
{
	sdang2st(aQAng,rQ);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdst2ang_(double *aQ,double *rQAng)
{
	sdst2ang(aQ,rQAng);
}

// Direction Cosines / Euler Angles
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sddc2ang_(double aDirCos[3][3],double *rE1,double *rE2,double *rE3)
{
	sddc2ang(aDirCos,rE1,rE2,rE3);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdang2dc_(double *aE1,double *aE2,double *aE3,double rDirCos[3][3])
{
	sdang2dc(*aE1,*aE2,*aE3,rDirCos);
}

// Direction Cosines / Quaternions
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void
sddc2quat_(double aDirCos[3][3],double *rQ1,double *rQ2,double *rQ3,double *rQ4)
{
	sddc2quat(aDirCos,rQ1,rQ2,rQ3,rQ4);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdquat2dc_(double *aQ1,double *aQ2,double *aQ3,double *aQ4,
					double rDirCos[3][3])
{
	sdquat2dc(*aQ1,*aQ2,*aQ3,*aQ4,rDirCos);
}

// TOPOLOGY
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdinfo_(int info[50])
{
	sdinfo(info);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdjnt_(int *aJoint,int info[50],int slider[6])
{
	int joint = *aJoint - 1;
	sdjnt(joint,info,slider);
}

// INERTIAL PARAMETERS
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdsys_(double *aM,double rCOM[3],double rI[3][3])
{
	sdsys(aM,rCOM,rI);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdgetmass_(int *aBody,double *rMass)
{
	int body = *aBody - 1;
	sdgetmass(body,rMass);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sdgetiner_(int *aBody,double rInertia[3][3])
{
	int body = *aBody - 1;
	sdgetiner(body,rInertia);
}
