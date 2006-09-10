#ifndef _sdfast_h_
#define _sdfast_h_
// sdfast.h
#include <stdio.h>

#endif

#ifdef __linux__
#ifndef STDCALL
#define STDCALL __attribute__((stdcall))
#endif
#else
#define STDCALL __stdcall
#endif

#ifdef __cplusplus
extern "C" {
#endif

// INITIALIZE
void STDCALL SDINIT();
void sdinit_();
void sdinit();

// GRAVITY
void STDCALL SDGRAV(double aGrav[3]);
void sdgrav_(double aGrav[3]);
void sdgrav(double aGrav[3]);
void STDCALL SDGETGRAV(double rGrav[3]);
void sdgetgrav_(double rGrav[3]);
void sdgetgrav(double rGrav[3]);

// SPECIFY SYSTEM STATE
void STDCALL SDSTATE(double *t,double *q,double *u);
void sdstate_(double *t,double *q,double *u);
void sdstate(double t,double *q,double *u);
void STDCALL SDASSEMBLE(double *time,double *state,int *lock,double *tol,
				int *maxevals,int *fcnt,int *err );
void sdassemble_(double *time,double *state,int *lock,double *tol,
				int *maxevals,int *fcnt,int *err );
void sdassemble(double time,double *state,int *lock,double tol,
				int maxevals,int *fcnt,int *err );

// APPLY LOADS AND PRESCRIBE MOTIONS
void STDCALL SDBODYT(int *aBody,double aTorque[3]);
void sdbodyt_(int *aBody,double aTorque[3]);
void sdbodyt(int aBody,double aTorque[3]);
void STDCALL SDPOINTF(int *aBody,double aPoint[3],double aForce[3]);
void sdpointf_(int *aBody,double aPoint[3],double aForce[3]);
void sdpointf(int aBody,double aPoint[3],double aForce[3]);
void STDCALL SDHINGET(int *aJoint,int *aAxis,double *aF);
void sdhinget_(int *aJoint,int *aAxis,double *aF);
void sdhinget(int aJoint,int aAxis,double aF);
void STDCALL SDPRES(int *aJoint,int *aAxis,int *aPrescribed);
void sdpres_(int *aJoint,int *aAxis,int *aPrescribed);
void sdpres(int aJoint,int aAxis,int aPrescribed);

// CALCULATE DERIVATIVES
void STDCALL SDDERIV(double *dqdt,double *dudt);
void sdderiv_(double *dqdt,double *dudt);
void sdderiv(double *dqdt,double *dudt);

// LOAD ACCESS
void STDCALL SDGETHT(int *aJoint,int *aAxis,double *aForce);
void sdgetht_(int *aJoint,int *aAxis,double *aForce);
void sdgetht(int aJoint,int aAxis,double *aForce);
void STDCALL SDREAC(double forces[][3],double torques[][3]);
void sdreac_(double forces[][3],double torques[][3]);
void sdreac(double forces[][3],double torques[][3]);
void STDCALL SDEQUIVHT(double force[]);
void sdequivht_(double force[]);
void sdequivht(double force[]);

// EQUATIONS OF MOTION
void STDCALL SDMASSMAT(double *rI);
void sdmassmat_(double *rI);
void sdmassmat(double *rI);
void SDREL2CART(int *coord,int *body,double pt[3],double lin[3],double rot[3]);
void sdrel2cart_(int *coord,int *body,double pt[3],double lin[3],double rot[3]);
void sdrel2cart(int coord,int body,double pt[3],double lin[3],double rot[3]);

// KINEMATICS
extern void STDCALL SDPOS(int *aBody,double aPoint[3],double rPos[3]);
extern void sdpos_(int *aBody,double aPoint[3],double rPos[3]);
extern void sdpos(int aBody,double aPoint[3],double rPos[3]);
void STDCALL SDVEL(int *aBody,double aPoint[3],double rVel[3]);
void sdvel_(int *aBody,double aPoint[3],double rVel[3]);
void sdvel(int aBody,double aPoint[3],double rVel[3]);
void STDCALL SDACC(int *aBody,double aPoint[3],double rAcc[3]);
void sdacc_(int *aBody,double aPoint[3],double rAcc[3]);
void sdacc(int aBody,double aPoint[3],double rAcc[3]);

// BODY KINEMATICS
void STDCALL SDORIENT(int *aBody,double rDirCos[3][3]);
void sdorient_(int *aBody,double rDirCos[3][3]);
void sdorient(int aBody,double rDirCos[3][3]);
void STDCALL SDANGVEL(int *aBody,double rAngVel[3]);
void sdangvel_(int *aBody,double rAngVel[3]);
void sdangvel(int aBody,double rAngVel[3]);
void STDCALL SDANGACC(int *aBody,double rAngAcc[3]);
void sdangacc_(int *aBody,double rAngAcc[3]);
void sdangacc(int aBody,double rAngAcc[3]);

// COORDINATE FRAME TRANSFORMATIONS
void STDCALL SDTRANS(int *aBody1,double aVec1[3],int *aBody2,double rVec2[3]);
void sdtrans_(int *aBody1,double aVec1[3],int *aBody2,double rVec2[3]);
void sdtrans(int aBody1,double aVec1[3],int aBody2,double rVec2[3]);

// OBTAIN ACCELERATION AND LOAD INFORMATION
void STDCALL SDCOMPTRQ(double *aUDot,double *rF);
void sdcomptrq_(double *aUDot,double *rF);
void sdcomptrq(double *aUDot,double *rF);

// ANGLE CONVERSION
// States / Euler Angles
void STDCALL SDANG2ST(double *qang,double *q);
void sdang2st_(double *qang,double *q);
void sdang2st(double *qang,double *q);
void STDCALL SDST2ANG(double *q,double *qang);
void sdst2ang_(double *q,double *qang);
void sdst2ang(double *q,double *qang);
// Direction Cosines / Euler Angles
void STDCALL SDDC2ANG(double aDirCos[3][3],double *rE1,double *rE2,double *rE3);
void sddc2ang_(double aDirCos[3][3],double *rE1,double *rE2,double *rE3);
void sddc2ang(double aDirCos[3][3],double *rE1,double *rE2,double *rE3);
void STDCALL SDANG2DC(double *aE1,double *aE2,double *aE3,double rDirCos[3][3]);
void sdang2dc_(double *aE1,double *aE2,double *aE3,double rDirCos[3][3]);
void sdang2dc(double aE1,double aE2,double aE3,double rDirCos[3][3]);
// Direction Cosines / Quaternions
void STDCALL SDDC2QUAT(double aDirCos[3][3],
								 double *rQ1,double *rQ2,double *rQ3,double *rQ4);
void sddc2quat_(double aDirCos[3][3],
					double *rQ1,double *rQ2,double *rQ3,double *rQ4);
void sddc2quat(double aDirCos[3][3],
				  double *rQ1,double *rQ2,double *rQ3,double *rQ4);
void STDCALL SDQUAT2DC(double *aQ1,double *aQ2,double *aQ3,double *aQ4,
								 double rDirCos[3][3]);
void sdquat2dc_(double *aQ1,double *aQ2,double *aQ3,double *aQ4,
					double rDirCos[3][3]);
void sdquat2dc(double aQ1,double aQ2,double aQ3,double aQ4,
				  double rDirCos[3][3]);

// TOPOLOGY
void STDCALL SDBTJ(int *body,double btj[3]);
void sdbtj_(int *body,double btj[3]);
void sdbtj(int body,double btj[3]);
void STDCALL SDGETBTJ(int *body,double btj[3]);
void sdgetbtj_(int *body,double btj[3]);
void sdgetbtj(int body,double btj[3]);
void STDCALL SDITJ(int *body,double itj[3]);
void sditj_(int *body,double itj[3]);
void sditj(int body,double itj[3]);
void STDCALL SDGETITJ(int *body,double itj[3]);
void sdgetitj_(int *body,double itj[3]);
void sdgetitj(int body,double itj[3]);
void STDCALL SDPIN(int *body,int *pinNumber,double pin[3]);
void sdpin_(int *body,int *pinNumber,double pin[3]);
void sdpin(int body,int pinNumber,double pin[3]);
void STDCALL SDGETPIN(int *body,int *pinNumber,double pin[3]);
void sdgetpin_(int *body,int *pinNumber,double pin[3]);
void sdgetpin(int body,int pinNumber,double pin[3]);
void STDCALL SDINFO(int info[50]);
void sdinfo_(int info[50]);
void sdinfo(int info[50]);
void STDCALL SDJNT(int *joint,int info[50],int slider[6]);
void sdjnt_(int *joint,int info[50],int slider[6]);
void sdjnt(int joint,int info[50],int slider[6]);

// INERTIAL PARAMETERS
void STDCALL SDSYS(double *aM,double rCOM[3],double rI[3][3]);
void sdsys_(double *aM,double rCOM[3],double rI[3][3]);
void sdsys(double *aM,double rCOM[3],double rI[3][3]);

void STDCALL SDMASS(int *aBody,double *aMass);
void sdmass_(int *aBody,double *aMass);
void sdmass(int aBody,double aMass);

void STDCALL SDGETMASS(int *aBody,double *rMass);
void sdgetmass_(int *aBody,double *rMass);
void sdgetmass(int aBody,double *rMass);

void STDCALL SDINER(int *aBody,double aInertia[3][3]);
void sdiner_(int *aBody,double aInertia[3][3]);
void sdiner(int aBody,double aInertia[3][3]);

void STDCALL SDGETINER(int *aBody,double rInertia[3][3]);
void sdgetiner_(int *aBody,double rInertia[3][3]);
void sdgetiner(int aBody,double rInertia[3][3]);

// ERROR
void sderror(int *rRoutine,int *rErrNo);
void sdprinterr(FILE *aFP);
void sdclearerr();

// USER WRITTEN ROUTINES


#ifdef __cplusplus
}
#endif
