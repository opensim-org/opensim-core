// SDFastFWin.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "SDFastDLL.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <OpenSim/Tools/rdMath.h>
#include "sdfast.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "SDFast.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
const int SDFast::GROUND = 0;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SDFast::SDFast()
{
	// NUMBERS OF THINGS
	_nx = 0;
	_nq = 0;
	_nu = 0;
	_ny = 0;
	_nyi = 0;
	_nj = 0;
	_nb = 0;


	// BODY NAMES


	// COORDINATE TO JOINT AND AXIS MAP

}
//_____________________________________________________________________________
/**
 * Destructor.
 */
SDFast::~SDFast()
{

	// COORDINATE MAPS
	if(_u2jMap!=NULL) { delete[] _u2jMap;  _u2jMap=NULL; }
	if(_u2aMap!=NULL) { delete[] _u2aMap;  _u2aMap=NULL; }

}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct maps from coordinate to joint and axis.
 * These maps should be set up using sdinfo.
 */
int SDFast::constructCoordinateMaps()
{
	_u2jMap = NULL;
	_u2aMap = NULL;

	return(0);
}



//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// KINEMATICS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the inertial position of a point on a body.
 */
void SDFast::
getPosition(int aBody,double aPoint[3],double aPos[3])
{
	SDPOS(&aBody,aPoint,aPos);
}
//_____________________________________________________________________________
/**
 * Get the inertial velocity of a point on a body.
 */
void SDFast::
getVelocity(int aBody,double aPoint[3],double aVel[3])
{
	SDVEL(&aBody,aPoint,aVel);
}
//_____________________________________________________________________________
/**
 * Get the inertial acceleration of a point on a body.
 */
void SDFast::
getAcceleration(int aBody,double aPoint[3],double aAcc[3])
{
	SDACC(&aBody,aPoint,aAcc);
}

//-----------------------------------------------------------------------------
// BODY KINEMATICS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the orientation of a body with respect to the inertial frame in
 * terms of direction cosines.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Body ID.
 * @param rDC Direction cosines.
 *
 * @see setConfiguration()
 */
void SDFast::
getDirectionCosines(int aBody,double rDC[3][3])
{
	SDORIENT(&aBody,rDC);
}
//_____________________________________________________________________________
/**
 * Get the angular velocity of a body with respect to the inertial frame
 * expressed in the body-local frame.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Body ID.
 * @param rAngVel Angular velocity.
 *
 * @see setConfiguration()
 */
void SDFast::
getAngularVelocityBodyLocal(int aBody,double rAngVel[3])
{
	SDANGVEL(&aBody,rAngVel);
}
//_____________________________________________________________________________
/**
 * Get the angular acceleration of a body with respect to the inertial frame
 * expressed in the body-local frame.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Body ID.
 * @param rAngAcc Angular acceleration.
 *
 * @see setConfiguration()
 */
void SDFast::
getAngularAccelerationBodyLocal(int aBody,double rAngAcc[3])
{
	SDANGACC(&aBody,rAngAcc);
}

//-----------------------------------------------------------------------------
// GRAVITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the gravity.
 */
void SDFast::
setGravity(double aG[3])
{
	Model::setGravity(aG);
	SDGRAV(aG);
	SDINIT();
}
//_____________________________________________________________________________
/**
 * Get the gravity.
 */
void SDFast::
getGravity(double aG[3])
{
	SDGETGRAV(aG);
}

//-----------------------------------------------------------------------------
// INERTIAL PARAMETERS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the mass, center of mass, and inertia tensor of the whole model.
 */
void SDFast::
getSystemInertia(double *aM,double aCOM[3],double aI[3][3])
{
	SDSYS(aM,aCOM,aI);
}


//=============================================================================
// CONFIGURATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the configuration of the model.
 */
void SDFast::
setConfiguration(const double aQ[],const double aU[])
{
	// IN BASE CLASS
	Model::setConfiguration(aQ,aU);

	// IN SDFAST
	SDSTATE(&_t,_q,_u);
}
//_____________________________________________________________________________
/**
 * Set the configuration of the model.
 */
void SDFast::
setConfiguration(const double aY[])
{
	// BASE CLASS
	Model::setConfiguration(aY);

	// SDFAST
	SDSTATE(&_t,_q,_u);
}


//=============================================================================
// DERIVATIVES
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the derivatives of the states.
 */
int SDFast::
computeAccelerations(double *dqdt,double *dudt)
{
	SDDERIV(dqdt,dudt);
	return(0);
}


//=============================================================================
// APPLY LOADS
//=============================================================================
//-----------------------------------------------------------------------------
// FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a force to a body aBody at a point aPoint.
 *
 * The body point and force should be expressed in the body-local frame.
 */
void SDFast::
applyForceBodyLocal(int aBody,double aPoint[3],double aForce[3])
{
	SDPOINTF(&aBody,aPoint,aForce);
}
//_____________________________________________________________________________
/**
 * Apply a set of forces to a set of bodies.
 *
 * The body points and forces should be expressed in the body-local frame.
 */
void SDFast::
applyForcesBodyLocal(int aN,int aBodies[],double aPoints[][3],double aForces[][3])
{
	int i;
	for(i=0;i<aN;i++) {
		applyForceBodyLocal(aBodies[i],aPoints[i],aForces[i]);
	}
}

//-----------------------------------------------------------------------------
// GLOBAL FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a force expressed in the global frame to a body at a point.
 *
 * The body point, aPoint, should be expressed in the body-local frame.
 * The force, aForce, should be expressed in the global (inertial) frame.
 */
void SDFast::
applyForce(int aBody,double aPoint[3],double aForce[3])
{
	int ground = GROUND;
	double force[3];
	SDTRANS(&ground,aForce,&aBody,force);
	SDPOINTF(&aBody,aPoint,force);
}
//_____________________________________________________________________________
/**
 * Apply forces expressed in the global frame to bodies.
 *
 * The body points, aPoint, should be expressed in the body-local frame.
 * The forces, aForce, should be expressed in the global (inertial) frame.
 */
void SDFast::
applyForces(int aN,int aBodies[],double aPoints[][3],double aForces[][3])
{
	int i;
	for(i=0;i<aN;i++) {
		applyForce(aBodies[i],aPoints[i],aForces[i]);
	}
}
//_____________________________________________________________________________
/**
 * Apply forces expressed in the inertial frame to bodies.
 *
 * @param aN Number of forces.
 * @param aBodes Array of body ID's.
 * @param aPoints Pointer to a sequence of points laid out as aPoints[aN][3].
 * @param aForces Pointer to a sequence of forces laid out as aForces[aN][3].
 */
void SDFast::
applyForces(int aN,int aBodies[],double *aPoints,double *aForces)
{
	int i,I;
	for(i=0;i<aN;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		applyForce(aBodies[i],&aPoints[I],&aForces[I]);
	}
}

//-----------------------------------------------------------------------------
// TORQUES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a torque expressed in the body-local to a body.
 *
 * @param aBody Body ID.
 * @param aTorque Torque expressed in the body-local frame.
 */
void SDFast::
applyTorqueBodyLocal(int aBody,double aTorque[3])
{
	SDBODYT(&aBody,aTorque);
}

//-----------------------------------------------------------------------------
// GLOBAL TORQUES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a torque expressed in the inertial frame to a body.
 *
 * @param aBody Body ID.
 * @param aTorque Torque expressed in the inertial frame.
 */
void SDFast::
applyTorque(int aBody,double aTorque[3])
{
	int ground = GROUND;
	double torque[3];
	SDTRANS(&ground,aTorque,&aBody,torque);
	SDBODYT(&aBody,torque);
}

//-----------------------------------------------------------------------------
// GENERALIZED FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a generalized force to a generalized coordinate.
 * Note that depending on the axis type the generalized force can be a
 * torque or a force.
 */
void SDFast::
applyGeneralizedForce(int aU,double aF)
{
	int joint = _u2jMap[aU];
	int axis = _u2aMap[aU];
	SDHINGET(&joint,&axis,&aF);
}
//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * The dimension of aF is assumed to be the number of generalized speeds.
 */
void SDFast::
applyGeneralizedForces(double aF[])
{
	int i;
	for(i=0;i<_nu;i++) applyGeneralizedForce(i,aF[i]);
}
//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * The dimension of aF is assumed to be the number of generalized speeds.
 */
void SDFast::
applyGeneralizedForces(int aN,int aU[],double aF[])
{
	int i;
	for(i=0;i<aN;i++) applyGeneralizedForce(aU[i],aF[i]);
}


//=============================================================================
// EQUATIONS OF MOTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Form the system mass matrix.
 *
 * rI is a square matrix of size NU*NU.
 */
void SDFast::
formMassMatrix(double *rI)
{
	SDMASSMAT(rI);
}
//_____________________________________________________________________________
/**
 * Form the transformation matrix E[3][3] that can be used to express the
 * angular velocity of a body in terms of the time derivatives of the euler
 * angles.  The Euler angle convention is body-fixed 1-2-3.
 */
void SDFast::
formEulerTransform(int aBody,double *rE)
{
	if(rE==NULL) return;

	// GET ORIENTATION OF aBody
	double ang[3], dc[3][3];
	SDORIENT(&aBody,dc);	
	SDDC2ANG(dc,&ang[0],&ang[1],&ang[2]);

	// ROW 1
	*(rE++) = cos(ang[2])/cos(ang[1]);
	*(rE++) = -sin(ang[2])/cos(ang[1]);
	*(rE++) = 0.0;

	// ROW 2
	*(rE++) = sin(ang[2]);
	*(rE++) = cos(ang[2]);
	*(rE++) = 0.0;

	// ROW 3
	*(rE++) = -cos(ang[2])*sin(ang[1])/cos(ang[1]);
	*(rE++) = sin(ang[1])*sin(ang[2])/cos(ang[1]);
	*(rE)   = 1.0;
}
//_____________________________________________________________________________
/**
 * Form the full angular velocity Jacobian matrix (J0) for a point on a body.
 *
 * Note that J0 is not appropriate for operations on the body when the body
 * orientation is specified in terms of Euler angles.  When the body
 * is described in terms of Euler angles, the method formJ should be used.
 *
 * J0 is laid out as follows:
 *		dPx/dq1	dPx/dq2	dPx/dq3	...
 *		dPy/dq1	dPy/dq2	dPy/dq3	...
 *		dPz/dq1	dPz/dq2	dPz/dq3	...
 *		dOx/dq1	dOx/dq2	dOx/dq3	...
 *		dOy/dq1	dOy/dq2	dOy/dq3	...
 *		dOz/dq1	dOz/dq2	dOz/dq3	...
 *	where P is the point on the body and O is the orientation of the body.
 *
 * So, J0 should have 6 rows and NU columns.  In memory, the column index
 * increments fastest, so the representation is J0[6][NU].
 *
 * It is assumed that enough space has been allocated at aJ to hold all
 * of the Jacobian elements.
 *
 * The Jacobian elements are expressed in the inertial or ground frame.
 */
void SDFast::
formJacobian(int aBody,double aPoint[3],double *rJ0)
{
	if(rJ0==NULL) return;

	int i,iu,j,I;
	int ground = GROUND;
	double trans[3],orien[3];
	for(i=0;i<getNU();i++) {

		// GET COLUMN
		iu = i + 1;
		SDREL2CART(&iu,&aBody,aPoint,trans,orien);

		// TRANSFORM TO GROUND FRAME
		SDTRANS(&aBody,trans,&ground,trans);
		SDTRANS(&aBody,orien,&ground,orien);

		// FORM MATRIX
		for(j=0;j<3;j++) {
			I = Mtx::ComputeIndex(j,_nu,i);
			rJ0[I] = trans[j];
			I = Mtx::ComputeIndex(j+3,_nu,i);
			rJ0[I] = orien[j];
		}
	}

	// PRINT
	//Mtx::Print(6,getNU(),rJ0,3);
}
//_____________________________________________________________________________
/**
 * Form the full Jacobian matrix (J) for a point on a body.
 *
 * Note that J is appropriate for operations on the body when the body
 * orientation is specified in terms of Euler angles.
 *
 * J is laid out as follows:
 *		dPx/dq1	dPx/dq2	dPx/dq3	...
 *		dPy/dq1	dPy/dq2	dPy/dq3	...
 *		dPz/dq1	dPz/dq2	dPz/dq3	...
 *		dOx/dq1	dOx/dq2	dOx/dq3	...
 *		dOy/dq1	dOy/dq2	dOy/dq3	...
 *		dOz/dq1	dOz/dq2	dOz/dq3	...
 *	where P is the point on the body and O is the orientation of the body.
 *
 * So, J should have 6 rows and NU columns.  In memory, the column index
 * increments fastest, so the representation is aJ[6][NU].
 *
 * It is assumed that enough space has been allocated at aJ to hold all
 * of the Jacobian elements.
 *
 * The Jacobian elements are expressed in the inertial or ground frame.
 */
void SDFast::
formJacobianEuler(int aBody,double aPoint[3],double *rJ)
{
	if(rJ==NULL) return;

	// FORM J0
	formJacobian(aBody,aPoint,rJ);

	// FORM E
	double E[3][3];
	formEulerTransform(aBody,&E[0][0]);
	printf("\nSDFast.formJacobianEuler:\n");
	Mtx::Print(3,3,&E[0][0],3);

	// TRANSFORM J0 TO J
	int I = Mtx::ComputeIndex(3,getNU(),0);
	Mtx::Multiply(3,3,getNU(),&E[0][0],&rJ[I],&rJ[I]);

	// PRINT
	//Mtx::Print(6,getNU(),rJ,3);
}
//_____________________________________________________________________________
/**
 * Form the generalized inverse of the Jacobian matrix, rJInv.
 *
 * rJInv has the shape rJInv[NU][6];
 *
 * rJInv = Inverse(aI) * aJT * Inverse( aJ * Inverse(aI) * JT)
 *
 * @param aI A pointer to the system mass matrix.
 * @param aJ A pointer to the system Jacobian.
 * @param aJInv A pointer to the generalized inverse of the Jacobian.
 * 
 * @return 0 when successful, -1 if an error is encountered, -2 when the
 * generalized inverse has a singularity.
 */
int SDFast::
formJacobianInverse(double *aI,double *aJ,double *rJInv)
{
	if(aI==NULL) return(-1);
	if(aJ==NULL) return(-1);
	if(rJInv==NULL) return(-1);

	// INVERSE OF THE MASS MATRIX
	int nu = getNU();
	double *IInv = new double[nu*nu];
	Mtx::Invert(nu,aI,IInv);
	//printf("\nSDFast.formJacobianInverse: Inverse Mass Matrix...\n");
	//Mtx::Print(nu,nu,IInv,3);

	// FORM JT (NUx6)
	double *JT = new double[nu*6];
	Mtx::Transpose(6,nu,aJ,JT);
	//printf("\nSDFast.formJacobianInverse: JT...\n");
	//Mtx::Print(nu,6,JT,3);

	// COMPUTE IInv * JT (NUx6)
	double *IInv_JT = new double[nu*6];
	Mtx::Multiply(nu,nu,6,IInv,JT,IInv_JT);
	//printf("\nSDFast.formJacobianInverse: IInv_JT...\n");
	//Mtx::Print(nu,6,IInv_JT,3);

	// COMPUTE Inverse(J * IInv * JT) (6x6)
	double J_IInv_JT[6*6];
	Mtx::Multiply(6,nu,6,aJ,IInv_JT,J_IInv_JT);
	//printf("\nSDFast.formJacobianInverse: J_IInv_JT...\n");
	//Mtx::Print(6,6,J_IInv_JT,3);
	int status = Mtx::Invert(6,J_IInv_JT,J_IInv_JT);
	//printf("\nSDFast.formJacobianInverse: Inverse(J_IInv_JT)...\n");
	//Mtx::Print(6,6,J_IInv_JT,3);

	// COMPUTE GENERALIZED INVERSE (NUx6)
	Mtx::Multiply(nu,6,6,JT,J_IInv_JT,rJInv);
	Mtx::Multiply(nu,nu,6,IInv,rJInv,rJInv);

	// CLEANUP
	if(IInv!=NULL) delete[] IInv;
	if(JT!=NULL) delete[] JT;
	if(IInv_JT!=NULL) delete[] IInv_JT;

	// PRINT STATUS
	if(status==-2) {
		printf("\nSDFast.formJcobianInverse: WARN- singularity.\n");
	}
	//printf("\nSDFast.formJacobianInverse:  Generalized Inverse...\n");
	//Mtx::Print(nu,6,rJInv,3);

	return(status);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Convert quaterions to angles.
 */
void SDFast::
convertQuaternionsToAngles(double *aQ,double *rQAng)
{
	SDST2ANG(aQ,rQAng);
}
//_____________________________________________________________________________
/**
 * Convert angles to quaterions.
 */
void SDFast::
convertAnglesToQuaternions(double *aQAng,double *rQ)
{
	SDANG2ST(aQAng,rQ);
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to Euler angles.
 * The Euler angle convention is body-fixed 1-2-3.
 *
 * @param aDirCos Matrix of direction cosines.
 * @param rE1 Euler angle about axis 1.
 * @param rE2 Euler angle about axis 2.
 * @param rE3 Euler angle about axis 3.
 */
void SDFast::
convertDirectionCosinesToAngles(double aDirCos[3][3],
	double *rE1,double *rE2,double *rE3)
{
	SDDC2ANG(aDirCos,rE1,rE2,rE3);
}
