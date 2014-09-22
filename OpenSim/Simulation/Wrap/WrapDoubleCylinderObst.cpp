/* -------------------------------------------------------------------------- *
 *                    OpenSim:  WrapDoubleCylinderObst.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Brian Garner, Peter Loan                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include "WrapDoubleCylinderObst.h"
#include <SimTKcommon.h>
#include <OpenSim/Simulation/Model/PathPoint.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include <OpenSim/Simulation/Wrap/WrapResult.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/Mtx.h>
#include <sstream>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const char* wrapTypeName = "doubleCylinderObst";

static const double TwoPi = 2.0*SimTK::Pi;
// static const double max_wrap_pts_circle_ang = (5.0/360.0)*TwoPi;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
WrapDoubleCylinderObst::WrapDoubleCylinderObst() :
WrapObject(),
_radiusUcyl(_radiusUcylProp.getValueDbl()),
_radiusVcyl(_radiusVcylProp.getValueDbl()),
_wrapUcylDirectionName(_wrapUcylDirectionNameProp.getValueStr()),
_wrapVcylDirectionName(_wrapVcylDirectionNameProp.getValueStr()),
_wrapVcylHomeBodyName(_wrapVcylHomeBodyNameProp.getValueStr()),
_xyzBodyRotationVcyl(_xyzBodyRotationVcylProp.getValueDblArray()),
_translationVcyl(_translationVcylProp.getValueDblVec()),
_length(_lengthProp.getValueDbl()),
_activeState(0)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
* Destructor.
*/
WrapDoubleCylinderObst::~WrapDoubleCylinderObst()
{
}

//_____________________________________________________________________________
/**
* Copy constructor.
*
* @param aWrapDoubleCylinderObst WrapDoubleCylinderObst to be copied.
*/
WrapDoubleCylinderObst::WrapDoubleCylinderObst(const WrapDoubleCylinderObst& aWrapDoubleCylinderObst) :
WrapObject(aWrapDoubleCylinderObst),
_radiusUcyl(_radiusUcylProp.getValueDbl()),
_radiusVcyl(_radiusVcylProp.getValueDbl()),
_wrapUcylDirectionName(_wrapUcylDirectionNameProp.getValueStr()),
_wrapVcylDirectionName(_wrapVcylDirectionNameProp.getValueStr()),
_wrapVcylHomeBodyName(_wrapVcylHomeBodyNameProp.getValueStr()),
_xyzBodyRotationVcyl(_xyzBodyRotationVcylProp.getValueDblArray()),
_translationVcyl(_translationVcylProp.getValueDblVec()),
_length(_lengthProp.getValueDbl()),
_activeState(0)
{
	setNull();
	setupProperties();
	copyData(aWrapDoubleCylinderObst);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Set the data members of this WrapCylinder to their null values.
*/
void WrapDoubleCylinderObst::setNull()
{
	_wrapUcylDirection = righthand;
	_wrapVcylDirection = righthand;
}

//_____________________________________________________________________________
/**
* Connect properties to local pointers.
*/
void WrapDoubleCylinderObst::setupProperties()
{
	// BASE CLASS
	WrapObject::setupProperties();

	_radiusUcylProp.setName("radiusUcyl");	_radiusUcylProp.setValue(-1.0);		_propertySet.append(&_radiusUcylProp);
	_radiusVcylProp.setName("radiusVcyl");	_radiusVcylProp.setValue(-1.0);		_propertySet.append(&_radiusVcylProp);

	_wrapUcylDirectionNameProp.setName("wrapUcylDirection");	_wrapUcylDirectionNameProp.setValue("Unassigned");	_propertySet.append(&_wrapUcylDirectionNameProp);
	_wrapVcylDirectionNameProp.setName("wrapVcylDirection");	_wrapVcylDirectionNameProp.setValue("Unassigned");	_propertySet.append(&_wrapVcylDirectionNameProp);
	_wrapVcylHomeBodyNameProp.setName("wrapVcylHomeBody");	_wrapVcylHomeBodyNameProp.setValue("Unassigned");	_propertySet.append(&_wrapVcylHomeBodyNameProp);

	const double defaultRotations[] = {0.0, 0.0, 0.0};
	_xyzBodyRotationVcylProp.setName("xyz_body_rotationVcyl");
	_xyzBodyRotationVcylProp.setValue(3, defaultRotations);
	_propertySet.append(&_xyzBodyRotationVcylProp);

	const SimTK::Vec3 defaultTranslations(0.0);
	_translationVcylProp.setName("translationVcyl");
	_translationVcylProp.setValue(defaultTranslations);
	//_translationProp.setAllowableListSize(3);
	_propertySet.append(&_translationVcylProp);

	_lengthProp.setName("length");
	_lengthProp.setValue(1.0);
	_propertySet.append(&_lengthProp);
}

//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param aModel simbody model
*/
void WrapDoubleCylinderObst::connectToModelAndBody(Model& aModel, OpenSim::Body& aBody)
{
	// Base class
	Super::connectToModelAndBody(aModel, aBody);

	// maybe set a parent pointer, _body = aBody;
	if ( (_radiusUcyl<0.0) || (_radiusVcyl < 0.0) )
	{
		string errorMessage = "Error: radii for WrapDoubleCylinderObst " + getName() + " were either not specified, or one or more is negative.";
		throw Exception(errorMessage);
	}

	AnalyticCylinder* cyl = new AnalyticCylinder(_radiusUcyl, _length);
	setGeometryQuadrants(cyl);

	// Check wrapUcylDirectionName
	if (_wrapUcylDirectionName == "righthand" || _wrapUcylDirectionName == "right" || _wrapUcylDirectionName == "righthanded" || _wrapUcylDirectionName == "Righthand" || _wrapUcylDirectionName == "Right" || _wrapUcylDirectionName == "Righthanded")
		_wrapUcylDirection = righthand;
	else
	if (_wrapUcylDirectionName == "lefthand"  || _wrapUcylDirectionName == "left"  || _wrapUcylDirectionName == "lefthanded"  || _wrapUcylDirectionName == "Lefthand"  || _wrapUcylDirectionName == "Left"  || _wrapUcylDirectionName == "Lefthanded")
		_wrapUcylDirection = lefthand;
	else
	if (_wrapUcylDirectionName == "Unassigned")  // wrapDirection was not specified in obstacle object definition; use default
		{ _wrapUcylDirection = righthand;	_wrapUcylDirectionName = "righthand";	}
	else {  // wrapUcylDirection was specified incorrectly in obstacle object definition; throw an exception
		string errorMessage = "Error: wrapUcylDirection for wrap obstacle " + getName() + " was specified incorrectly.  Use \"righthand\" or \"lefthand\".";
		throw Exception(errorMessage);
	}

	// Check wrapVcylDirectionName
	if (_wrapVcylDirectionName == "righthand" || _wrapVcylDirectionName == "right" || _wrapVcylDirectionName == "righthanded" || _wrapVcylDirectionName == "Righthand" || _wrapVcylDirectionName == "Right" || _wrapVcylDirectionName == "Righthanded")
		_wrapVcylDirection = righthand;
	else
	if (_wrapVcylDirectionName == "lefthand"  || _wrapVcylDirectionName == "left"  || _wrapVcylDirectionName == "lefthanded"  || _wrapVcylDirectionName == "Lefthand"  || _wrapVcylDirectionName == "Left"  || _wrapVcylDirectionName == "Lefthanded")
		_wrapVcylDirection = lefthand;
	else
	if (_wrapVcylDirectionName == "Unassigned")  // wrapDirection was not specified in obstacle object definition; use default
		{ _wrapVcylDirection = righthand;	_wrapVcylDirectionName = "righthand";	}
	else {  // wrapVcylDirection was specified incorrectly in obstacle object definition; throw an exception
		string errorMessage = "Error: wrapVcylDirection for wrap obstacle " + getName() + " was specified incorrectly.  Use \"righthand\" or \"lefthand\".";
		throw Exception(errorMessage);
	}
	
	// Initialize value of _activeState
	_activeState = 3;	// By default assume both cylinders are active
	_wrapUcylHomeBody = &aBody;	// Save this for use in wrapLine
	_model = &aModel;	// Save this for use in wrapLine

	// Obtain wrapVcylHomeBody based on wrapVcylHomeBodyName
	if(!aModel.updBodySet().contains(_wrapVcylHomeBodyName)) {
		string errorMessage = "Error: wrapVcylHomeBody " + _wrapVcylHomeBodyName + " for wrap obstacle " + getName() + " was not found in model.";
		throw Exception(errorMessage);
	}
	_wrapVcylHomeBody = &aModel.updBodySet().get(_wrapVcylHomeBodyName);
	
}

//_____________________________________________________________________________
/**
* Copy data members from one WrapDoubleCylinderObst to another.
*
* @param aWrapDoubleCylinderObst WrapDoubleCylinderObst to be copied.
*/
void WrapDoubleCylinderObst::copyData(const WrapDoubleCylinderObst& aWrapDoubleCylinderObst)
{
	// BASE CLASS
	WrapObject::copyData(aWrapDoubleCylinderObst);

	_radiusUcyl = aWrapDoubleCylinderObst._radiusUcyl;
	_radiusVcyl = aWrapDoubleCylinderObst._radiusVcyl;
	_wrapUcylDirectionName = aWrapDoubleCylinderObst._wrapUcylDirectionName;
	_wrapVcylDirectionName = aWrapDoubleCylinderObst._wrapVcylDirectionName;
	_wrapUcylDirection = aWrapDoubleCylinderObst._wrapUcylDirection;
	_wrapVcylDirection = aWrapDoubleCylinderObst._wrapVcylDirection;
	_wrapVcylHomeBodyName = aWrapDoubleCylinderObst._wrapVcylHomeBodyName;
	_wrapVcylHomeBody = aWrapDoubleCylinderObst._wrapVcylHomeBody;
	_xyzBodyRotationVcyl = aWrapDoubleCylinderObst._xyzBodyRotationVcyl;
	_translationVcyl = aWrapDoubleCylinderObst._translationVcyl;
	_length = aWrapDoubleCylinderObst._length;
	_activeState = aWrapDoubleCylinderObst._activeState;
}

//_____________________________________________________________________________
/**
 * Get the name of the type of wrap object ("cylinderObst" in this case)
 *
 * @return A string representing the type of wrap object
 */
const char* WrapDoubleCylinderObst::getWrapTypeName() const
{
	return wrapTypeName;
}

//_____________________________________________________________________________
/**
 * Get a string holding the dimensions definition that SIMM would
 * use to describe this object. This is a rather ugly convenience
 * function for outputting SIMM joint files.
 *
 * @return A string containing the dimensions of the wrap object
 */
string WrapDoubleCylinderObst::getDimensionsString() const
{
	stringstream dimensions;
	dimensions << "radius " << _radiusUcyl << "\nheight " << _length;

	return dimensions.str();
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
* Assignment operator.
*
* @return Reference to this object.
*/
WrapDoubleCylinderObst& WrapDoubleCylinderObst::operator=(const WrapDoubleCylinderObst& aWrapDoubleCylinderObst)
{
	// BASE CLASS
	WrapObject::operator=(aWrapDoubleCylinderObst);

	return(*this);
}






/*============================================================================*/
#define OBST_BUFF	0.0001	/* BUFFER BETWEEN OBST SURFACE AND POINTS (IN mm) */
#define ABS(A) ( (A)>0 ? (A) : (-(A)) )
#define SIGN(A) ( (A)>=0 ? (1) : (-1) )
#define ZERO(A) ( ABS(A)<1.0e-8 ? (1) : (0) )
#define DISTANCE_BETWEEN(P,S)						\
	(												\
		sqrt(	(P[0]-S[0])*(P[0]-S[0]) +			\
				(P[1]-S[1])*(P[1]-S[1]) +			\
				(P[2]-S[2])*(P[2]-S[2])   )			\
	)
/*===========================================================================*/
static int quick_add_vec_to_vec(double *A,double *B,double *C) {
	C[0] = A[0] + B[0];
	C[1] = A[1] + B[1];
	C[2] = A[2] + B[2];
	return(0);
}
/*===========================================================================*/
static int quick_sub_vec_fm_vec(double *A,double *B,double *C) {
	C[0] = A[0] - B[0];
	C[1] = A[1] - B[1];
	C[2] = A[2] - B[2];
	return(0);
}
/*===========================================================================*/
static int quick_mul_vec_by_mtx(double *V,double *R,double *C) {
	double x,y,z;
	x = R[0]*V[0] + R[1]*V[1] + R[2]*V[2];
	y = R[3]*V[0] + R[4]*V[1] + R[5]*V[2];
	z = R[6]*V[0] + R[7]*V[1] + R[8]*V[2];
	C[0]=x;	C[1]=y;	C[2]=z;
	return(0);
}
/*===========================================================================*/
static int quick_mul_vec_by_mtxT(double *V,double *R,double *C) {
	double x,y,z;
	x = R[0]*V[0] + R[3]*V[1] + R[6]*V[2];
	y = R[1]*V[0] + R[4]*V[1] + R[7]*V[2];
	z = R[2]*V[0] + R[5]*V[1] + R[8]*V[2];
	C[0]=x;	C[1]=y;	C[2]=z;
	return(0);
}
/*===========================================================================*/
static int quick_mul_mtx_by_mtx(double *A,double *B,double *C) {
	C[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
	C[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
	C[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];
	C[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
	C[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
	C[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];
	C[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
	C[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
	C[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];
	return(0);
}
/*===========================================================================*/
static int quick_mul_mtx_by_mtxT(double *A,double *B,double *C) {
	C[0] = A[0]*B[0] + A[1]*B[1] + A[2]*B[2];
	C[1] = A[0]*B[3] + A[1]*B[4] + A[2]*B[5];
	C[2] = A[0]*B[6] + A[1]*B[7] + A[2]*B[8];
	C[3] = A[3]*B[0] + A[4]*B[1] + A[5]*B[2];
	C[4] = A[3]*B[3] + A[4]*B[4] + A[5]*B[5];
	C[5] = A[3]*B[6] + A[4]*B[7] + A[5]*B[8];
	C[6] = A[6]*B[0] + A[7]*B[1] + A[8]*B[2];
	C[7] = A[6]*B[3] + A[7]*B[4] + A[8]*B[5];
	C[8] = A[6]*B[6] + A[7]*B[7] + A[8]*B[8];
	return(0);
}
/*===========================================================================*/
static int quick_mul_mtxT_by_mtx(double *A,double *B,double *C) {
	C[0] = A[0]*B[0] + A[3]*B[3] + A[6]*B[6];
	C[1] = A[0]*B[1] + A[3]*B[4] + A[6]*B[7];
	C[2] = A[0]*B[2] + A[3]*B[5] + A[6]*B[8];
	C[3] = A[1]*B[0] + A[4]*B[3] + A[7]*B[6];
	C[4] = A[1]*B[1] + A[4]*B[4] + A[7]*B[7];
	C[5] = A[1]*B[2] + A[4]*B[5] + A[7]*B[8];
	C[6] = A[2]*B[0] + A[5]*B[3] + A[8]*B[6];
	C[7] = A[2]*B[1] + A[5]*B[4] + A[8]*B[7];
	C[8] = A[2]*B[2] + A[5]*B[5] + A[8]*B[8];
	return(0);
}
/*===========================================================================*/
static int load_Rx(double q,double R[9]) {
	double cn,sn;
	cn = cos(q);	sn = sin(q);
	R[0] = 1.0;		R[1] = 0.0;		R[2] = 0.0;
	R[3] = 0.0;		R[4] =  cn;		R[5] = -sn;
	R[6] = 0.0;		R[7] =  sn;		R[8] =  cn;
	return(0);
}
/*===========================================================================*/
static int load_Ry(double q,double R[9]) {
	double cn,sn;
	cn = cos(q);	sn = sin(q);
	R[0] =  cn;		R[1] = 0.0;		R[2] =  sn;
	R[3] = 0.0;		R[4] = 1.0;		R[5] = 0.0;
	R[6] = -sn;		R[7] = 0.0;		R[8] =  cn;
	return(0);
}
/*===========================================================================*/
static int load_Rz(double q,double R[9]) {
	double cn,sn;
	cn = cos(q);	sn = sin(q);
	R[0] =  cn;		R[1] = -sn;		R[2] = 0.0;
	R[3] =  sn;		R[4] =  cn;		R[5] = 0.0;
	R[6] = 0.0;		R[7] = 0.0;		R[8] = 1.0;
	return(0);
}
/*===========================================================================*/
static int load_Rxyz(double q[3],double Rxyz[9]) {
	double Rx[9],Ry[9],Rz[9],Rxy[9];
	load_Rx(q[0],Rx); load_Ry(q[1],Ry); load_Rz(q[2],Rz);
	quick_mul_mtx_by_mtx(Rx,Ry,Rxy);
	quick_mul_mtx_by_mtx(Rxy,Rz,Rxyz);
	return(0);
}
/*===========================================================================*/
/*====== SOLVE THE SYSTEM OF LINEAR EQUATIONS:  A(NxN)*X(Nx1)=B(Nx1) ========*/
/*===========================================================================*/
static int quick_solve_linear(int N,double A[],double X[],double B[]) {
	static double *MTX=NULL,**Mtx=NULL;
	static int mtxsize=0;
	double **Mr,*Mrj,*Mij,*Xr,*Br,d;
	int r,i,j,n;

	/*====================================================================*/
	/*======= ALLOCATE STORAGE FOR DUPLICATE OF A AND ROW POINTERS =======*/
	/*====================================================================*/
	if(mtxsize<N*(N+1)) {	mtxsize=N*(N+1);
		MTX=(double *)  malloc(mtxsize*sizeof(double));
		Mtx=(double **) malloc(N*sizeof(double *));
		if(Mtx==NULL) { fprintf(stderr,"\nOut of memory.\n\n"); exit(0); }
	}
	/*====================================================================*/

	/*====================================================================*/
	/*== COPY A INTO MTX(NxN), B INTO MTX(N+1), AND LOAD POINTER VECTOR ==*/
	/*====================================================================*/
	for(r=0,Mrj=MTX,Mij=A,Br=B;r<N;r++) {
		for(j=0;j<N;j++) *(Mrj++) = *(Mij++);	*(Mrj++) = *(Br++); }
	for(r=0,Mr=Mtx;r<N;r++) *(Mr++)=MTX+r*(N+1);
	/*====================================================================*/

	/*====================================================================*/
	/*======= REDUCE MTX TO UPPER TRIANGULAR USING ROW OPS ON MTX ========*/
	/*====================================================================*/
	for(r=0,n=N-1;r<n;r++) {
		if(ZERO(*(Mrj=Mtx[r]+r))) {	/*==== SWAP ROWS ====*/
			for(i=r+1;i<N;i++) if(!ZERO(*(Mtx[i]+r))) break;
			if(i==N) return(-1);	/*=== NON-INVERTIBLE ===*/
			Mrj=Mtx[r];	Mtx[r]=Mtx[i];	Mtx[i]=Mrj;	Mrj=Mtx[r]+r;
		}
		for(i=r+1;i<N;i++) {
			if(ZERO(*(Mij=Mtx[i]+r))) continue;	Mrj=Mtx[r]+r;
			d = (*Mij)/(*Mrj);	*(Mij++)=0.0;	Mrj++;
			for(j=r+1;j<=N;j++) *(Mij++) -= *(Mrj++)*d;
		}
	}
	/*====================================================================*/

	if(ZERO(*(Mrj=Mtx[r]+r))) return(-1);	/*=== NON-INVERTIBLE ===*/

	/*====================================================================*/
	/*========= BACK PROPAGATE SOLUTION THROUGH TRIANGULAR MATRIX ========*/
	/*====================================================================*/
	n=N-1;	Xr=X+n;
    // The next two lines used to be combined in 1 line:
    //      *Xr = *((Mrj=Mtx[n])+N) / *(Mrj+n);
    // which gave a Wunsequenced warning using Clang, due to the assignment to
    // Mrj.
    // These lines are doing the first step in the back propagation: the last
    // entry of X is the last entry of B divided by the (N, N) element of A.
    Mrj = Mtx[n];
    *Xr = *(Mrj+N) / *(Mrj+n);
	for(r=n-1,Xr--,Mr=Mtx+r;r>=0;r--,Mr--) {
		d = *(*Mr+N);
		for(j=r+1,Mrj= *Mr+j;j<N;j++) d -= *(Mrj++)*X[j];
		*(Xr--) = d / *(*Mr+r);
	}
	/*====================================================================*/

	return(0);
}
/*============================================================================*/







//=============================================================================
// WRAPPING
//=============================================================================

static int double_cylinder(double U[3],double Ru,double V[3],double Rv,double M[9],
   double P[3],double q[3],double Q[3],double T[3],double t[3],double S[3],
   double *Pq,double *qQ,double *QT,double *Tt,double *tS,double *L,
	int *active,double *ru,double *rv);

//_____________________________________________________________________________
/**
 * Calculate the wrapping of one line segment over the cylinder oriented parallel to z-axis.
 *
 * @param aPointP One end of the line segment, already expressed in cylinder frame
 * @param aPointS The other end of the line segment, already expressed in cylinder frame
 * @param aPathWrap An object holding the parameters for this line/cylinder pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @param aFlag A flag for indicating errors, etc.
 * @return The status, as a WrapAction enum
 */
int WrapDoubleCylinderObst::wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
						const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const
{

	double U[3];	U[0]=_translation[0];		U[1]=_translation[1];		U[2]=_translation[2];
	double V[3];	V[0]=_translationVcyl[0];	V[1]=_translationVcyl[1];	V[2]=_translationVcyl[2];
	double Ru = ( _wrapUcylDirection==righthand ? _radiusUcyl : -_radiusUcyl );
	double Rv = ( _wrapVcylDirection==righthand ? _radiusVcyl : -_radiusVcyl );

	double P[3],q[3],Q[3],Pq,qQ,QT,ru;		P[0]=aPoint1[0];	P[1]=aPoint1[1];	P[2]=aPoint1[2];
	double S[3],t[3],T[3],Tt,tS,L,rv;		S[0]=aPoint2[0];	S[1]=aPoint2[1];	S[2]=aPoint2[2];
	
	// CONSTRUCT SOME ROTATION MATRICES
	double VcylObstToUcylObst[9];	// DEFINE M As Rotation Matrix from V-Cylinder to U-Cyllinder Frame
	double xyzBodyRotation[3] = { _xyzBodyRotation[0], _xyzBodyRotation[1], _xyzBodyRotation[2] };
	double xyzBodyRotationVcyl[3] = { _xyzBodyRotationVcyl[0],_xyzBodyRotationVcyl[1],_xyzBodyRotationVcyl[2] };
	double UcylObstToUcylBody[9];	load_Rxyz(xyzBodyRotation,UcylObstToUcylBody);
	double VcylObstToVcylBody[9];	load_Rxyz(xyzBodyRotationVcyl,VcylObstToVcylBody);
	double UcylBodyToGround[9];		_model->getSimbodyEngine().getDirectionCosines(s, *_wrapUcylHomeBody,UcylBodyToGround);
	double VcylBodyToGround[9];		_model->getSimbodyEngine().getDirectionCosines(s, *_wrapVcylHomeBody,VcylBodyToGround);
	double VcylBodyToUcylBody[9];	quick_mul_mtxT_by_mtx(UcylBodyToGround,VcylBodyToGround,VcylBodyToUcylBody);
	double VcylObstToUcylBody[9];	quick_mul_mtx_by_mtx(VcylBodyToUcylBody,VcylObstToVcylBody,VcylObstToUcylBody);
	quick_mul_mtxT_by_mtx(UcylObstToUcylBody,VcylObstToUcylBody,VcylObstToUcylObst);

	double u[3];	// Position of Ucyl center in Vcyl frame;   NOTE:  U is Posn of Ucyl center in Ucyl body frame
	_model->getSimbodyEngine().transformPosition(s, *_wrapUcylHomeBody, U, *_wrapVcylHomeBody, u);
	quick_sub_vec_fm_vec( u, V, u );					// Translate u from Vcyl body to Vcyl obstacle
	quick_mul_vec_by_mtxT( u, VcylObstToVcylBody, u );	// Rotate u into Vcyl obstacle frame

	double v[3];	// Position of Vcyl center in Ucyl frame;   NOTE:  V is Posn of Vcyl center in Vcyl body frame
	_model->getSimbodyEngine().transformPosition(s, *_wrapVcylHomeBody, V, *_wrapUcylHomeBody, v);
	quick_sub_vec_fm_vec( v, U, v );					// Translate v from Ucyl body to Ucyl obstacle
	quick_mul_vec_by_mtxT( v, UcylObstToUcylBody, v );	// Rotate v into Ucyl obstacle frame

	double vs[3],ss[3];	// Position of S in Vcyl obstacle frame;	NOTE:  S is Posn of S in Ucyl obstacle frame
	quick_mul_vec_by_mtx( S, UcylObstToUcylBody, vs );	// Rotate S into Ucyl body frame
	quick_add_vec_to_vec( vs, U, vs );					// Translate s into Ucyl body frame
	_model->getSimbodyEngine().transformPosition(s, *_wrapUcylHomeBody, vs, *_wrapVcylHomeBody, ss);
	quick_sub_vec_fm_vec( ss, V, vs );					// Translate s from Vcyl body to Vcyl obstacle
	quick_mul_vec_by_mtxT( vs, VcylObstToVcylBody, vs );	// Rotate s into Vcyl obstacle frame

	int activeState=(int)aWrapResult.c1[0];	// _activeState;	// TEMP set to zero
	if( (int)aWrapResult.c1[1] != 12345 ) {	aWrapResult.c1[1]=12345.01; activeState=0; }	// Initialize activeState first iteration
	double_cylinder(u,Ru,v,Rv,  VcylObstToUcylObst,  P,q,Q,T,t,vs, &Pq,&qQ,&QT,&Tt,&tS,&L, &activeState, &ru,&rv);
//	_activeState=activeState;		???? I CAN'T SEEM TO SET THIS!!!  GARNER - Needs to be Set in a WrapResult variable.
	aWrapResult.c1[0]=activeState+0.01;	// Save active state for reference in next iteration

	// Transform T back into Ucylinder frame
	quick_mul_vec_by_mtx( T, VcylObstToVcylBody, T );
	quick_add_vec_to_vec( T, V, T );
	_model->getSimbodyEngine().transformPosition(s, *_wrapVcylHomeBody, T, *_wrapUcylHomeBody, T);
	quick_sub_vec_fm_vec( T, U, T );					// Translate T from Ucyl body to Ucyl obstacle
	quick_mul_vec_by_mtxT( T, UcylObstToUcylBody, T );	// Rotate T into Ucyl obstacle frame

	// Transform t back into Ucylinder frame
	quick_mul_vec_by_mtx( t, VcylObstToVcylBody, t );
	quick_add_vec_to_vec( t, V, t );
	_model->getSimbodyEngine().transformPosition(s, *_wrapVcylHomeBody, t, *_wrapUcylHomeBody, t);
	quick_sub_vec_fm_vec( t, U, t );					// Translate t from Ucyl body to Ucyl obstacle
	quick_mul_vec_by_mtxT( t, UcylObstToUcylBody, t );	// Rotate t into Ucyl obstacle frame

	// Initialize return values
	aFlag = false;
	aWrapResult.wrap_path_length = 0.0;
	aWrapResult.wrap_pts.setSize(0);

	// Register results and return
	aFlag = true;
	aWrapResult.wrap_path_length = qQ + QT + Tt;	// PQ + TS + QT;
	aWrapResult.r1[0]=q[0];  aWrapResult.r1[1]=q[1];  aWrapResult.r1[2]=q[2];  
	aWrapResult.r2[0]=t[0];  aWrapResult.r2[1]=t[1];  aWrapResult.r2[2]=t[2];
//	aWrapResult.c1[0]=Q[0];  aWrapResult.c1[1]=Q[1];  aWrapResult.c1[2]=Q[2];  
//	aWrapResult.sv[0]=T[0];  aWrapResult.sv[1]=T[1];  aWrapResult.sv[2]=T[2];
	
	// Generate wrap_pts sequence of points tracing out wrapping path
	aWrapResult.wrap_pts.append(aWrapResult.r1);
//	SimmPoint wppt2(aWrapResult.c1);	aWrapResult.wrap_pts.append(wppt2);
//	SimmPoint wppt3(aWrapResult.sv);	aWrapResult.wrap_pts.append(wppt3);
	aWrapResult.wrap_pts.append(aWrapResult.r2);

	return wrapped;
}





/*============================================================================*/
void WrapDoubleCylinderObst::
getVcylToUcylRotationMatrix(const SimTK::State& s, double VcylObstToUcylObst[9]) const {
	double xyzBodyRotation[3] = { _xyzBodyRotation[0], _xyzBodyRotation[1], _xyzBodyRotation[2] };
	double xyzBodyRotationVcyl[3] = { _xyzBodyRotationVcyl[0],_xyzBodyRotationVcyl[1],_xyzBodyRotationVcyl[2] };
	double UcylBodyToUcylObst[9];	load_Rxyz(xyzBodyRotation,UcylBodyToUcylObst);
	double VcylBodyToVcylObst[9];	load_Rxyz(xyzBodyRotationVcyl,VcylBodyToVcylObst);
	double UcylBodyToGround[9];		_model->getSimbodyEngine().getDirectionCosines(s, *_wrapUcylHomeBody,UcylBodyToGround);
	double VcylBodyToGround[9];		_model->getSimbodyEngine().getDirectionCosines(s, *_wrapVcylHomeBody,VcylBodyToGround);
	double VcylBodyToUcylBody[9];	quick_mul_mtxT_by_mtx(UcylBodyToGround,VcylBodyToGround,VcylBodyToUcylBody);
	double VcylObstToUcylBody[9];	quick_mul_mtx_by_mtxT(VcylBodyToUcylBody,VcylBodyToVcylObst,VcylObstToUcylBody);
	quick_mul_mtx_by_mtx(UcylBodyToUcylObst,VcylObstToUcylBody,VcylObstToUcylObst);
}
/*============================================================================*/

	

/*============================================================================*/
/*==== DETERMINE WHETHER 2D PATH FROM P TO S WOULD COME WITHIN R OF ORIGIN ===*/
/*============================================================================*/
static int path_inside_radius(double P[2],double S[2],double R) {
	double a,b,c;
	a=S[1]-P[1];	b=P[0]-S[0];	c=a*P[0]+b*P[1];
	return( (c*c/(a*a+b*b)) <= R*R );
}
/*============================================================================*/









/*============================================================================*/
/*====== COMPUTE VIA PATH FOR MINIMUM DISTANCE AROUND A SINGLE CYLINDER ======*/
/*============================================================================*/
static int single_cylinder(double R,double P[3],double q[3],double t[3],double S[3],
   double *Pq,double *qt,double *tS,double *L,int *active,double *ru) {
	double dp,ds,rtp,rts,sgn,Rpos,buff;		*ru=R;
	sgn=SIGN(R);	Rpos=ABS(R);	buff=(Rpos+OBST_BUFF)*(Rpos+OBST_BUFF);
	/*==================================================================*/
	dp=P[0]*P[0]+P[1]*P[1];	rtp=dp-R*R;	ds=S[0]*S[0]+S[1]*S[1];	rts=ds-R*R;
	if((dp<OBST_BUFF)||(ds<OBST_BUFF)) return(0);	/* NO ROOM TO SHRINK */
	if((dp<buff)||(ds<buff)) {	/* ONE POINT IS INSIDE CYLINDER SO SHRINK IT */
		if(dp<ds)	{ R=sgn*(sqrt(dp)-OBST_BUFF);	rtp=dp-R*R; rts=ds-R*R; }
		else        { R=sgn*(sqrt(ds)-OBST_BUFF);	rtp=dp-R*R; rts=ds-R*R; }
	}	*ru=R;
	/*==================================================================*/
	dp=R/dp; rtp=sqrt(rtp);	q[0]=(R*P[0]-rtp*P[1])*dp;	q[1]=(R*P[1]+rtp*P[0])*dp;
	ds=R/ds; rts=sqrt(rts);	t[0]=(R*S[0]+rts*S[1])*ds;	t[1]=(R*S[1]-rts*S[0])*ds;
	/*==================================================================*/
	if(R*(q[0]*t[1]-q[1]*t[0])<0.0) {	/* CURL>180 DEGREE - ASSUME INACTIVE */
		q[0]=P[0];	q[1]=P[1];	q[2]=P[2];	*Pq=0.0;
		t[0]=S[0];	t[1]=S[1];	t[2]=S[2];	*tS=0.0;
		*L=*qt=DISTANCE_BETWEEN(q,t);	*active=0;	/* INACTIVE */
		return(0);
	}
	/*==================================================================*/
	*Pq=sqrt((q[0]-P[0])*(q[0]-P[0])+(q[1]-P[1])*(q[1]-P[1]));
	*tS=sqrt((t[0]-S[0])*(t[0]-S[0])+(t[1]-S[1])*(t[1]-S[1]));
	*qt=R*acos(1.0-0.5*((q[0]-t[0])*(q[0]-t[0])+(q[1]-t[1])*(q[1]-t[1]))/(R*R));
	*qt=ABS(*qt);
	/*==================================================================*/
	q[2]=P[2]+(S[2]-P[2])*(*Pq)/(*Pq+*tS+*qt);
	t[2]=S[2]+(P[2]-S[2])*(*tS)/(*Pq+*tS+*qt);
	/*==================================================================*/
	*Pq=sqrt((*Pq)*(*Pq)+(q[2]-P[2])*(q[2]-P[2]));
	*tS=sqrt((*tS)*(*tS)+(t[2]-S[2])*(t[2]-S[2]));
	*qt=sqrt((*qt)*(*qt)+(q[2]-t[2])*(q[2]-t[2]));	*L = *Pq + *qt + *tS;
	/*==================================================================*/
	*active=1;	/* ACTIVE */
	return(0);
}
/*============================================================================*/








#define RETURN_DOUBLE return(double_cylinder(U,Ru,V,Rv,M,P,q,Q,T,t,S,Pq,qQ,QT,Tt,tS,L,active,ru,rv))

/*============================================================================*/
/*======== COMPUTE VIA PATH FOR MINIMUM DISTANCE AROUND TWO CYLINDERS ========*/
/*============================================================================*/
static int double_cylinder(double U[3],double Ru,double V[3],double Rv,double M[9],
   double P[3],double q[3],double Q[3],double T[3],double t[3],double S[3],
   double *Pq,double *qQ,double *QT,double *Tt,double *tS,double *L,
	int *active,double *ru,double *rv) {

	double Qd,Qrt,Qc,Td,Trt,Tc,TQ,LU,LV,TnU[3],QnV[3];
	double x[3],C[3],Cx[9];
	double dQdx[9],dTdQ[9];
	double dp,ds,rtp,rts,z1,z2,buff,Rpos,sgn;
	int i,count=0,MAXCOUNT=50;

	*ru=Ru;	*rv=Rv;

	if(*active!=3) switch(*active) {
	/*==================================================================*/
	/*=== RESPOND TO CURRENT INACTIVE STATE OF ONE OR BOTH CYLINDERS ===*/
	/*==================================================================*/
	case 0:	/* NEITHER CYLINDER IS ACTIVE */
		q[0]=P[0]; q[1]=P[1]; q[2]=P[2];		Q[0]=P[0]; Q[1]=P[1]; Q[2]=P[2];
		T[0]=S[0]; T[1]=S[1]; T[2]=S[2];		t[0]=S[0]; t[1]=S[1]; t[2]=S[2];
		quick_mul_vec_by_mtx(S,M,C);	quick_add_vec_to_vec(C,V,C);
		*Pq=0.0;	*qQ=0.0;		*L=*QT=DISTANCE_BETWEEN(P,C);		*Tt=0.0;	*tS=0.0;
		if(path_inside_radius(P,C,Ru)) {	*active=1;	RETURN_DOUBLE; }
		quick_mul_vec_by_mtxT(P,M,C); quick_add_vec_to_vec(C,U,C);
		if(path_inside_radius(C,S,Rv)) {	*active=2;	RETURN_DOUBLE; } return(0);
	/*==================================================================*/
	case 1:	/* U CYLINDER IS ACTIVE ALONE */
		T[0]=S[0]; T[1]=S[1]; T[2]=S[2];		t[0]=S[0]; t[1]=S[1]; t[2]=S[2];
		quick_mul_vec_by_mtx(S,M,C); quick_add_vec_to_vec(C,V,C); *Tt=0.0;*tS=0.0;
		i=1;	single_cylinder(Ru,P,q,Q,C,Pq,qQ,QT,L,&i,ru);
		if(!i) {	Q[0]=P[0]; Q[1]=P[1]; Q[2]=P[2];	/* NIETHER SHOULD BE ACTIVE */
			*QT=*qQ;	*qQ=0.0; *active=0;
			quick_mul_vec_by_mtxT(P,M,C); quick_add_vec_to_vec(C,U,C);	/*=2?*/
			if(path_inside_radius(C,S,Rv)) {	*active=2;	RETURN_DOUBLE; }
			return(0);
		}
		quick_mul_vec_by_mtxT(Q,M,C); quick_add_vec_to_vec(C,U,C);
		if(path_inside_radius(C,S,Rv)) {	*active=3;	RETURN_DOUBLE; } return(0);
	/*==================================================================*/
	case 2:	/* V CYLINDER IS ACTIVE ALONE */
		q[0]=P[0]; q[1]=P[1]; q[2]=P[2];		Q[0]=P[0]; Q[1]=P[1]; Q[2]=P[2];
		quick_mul_vec_by_mtxT(P,M,C); quick_add_vec_to_vec(C,U,C);*Pq=0.0;*qQ=0.0;
		i=1;	single_cylinder(Rv,C,T,t,S,QT,Tt,tS,L,&i,rv);
		if(!i) {	T[0]=S[0]; T[1]=S[1]; T[2]=S[2]; /* NIETHER SHOULD BE ACTIVE */
			*QT=*Tt; *Tt=0.0; *active=0; return(0); }
		quick_mul_vec_by_mtx(T,M,C);	quick_add_vec_to_vec(C,V,C);
		if(path_inside_radius(P,C,Ru)) { *active=3;	RETURN_DOUBLE; } return(0);
	}
	/*==================================================================*/

	/*==================================================================*/
	/*=== COMPUTE COMPONENTS OF q AND t WHICH DEPEND ONLY ON P AND S ===*/
	/*==================================================================*/
	x[0]=T[0];	x[1]=T[1];	x[2]=T[2];
	/*==================================================================*/
	dp=P[0]*P[0]+P[1]*P[1];	rtp=dp-Ru*Ru;
	sgn=SIGN(Ru);	Rpos=ABS(Ru);	buff=(Rpos+OBST_BUFF)*(Rpos+OBST_BUFF);
	if(dp<OBST_BUFF) return(0);							/* NO ROOM TO SHRINK */
	if(dp<buff) {					 /* POINT IS INSIDE CYLINDER SO SHRINK IT */
		Ru=sgn*(sqrt(dp)-OBST_BUFF);	rtp=dp-Ru*Ru; }
	/*==================================================================*/
	ds=S[0]*S[0]+S[1]*S[1];	rts=ds-Rv*Rv;
	sgn=SIGN(Rv);	Rpos=ABS(Rv);	buff=(Rpos+OBST_BUFF)*(Rpos+OBST_BUFF);
	if(ds<OBST_BUFF) return(0);							/* NO ROOM TO SHRINK */
	if(ds<buff) {					 /* POINT IS INSIDE CYLINDER SO SHRINK IT */
		Rv=sgn*(sqrt(ds)-OBST_BUFF);	rts=ds-Rv*Rv; }
	/*==================================================================*/
	*ru=Ru;	*rv=Rv;					dp=Ru/dp; rtp=sqrt(rtp);
	q[0]=(Ru*P[0]-rtp*P[1])*dp; q[1]=(Ru*P[1]+rtp*P[0])*dp;
	ds=Rv/ds; rts=sqrt(rts);
	t[0]=(Rv*S[0]+rts*S[1])*ds; t[1]=(Rv*S[1]-rts*S[0])*ds;
	*Pq=sqrt((q[0]-P[0])*(q[0]-P[0])+(q[1]-P[1])*(q[1]-P[1]));
	*tS=sqrt((t[0]-S[0])*(t[0]-S[0])+(t[1]-S[1])*(t[1]-S[1]));
	/*==================================================================*/

	/*==================================================================*/
	/*======== ITERATIVELY CONVERGE ON DOUBLE CYLINDER SOLUTION ========*/
	/*==================================================================*/
	for(count=0;count<MAXCOUNT;count++) {
		/*===============================================================*/
		/*======= COMPUTE CONSTRAINTS WHICH REPRESENT CONVERGENCE =======*/
		/*===============================================================*/
		quick_mul_vec_by_mtx(x,M,TnU);	quick_add_vec_to_vec(TnU,V,TnU);	
		/*===============================================================*/
		Td=TnU[0]*TnU[0]+TnU[1]*TnU[1];	Trt=Td-Ru*Ru;	Td=Ru/Td;
		if(Trt<0.0) return(-1); Trt=sqrt(Trt);
		Q[0]=(Ru*TnU[0]+Trt*TnU[1])*Td;	Q[1]=(Ru*TnU[1]-Trt*TnU[0])*Td;
		/*===============================================================*/
		*QT=sqrt((TnU[0]-Q[0])*(TnU[0]-Q[0])+(TnU[1]-Q[1])*(TnU[1]-Q[1]));
		Tc=1.0-0.5*((q[0]-Q[0])*(q[0]-Q[0])+(q[1]-Q[1])*(q[1]-Q[1]))/(Ru*Ru);
		if(fabs(Tc)>=1.0) *qQ=fabs(Ru*SimTK::Pi); else	*qQ=fabs(Ru*acos(Tc));
		/*===============================================================*/
		LU=1.0/(*Pq+*QT+*qQ);
		q[2]=P[2]+(TnU[2]-P[2])*(*Pq)*LU;
		Q[2]=TnU[2]+(P[2]-TnU[2])*(*QT)*LU;
		/*===============================================================*/
		quick_mul_vec_by_mtxT(Q,M,QnV);	quick_add_vec_to_vec(QnV,U,QnV);
		/*===============================================================*/
		Qd=QnV[0]*QnV[0]+QnV[1]*QnV[1];	Qrt=Qd-Rv*Rv;	Qd=Rv/Qd;
		if(Qrt<0.0) return(-1); Qrt=sqrt(Qrt);
		T[0]=(Rv*QnV[0]-Qrt*QnV[1])*Qd;	T[1]=(Rv*QnV[1]+Qrt*QnV[0])*Qd;
		/*===============================================================*/
		TQ=sqrt((T[0]-QnV[0])*(T[0]-QnV[0])+(T[1]-QnV[1])*(T[1]-QnV[1]));
		Qc=1.0-0.5*((t[0]-T[0])*(t[0]-T[0])+(t[1]-T[1])*(t[1]-T[1]))/(Rv*Rv);
		if(fabs(Qc)>=1.0) *Tt=fabs(Rv*SimTK::Pi); else	*Tt=fabs(Rv*acos(Qc));
		/*===============================================================*/
		LV=1.0/(*tS+TQ+*Tt);
		t[2]=S[2]+(QnV[2]-S[2])*(*tS)*LV;
		T[2]=QnV[2]+(S[2]-QnV[2])*(TQ)*LV;
		/*===============================================================*/
		C[0]=T[0]-x[0];	C[1]=T[1]-x[1];	C[2]=T[2]-x[2];
		/*===============================================================*/
	
		if(fabs(C[0])+fabs(C[1])+fabs(C[2])<1.0e-12) break;	/* CONVERGED */
	
		/*===============================================================*/
		/*========= COMPUTE ANALYTICAL GRADIENTS OF CONSTRAINTS =========*/
		/*===============================================================*/
		rtp=1/Trt;	dp=1/Ru;
		dQdx[0]=  (Ru +TnU[0]*TnU[1]*rtp-2*TnU[0]*Q[0]*dp)*Td;
		dQdx[1]=  (Trt+TnU[1]*TnU[1]*rtp-2*TnU[1]*Q[0]*dp)*Td;	dQdx[2]=0.0;
		dQdx[3]= -(Trt+TnU[0]*TnU[0]*rtp+2*TnU[0]*Q[1]*dp)*Td;
		dQdx[4]=  (Ru -TnU[0]*TnU[1]*rtp-2*TnU[1]*Q[1]*dp)*Td;	dQdx[5]=0.0;
		z1= *QT*(P[2]-TnU[2])*LU*LU;		z2=(P[2]-TnU[2])*LU-z1;
		rtp=(*qQ>=0.0?1.0:-1.0)*Ru/sqrt(1.0-Tc*Tc);
		dQdx[6]= z2*((Q[0]-TnU[0])*(dQdx[0]-1.0)+(Q[1]-TnU[1])*dQdx[3])/(*QT) +
					z1*rtp*((Q[0]-q[0])*dQdx[0]+(Q[1]-q[1])*dQdx[3])/(Ru*Ru);
		dQdx[7]= z2*((Q[0]-TnU[0])*dQdx[1]+(Q[1]-TnU[1])*(dQdx[4]-1.0))/(*QT) +
					z1*rtp*((Q[0]-q[0])*dQdx[1]+(Q[1]-q[1])*dQdx[4])/(Ru*Ru);
		dQdx[8]=1-(*QT)*LU;
		/*===============================================================*/
		quick_mul_mtxT_by_mtx(M,dQdx,Cx);	/* Cx USED AS TMP STORAGE */
		quick_mul_mtx_by_mtx(Cx,M,dQdx);
		/*===============================================================*/
		rtp=1/Qrt;	dp=1/Rv;
		dTdQ[0]=  (Rv -QnV[0]*QnV[1]*rtp-2*QnV[0]*T[0]*dp)*Qd;
		dTdQ[1]= -(Qrt+QnV[1]*QnV[1]*rtp+2*QnV[1]*T[0]*dp)*Qd; dTdQ[2]=0.0;
		dTdQ[3]=  (Qrt+QnV[0]*QnV[0]*rtp-2*QnV[0]*T[1]*dp)*Qd;
		dTdQ[4]=  (Rv +QnV[0]*QnV[1]*rtp-2*QnV[1]*T[1]*dp)*Qd; dTdQ[5]=0.0;
		z1=TQ*(S[2]-QnV[2])*LV*LV;		z2=(S[2]-QnV[2])*LV-z1;
		rtp=(*Tt>0.0?1.0:-1.0)*Rv/sqrt(1.0-Qc*Qc);
		dTdQ[6]= z2*((T[0]-QnV[0])*(dTdQ[0]-1.0)+(T[1]-QnV[1])*dTdQ[3])/TQ +
					z1*rtp*((T[0]-t[0])*dTdQ[0]+(T[1]-t[1])*dTdQ[3])/(Rv*Rv);
		dTdQ[7]= z2*((T[0]-QnV[0])*dTdQ[1]+(T[1]-QnV[1])*(dTdQ[4]-1.0))/TQ +
					z1*rtp*((T[0]-t[0])*dTdQ[1]+(T[1]-t[1])*dTdQ[4])/(Rv*Rv);
		dTdQ[8]=  1-TQ*LV;
		/*===============================================================*/
		quick_mul_mtx_by_mtx(dTdQ,dQdx,Cx);	Cx[0]-=1.0;	Cx[4]-=1.0;	Cx[8]-=1.0;
		quick_solve_linear(3,Cx,C,C);	quick_sub_vec_fm_vec(x,C,x);	/* UPDATE x */
		/*===============================================================*/
	}	T[0]=x[0];	T[1]=x[1];	T[2]=x[2];	/*== APPLY SOLUTION ==*/
	if(count>=MAXCOUNT) {
		printf("double_cyl: Unable to converge.\n");
		return(-1);
	}
	/*==================================================================*/

	/*==================================================================*/
	/*====== CHECK CYLINDERS TO SEE IF THEY SHOULD REMAIN ACTIVE =======*/
	/*==================================================================*/
	if(Ru*(Q[0]*q[1]-Q[1]*q[0])>0.0) {	/* U CYLINDER SHOULD BE INACTIVE */
		q[0]=P[0]; q[1]=P[1]; q[2]=P[2];		Q[0]=P[0]; Q[1]=P[1]; Q[2]=P[2];
		quick_mul_vec_by_mtxT(P,M,C);	quick_add_vec_to_vec(C,U,C);	*active=2;
		i=1;	single_cylinder(Rv,C,T,t,S,QT,Tt,tS,L,&i,rv);	*Pq=0.0;	*qQ=0.0;
		if(!i) { *active=0; return(0); }	/* NIETHER CYLINDER SHOULD BE ACTIVE */
	} else
	if(Rv*(t[0]*T[1]-t[1]*T[0])>0.0) {	/* V CYLINDER SHOULD BE INACTIVE */
		t[0]=S[0]; t[1]=S[1]; t[2]=S[2];		T[0]=S[0]; T[1]=S[1]; T[2]=S[2];
		quick_mul_vec_by_mtx(S,M,C);	quick_add_vec_to_vec(C,V,C);	*active=1;
		i=1;	single_cylinder(Ru,P,q,Q,C,Pq,qQ,QT,L,&i,ru);	*tS=0.0;	*Tt=0.0;
		if(!i) { *active=0; return(0); }	/* NIETHER CYLINDER SHOULD BE ACTIVE */
	} else { *active=3;						/* BOTH CYLINDERS SHOULD REMAIN ACTIVE */
		*Pq=sqrt((*Pq)*(*Pq)+(q[2]-P[2])*(q[2]-P[2]));
		*qQ=sqrt((*qQ)*(*qQ)+(Q[2]-q[2])*(Q[2]-q[2]));
		*QT=sqrt(TQ*TQ+(T[2]-QnV[2])*(T[2]-QnV[2]));
		*Tt=sqrt((*Tt)*(*Tt)+(t[2]-T[2])*(t[2]-T[2]));
		*tS=sqrt((*tS)*(*tS)+(S[2]-t[2])*(S[2]-t[2]));
		*L = *Pq + *qQ + *QT + *Tt + *tS;
	}
	/*==================================================================*/
	return(0);

}
/*============================================================================*/





























