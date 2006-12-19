// rdSDFast.cpp
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
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/Memory.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Models/SdfastEngine/SdfastBody.h>
#include <OpenSim/Simulation/SIMM/BodyIterator.h>
#include <OpenSim/Models/SdfastEngine/SdfastCoordinate.h>
#include <OpenSim/Simulation/SIMM/CoordinateIterator.h>
#include "rdSDFastDLL.h"
#include "sdfast.h"
#include "rdSDFast.h"
#include <OpenSim/Tools/ScaleSet.h>

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
const int rdSDFast::GROUND = -1;
rdSDFast* rdSDFast::_Instance = NULL;


using namespace std;



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
rdSDFast::rdSDFast()
{
	// INITIALIZE SDFAST
	sdinit();

	// SET NULL
	setNull();

	init();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML file
 */
rdSDFast::rdSDFast(const string &aFileName) :
Model(aFileName)
{
	// INITIALIZE SDFAST
	sdinit();

	setNull();
	updateFromXMLNode();

	init();

}
void rdSDFast::init()
{
	// SYSTEM INFORMATION
	constructSystemVariables();

	// JOINT AND AXIS MAPS
	constructJointAndAxisMaps();

	// BODIES, COORDINATES, JOINTS
	constructEngine();

	// STATIC INSTANCE
	_Instance = this;
}
//_____________________________________________________________________________
/**
 * Destructor.
 */
rdSDFast::~rdSDFast()
{
	// COORDINATE MAPS
	if(_u2jMap!=NULL) { delete[] _u2jMap;  _u2jMap=NULL; }
	if(_u2aMap!=NULL) { delete[] _u2aMap;  _u2aMap=NULL; }
	if(_dudt!=NULL) { delete[] _dudt;  _dudt=NULL; }
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the values of all data members to an appropriate "null" value.
 */
void rdSDFast::setNull()
{
	_u2jMap = NULL;
	_u2aMap = NULL;
}

//_____________________________________________________________________________
/**
 * Construct system variables.
 *
 * The following numbers are initialized by making SD/Fast calls:
 *		_nb	Number of bodies.
 *		_nj	Number of joints.
 *		_nu	Number of degrees of freedom.
 *		_nq	Number of generalized coordinates (_nu + number of ball joints).
 *
 * Memory allocations are performed for the following data members:
 *		_q
 *		_u
 *
 * @return -1 on an error, 0 otherwise.
 */
void rdSDFast::
constructSystemVariables()
{

	// GET SYSTEM INFORMATION.
	int sysinfo[50];
	sdinfo(sysinfo);

	// ASSIGN
	_nb = sysinfo[1];  // Number of bodies (joints).
	_nu = sysinfo[2];  // Number of degrees of freedom (sum of all tree joints).
	_nq = _nu + sysinfo[7];	// Number of generalized coordinates
	_nj = _nb + sysinfo[4];	// Number of joints

    // ALLOCATE MEMORY
	_dudt = new double[nu];

	// Check that some system has been defined.
	if ( (_nb<=0) || (_nu<=0) ) return;

	return;
}
//_____________________________________________________________________________
/**
 * Construct maps that provide the joint and axis of a specified degree
 * of freedom.
 * @return -1 on an error, 0 otherwise.
 */
void rdSDFast::
constructJointAndAxisMaps()
{
	// ALLOCATE JOINT MAP
	_u2jMap = NULL;
	_u2jMap = new int[_u.getSize()];
	if (_u2jMap==NULL) return;

	// ALLOCATE AXIS MAP
	_u2aMap = NULL;
	_u2aMap = new int[_u.getSize()];
	if (_u2aMap==NULL) {
		delete _u2jMap; _u2jMap=NULL;
		return;
	}

	// SET MAPS
	// Run through each joint (body) and get information about the number of
	// degrees of freedom for that joint.  Then, assign appropriate values to
	// the maps.
	int jntinfo[50];   // Joint information data vector
	int slider[6];     // Slider joint information vector (not used).
	int u, joint, axis;
	for (u=joint=0;joint<_nb;joint++)
	{
		// GET JOINT INFO
		sdjnt(joint,jntinfo,slider);

		// LOOP THROUH AXES AT JOINT
		// BUG-  This requires testing.  jntinfo[4] may not provide all dof,
		// but only unprescribed dof's.
		axis = 0;
		while (axis < jntinfo[4])
		{
			_u2jMap[u] = joint; // Generalized coordinate "u" points to this joint
			_u2aMap[u] = axis;  //  and this axis of the joint.

			u++;                // Increment to the next coordinate
			axis++;             //  and the next axis of this joint.
		}
	}
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the number of generalized speeds.
 */
int rdSDFast::
getNumSpeeds() const
{
	return(_u.getSize());
}
//_____________________________________________________________________________
/**
 * Set the name of a generalized speed.
 *
 * @param aIndex Index of the speed whose name is to be set.
 * @param aName Name to which to set the name of the speed name.
 */
void rdSDFast::
setSpeedName(int aIndex,const string &aName)
{
	if(aIndex<0) return;
	if(aIndex>=getNumSpeeds()) return;
	_u.setNameAt(aIndex,aName);
}
//_____________________________________________________________________________
/**
 * Get the name of a generalized speed.
 *
 * @param aIndex Index of the speed whose name is desired.  aIndex should
 * be greater than or equal to 0 and less than the number of generalized
 * speeds.
 * @return Control name.
 * @see getNumSpeeds()
 */
string rdSDFast::
getSpeedName(int aIndex) const
{
	if(aIndex<0) return("");
	if(aIndex>=getNumSpeeds()) return("");
	return(_u.getNameAt(aIndex));
}
//_____________________________________________________________________________
/**
 * Get the index of a generalized speed given its name.
 * The returned indices start at 0: for the first, 0 is returned; for
 * the second, 1; etc.
 *
 * Note that the speed names need to be unique for this method
 * to work properly.
 *
 * @param aName Name of a speed.
 * @return Index of the first speed with the name aName; -1, if there is no
 * such speed or if there is an error.  Indicies start at 0.
 */
int rdSDFast::
getSpeedIndex(const string &aName) const
{
	return(_u.getIndex(aName));
}
//_____________________________________________________________________________
/**
 * Get the last-computed values of the accelerations of the generalized
 * coordinates.  For the values to be valid, the method
 * computeAccelerations() must have been called.
 *
 * @param rDUDT Array to be filled with values of the accelerations of the
 * generalized coordinates.  The length of rDUDT should be at least as large
 * as the value returned by getNumSpeeds().
 * @see computeAccelerations()
 * @see getAcceleration(int aIndex)
 * @see getAcceleration(const char* aName);
 */
void rdSDFast::
getAccelerations(double rDUDT[]) const
{
	if(rDUDT==NULL) return;
	int i;
	for(i=0;i<_u.getSize();i++) rDUDT[i] = _dudt[i];
}
//_____________________________________________________________________________
/**
 * Get the last-computed value of the acceleration a generalized coordinate.
 * For the returned value to be valid, the method computeAccelerations()
 * must have been called.
 *
 * @param aIndex Index of the acceleration:  0 <= aIndex < getNumSpeeds().
 * @return Value of the acceleration.  rdMath::NAN is returned on an error.
 * @see computeAccelerations()
 * @see getAccelerations(double rDUDT[])
 * @see getAcceleration(const char* aName);
 */
double rdSDFast::
getAcceleration(int aIndex) const
{
	if((aIndex<0)||(aIndex>=getNumSpeeds())) {
		printf("rdSDFast.getAcceleration: ERROR- index out of bounds %d.\n",aIndex);
		return(rdMath::NAN);
	}
	return(_dudt[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get the last-computed value of the acceleration a generalized coordinate
 * specified by name.
 * For the returned value to be valid, the method computeAccelerations()
 * must have been called.
 *
 * Note that this method is slow and should not be used in code where
 * it might be called repeatedly.
 *
 * @param aName Name of the speed whose derivative is returned.
 * @return Value of the acceleration.  rdMath::NAN is returned on an error.
 * @see computeAccelerations()
 * @see getAccelerations(double rDUDT[])
 * @see getAcceleration(int aIndex);
 */
double rdSDFast::
getAcceleration(const string &aSpeedName) const
{
	int index = getSpeedIndex(aSpeedName);
	if(index<0) {
		cout<<"rdSDFast.getAcceleration: ERROR- unknown speed "<<aSpeedName<<
			"."<<endl;
		return(rdMath::NAN);
	}
	return(_dudt[index]);
}

//-----------------------------------------------------------------------------
// SPEEDS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the current values of the generalized speeds of the model.
 *
 * @param rU Array to be filled with values of the generalized speeds.
 * The length of rU should be at least as large as the value returned by
 * getNumSpeeds().
 */
void rdSDFast::
getSpeedValues(double rU[]) const
{
	_u.getValues(rU);
}
//_____________________________________________________________________________
/**
 * Get a speed by name.
 * Note that this method is slow and should not be used in code where
 * it might be called repeatedly.
 *
 * @param aName Name of the speed.
 * @return Pointer to the speed object.  NULL is returned on an error.
 * @see getSpeedValues(double rU[])
 */
AbstractSpeed* rdSDFast::
getSpeed(const string &aName) const
{
	int index = getSpeedIndex(aName);
	if(index<0) {
		cout<<"rdSDFast.getSpeed: ERROR- unknown speed "<<aName<<"."<<endl; 
		return(NULL);
	}
	return(_u[aName]);
}

//-----------------------------------------------------------------------------
// COORDINATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the current values of the generalized coordinates of the model.
 *
 * @param rQ Array to be filled with values of the generalized coordinates.
 * The length of rQ should be at least as large as the value returned by
 * getNumCoordinates().
 */
void rdSDFast::
getCoordinates(double rQ[]) const
{
	if(rQ==NULL) return;
	int i;
	for(i=0;i<getNumCoordinates();i++) rQ[i] = _q[i];
}
//_____________________________________________________________________________
/**
 * Get the current value of a coordinate by index.
 *
 * @param aIndex Index of the coordinate:  0 <= aIndex < getNumCoordinates().
 * @return Value of the cooridnate.  rdMath::NAN is returned on an error.
 * @see getCoordinates(double rQ[])
 * @see getCoordinate(const char* aName);
 */
double rdSDFast::
getCoordinate(int aIndex) const
{
	if((aIndex<0)||(aIndex>=getNumCoordinates())) {
		printf("rdSDFast.getCoordinate: ERROR- index out of bounds %d.\n",aIndex);
		return(rdMath::NAN);
	}
	return(_q[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get the value of a coordinate by name.
 * Note that this method is slow and should not be used in code where
 * it might be called repeatedly.
 *
 * @param aName Name of the coordinate.
 * @return Value of the coordinate.  rdMath::NAN is returned on an error.
 * @see getCoordinates(double rQ[])
 * @see getCoordinate(int aIndex);
 */
double rdSDFast::
getCoordinate(const string &aName) const
{
	int index = getCoordinateIndex(aName);
	if(index<0) {
		cout<<"rdSDFast.getCoordinate: ERROR- unknown coordinate "<<
			aName<<"."<<endl;
		return(rdMath::NAN);
	}
	return(_q[index]);
}
//_____________________________________________________________________________
/**
 * Get the index of a generalized coordinate given its name.
 * The returned indices start at 0: for the first, 0 is returned; for
 * the second, 1; etc.
 *
 * Note that the coordinate names need to be unique for this method
 * to work properly.
 *
 * @param aName Name of a coordinate.
 * @return Index of the first coordinate with the name aName; -1, if there is no
 * such coordinate or if there is an error.  The indicies start at 0.
 */
int rdSDFast::
getCoordinateIndex(const string &aName) const
{
	int i;
	for(i=0;i<getNumCoordinates();i++) {
		if(aName == getCoordinateName(i)) return(i);
	}

	return(-1);
}
//-----------------------------------------------------------------------------
// CONFIGURATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the configuration of the model.
 *
 * @param aQ Generalized coordinates.
 * @param aU Generalized speeds.
 * @todo Should the states (y) be set when q and u are set?
 */
void rdSDFast::
setConfiguration(const double aQ[],const double aU[])
{
	_q.setValues(aQ);
	_u.setValues(aU);
	sdstate(_t,(double *)aQ,(double *)aU);
}
//_____________________________________________________________________________
/**
 * Set the configuration of the model.
 *
 * @param aY Statevector.
 */
void rdSDFast::
setConfiguration(const double aY[])
{
	setConfiguration((double *)aY,(double *)&aY[getNumCoordinates()]);
}

//_____________________________________________________________________________
/**
 * Extract the generalized coordinates and speeds from a state vector.
 * Note that this method does not alter the internal state of the model.
 *
 * @param aY Array of states from which to extract the generalized
 * coordinates and speeds.
 * @param rQ Extracted generalized coordinates.
 * @param rU Extracted generalized speeds.
 */
void rdSDFast::
extractConfiguration(const double aY[],double rQ[],double rU[]) const
{
	int i;
	for(i=0;i<getNumCoordinates();i++)  rQ[i] = aY[i];
	for(i=0;i<_u.getSize();i++)  rU[i] = aY[getNumCoordinates()+i];
}
//_____________________________________________________________________________
/**
 * Assemble the model by calling rdSDFast code to perform assembly.
 * Model constructors should invoke this method to make sure models are started
 * in a well defined configuration.
 *
 */
int rdSDFast::
assemble(double aTime,double *rState,int *aLock,double aTol,
		 int aMaxevals,int *rFcnt,int *rErr)
{
	sdassemble(aTime,rState,aLock,aTol,aMaxevals,rFcnt,rErr);
	return *rErr;
}
//_____________________________________________________________________________
/**
 * Scale a model by the specified ScaleSet.
 *
 * return true on success, false on failure
 */
bool rdSDFast::
scale(const ScaleSet& aScaleSet)
{
	int i;
	double	(*segmentScales)[3] = new double[getNumBodies()][3];
	for (i=0; i < getNumBodies(); i++){
		segmentScales[i][0]=segmentScales[i][1]=segmentScales[i][2]=1.0;
	}
	for(i=0; i < aScaleSet.getSize(); i++){
		// Scale factors
		Scale *aScale = aScaleSet.get(i);
		Array<double>	scaleFactors(1.0, 3);
		aScale->getScaleFactors(scaleFactors);
		// get name of segment from aScale 
		const string& segName = aScale->getSegmentName();
		// Keep scale factor in array for later use
		int segmentIndex = getBodySet()->getIndex(segName);
		for(int j=0; j <3; j++)
			segmentScales[segmentIndex][j] = scaleFactors[j];

		// Name search to locate the body and scale it.
		Body* segment = getBodySet()->get(segName);
		// scaleBy scales the body's center of mass and inertial properties
		segment->scaleBy(&scaleFactors[0]);
	}
	// Scale the location of the joints using segment scaling factors
	int info[50], slider[6], inbody, outbody;
	double itj[3], btj[3];
	for (i=0;i<getNumBodies();i++) {
		sdgetitj(i,itj);
		sdgetbtj(i,btj);
		sdjnt(i,info,slider);
		inbody=info[2];
		outbody=info[3];
		for (int j=0;j<3;j++) {
			itj[j]*=segmentScales[inbody][j];
			btj[j]*=segmentScales[outbody][j];
		}
		sditj(i,itj);
		sdbtj(i,btj);
	}

	delete[] segmentScales;
	return true;
}
//_____________________________________________________________________________
/**
 * Dump model description to a text file
 *
 * @param aCallback Pointer to the derivative callback to add.
 */
void rdSDFast::
dump(const std::string& aFileName)
{
	FILE *dumpFile = fopen(aFileName.c_str(), "w");
	int info[50], slider[6], inbody, outbody;
	double itj[3], btj[3];
	for (int i=0; i < _nb;i++) {
		fprintf(dumpFile, "Body %d:\n=======\n", i);
		Body* segment = getBodySet()->get(i);
		double com[3];
		segment->getCenterOfMass(com);
		fprintf(dumpFile, "COM: (%lf, %lf, %lf)\n", com[0], com[1], com[2]);
		double inertia[3][3];
		segment->getInertia(inertia);
		fprintf(dumpFile, "Mass: %lf, Inertia (%lf, %lf, %lf, %lf, %lf, %lf)\n", 
			segment->getMass(), inertia[0][0], inertia[1][1], inertia[2][2], 
								inertia[0][1], inertia[1][2], inertia[0][2]);

		double	xform[3];
		getInboardToJointBodyLocal(i, xform);
		fprintf(dumpFile, "Inboard to joint %lf, %lf, %lf\n", 
			i , xform[0], xform[1], xform[2]);
		getBodyToJointBodyLocal(i, xform);
		fprintf(dumpFile, "Body to joint body %lf, %lf, %lf\n", 
			i , xform[0], xform[1], xform[2]);
	}
	fclose(dumpFile);

}//-----------------------------------------------------------------------------
// GRAVITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the gravity.
 * @param aG Gravity vector.
 */
bool rdSDFast::
setGravity(double aG[3])
{
	Model::setGravity(aG);
	sdgrav(aG);
	sdinit();
	return true;
}
//_____________________________________________________________________________
/**
 * Get the gravity.
 * @param rG Gravity vector.
 */
void rdSDFast::
getGravity(double rG[3]) const
{
	sdgetgrav(rG);
}

//-----------------------------------------------------------------------------
// BODY INFORMATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the body ID of the ground (or inertial) frame.
 */
int rdSDFast::
getGroundID() const
{
	return(GROUND);
}

//_____________________________________________________________________________
/**
 * Set the vector directed from a body's center of mass to its joint.  The
 * body's joint is always the joint that that body has with its inboard body,
 * that is, the body specified should be the outboard body of the joint of interest.
 *
 * @param aBody Body ID.
 * @param aBTJ Vector from the center of mass of the body to its joint with
 * its inboard body.  This vector is expressed in the local frame of aBody.
 */
void rdSDFast::
setBodyToJointBodyLocal(int aBody,const double aBTJ[3])
{
	double btj[3];
	memcpy(btj,aBTJ,3*sizeof(double));
	sdbtj(aBody,btj);
	sdinit();
}
//_____________________________________________________________________________
/**
 * Get the vector directed from a body's center of mass to its joint.  The
 * body's joint is always the joint that that body has with its inboard body.
 * that is, the body specified should be the outboard body of the joint of interest.
 *
 * @param aBody Body ID.
 * @param rBTJ Vector from the center of mass of the body to its joint with
 * its inboard body.  This vector is expressed in the local frame of aBody.
 */
void rdSDFast::
getBodyToJointBodyLocal(int aBody,double rBTJ[3]) const
{
	sdgetbtj(aBody,rBTJ);
}

//_____________________________________________________________________________
/**
 * For a specifited body, set the vector directed from the center of mass of
 * the body's inboard body to the body's joint with the inboard body.
 * that is, the body specified should be the outboard body of the joint of interest.
 *
 * @param aBody Body ID.
 * @param aBTJ Vector from the center of mass of the body's inboard body
 * to its joint with the body's inboard body.  This vector is expressed
 * in the local frame of the inboard body.
 */
void rdSDFast::
setInboardToJointBodyLocal(int aBody,const double aITJ[3])
{
	double itj[3];
	memcpy(itj,aITJ,3*sizeof(double));
	sditj(aBody,itj);
	sdinit();
}
//_____________________________________________________________________________
/**
 * For a specifited body, get the vector directed from the center of mass of
 * the body's inboard body to the body's joint with the inboard body.
 * that is, the body specified should be the outboard body of the joint of interest.
 *
 * @param aBody Body ID.
 * @param rBTJ Vector from the center of mass of the body's inboard body
 * to its joint with the body's inboard body.  This vector is expressed
 * in the local frame of the inboard body.
 */
void rdSDFast::
getInboardToJointBodyLocal(int aBody,double rITJ[3]) const
{
	sdgetitj(aBody,rITJ);
}
//_____________________________________________________________________________
/**
 * Set the vector describing the pin axis direction.  The
 * body's joint is always the joint that that body has with its inboard body.
 * that is, the body specified should be the outboard body of the joint of interest.
 *
 * @param aBody Body ID.
 * @param aPinNumber Number of the pin within the body's joint to be set.
 * @param aPin Vector describing the pin axis direction. This vector is expressed in
 * the local frame of aBody.
 */
void rdSDFast::
setPin(int aBody,int aPinNumber,const double aPin[3])
{
	double pin[3];
	memcpy(pin,aPin,3*sizeof(double));
	sdpin(aBody,aPinNumber,pin);
	sdinit();
}
//_____________________________________________________________________________
/**
 * Get the vector describing the pin axis direction.  The
 * body's joint is always the joint that that body has with its inboard body.
 * that is, the body specified should be the outboard body of the joint of interest.
 *
 * @param aBody Body ID.
 * @param aPinNumber Number of the pin within the body's joint to be set.
 * @param rPin Vector describing the pin axis direction. This vector is expressed in
 * the local frame of aBody.
 */
void rdSDFast::
getPin(int aBody,int aPinNumber,double rPin[3]) const
{
	sdgetpin(aBody,aPinNumber,rPin);
}
//_____________________________________________________________________________
/**
 * Get information about a specified joint.
 *
 * @param aJoint Joint ID.
 * @param rInfo Vector containing joint information.
 * @param rSlider Vector containing slider information
 */
void rdSDFast::
getJointInfo(int aJoint,int rInfo[50],int rSlider[6]) const
{
	sdjnt(aJoint,rInfo,rSlider);
}

//_____________________________________________________________________________

//-----------------------------------------------------------------------------
// INERTIA PARAMETERS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the mass of a body.
 * @param aBody Body ID.
 * @return Body mass
 */
double rdSDFast::
getMass(int aBody) const
{
	double mass;

	sdgetmass(aBody,&mass);

	return(mass);
}
//_____________________________________________________________________________
/**
 * Set the mass of a body.
 * @param aBody Body ID.
 * @param aMass Body mass
 * @return -1 on an error, 0 otherwise.
 */
int rdSDFast::
setMass(int aBody, double aMass)
{
	// CHECK aBody
	if((aBody<0)||(aBody>=_nb)) return(-1);

	// CHECK aMass
	if(aMass<0) return(-1);

	sdmass(aBody,aMass);
	sdinit();

	return(0);
}

//_____________________________________________________________________________
/**
 * Get the scalar inertia matrix of a body.
 *
 * @param aBody Body ID.
 * @param rI Inertia matrix expressed in terms of body-local coordinates.
 * @return -1 on an error, 0 otherwise.
 */
int rdSDFast::
getInertiaBodyLocal(int aBody,double rI[3][3]) const
{
	// CHECK aBody
	if((aBody<0)||(aBody>=_nb)) return(-1);

	// GET I
	sdgetiner(aBody,rI);

	return(0);
}

//_____________________________________________________________________________
/**
 * Set the scalar inertia matrix of a body.
 *
 * @param aBody Body ID.
 * @param aI Inertia matrix expressed in terms of body-local coordinates.
 * @return -1 on an error, 0 otherwise.
 */
int rdSDFast::
setInertiaBodyLocal(int aBody,const double aI[3][3])
{
	// CHECK aBody
	if((aBody<0)||(aBody>=_nb)) return(-1);

	double i[3][3];
	memcpy(i,aI,9*sizeof(double));

	// SET I
	sdiner(aBody,i);
	sdinit();

	return(0);
}

//_____________________________________________________________________________
/**
 * Get the scalar inertia matrix of a body.
 *
 * @param aBody Body ID.
 * @param rI Inertia matrix expressed in terms of body-local coordinates.
 * @return -1 on an error, 0 otherwise.
 */
int rdSDFast::
getInertiaBodyLocal(int aBody,double *rI) const
{

	// Check that return vector has been defined.
	if (rI==NULL) return(-1);

	// CHECK aBody
	if((aBody<0)||(aBody>=_nb)) return(-1);

	// GET I
	double I[3][3];
	sdgetiner(aBody,I);

	// ASSIGN returned values to the return vector.
	memcpy(rI,&I[0],9*sizeof(double));

	return(0);
}

//_____________________________________________________________________________
/**
 * Get the mass, center of mass, and inertia tensor of the whole model.
 * @param rM System mass.
 * @param rCOM System center of mass.
 * @param rI System inertia tensor.
 */
void rdSDFast::
getSystemInertia(double *rM,double rCOM[3],double rI[3][3]) const
{
	sdsys(rM,rCOM,rI);
}

//_____________________________________________________________________________
/**
 * Get the mass, center of mass, and inertia tensor of the whole model.
 * @param rM System mass.
 * @param rCOM System center of mass.
 * @param rI System inertia tensor.
 */
void rdSDFast::
getSystemInertia(double *rM,double *rCOM,double *rI) const
{

	// Check that the return vectors have been defined.
	if ( (rM==NULL) || (rCOM==NULL) || (rI==NULL) ) return;

	double com[3];
	double I[3][3];

	sdsys(rM,com,I);

	// ASSIGN returned values to the return vectors.
	         *rCOM = com[0];
	rCOM++;  *rCOM = com[1];
	rCOM++;  *rCOM = com[2];

	memcpy(rI,&I[0][0],9*sizeof(double));

}
//_____________________________________________________________________________
/**
 * Get the number of generalized coordinates.
 */
int rdSDFast::
getNumCoordinates() const
{
	return(_q.getSize());
}
//_____________________________________________________________________________
/**
 * Set the name of a generalized coordinate.
 *
 * @param aIndex Index of the coordinate whose name should be set.  aIndex
 * should be greater than or equal to 0 and less than the number of
 * coordinates.
 * @param aName Name of generalized coordinate.
 * @see getNumCoordinates()
 *
 * @todo: if name is null we need to generate a unique default name. Need to know how this
 * function is used to make sure generated names do not collide with user-assigned names
 * that the system might not be aware of yet.
 */
void rdSDFast::
setCoordinateName(int aIndex,const string &aName)
{
	// CHECK
	if(aIndex<0) return;
	if(aIndex>=getNumCoordinates()) return;
	_q.setNameAt(aIndex, aName);
}
//_____________________________________________________________________________
/**
 * Get the name of a generalized coordinate.
 *
 * @param aIndex Index of the coordinate whose name is desired.  aIndex should
 * be greater than or equal to 0 and less than the number of generalized
 * coordinates.
 * @return Control name.
 * @see getNumCoordinates()
 */
string rdSDFast::
getCoordinateName(int aIndex) const
{
	if(aIndex<0) return(NULL);
	if(aIndex>=getNumCoordinates()) return(NULL);
	return(_q.getNameAt(aIndex));
}

//_____________________________________________________________________________
/**
 * Get the number of joints in this model.
 */
int rdSDFast::
getNumJoints() const
{
	return(_nj);
}

//-----------------------------------------------------------------------------
// KINEMATICS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Transform a vector from one frame to another.
 *
 * @param aBody1 ID of the body in which the vector is originally defined.
 * @param aVec1 Vector to be transformed.
 * @param aBody2 ID of the body in which the new vector will be defined.
 * @param rVec2 Vector coordinates after transformation.
 */
void rdSDFast::
transform(int aBody1,const double aVec1[3],int aBody2,double rVec2[3]) const
{

	// Check that input and return vectors have been defined.
	if ( (aVec1==NULL) || (rVec2==NULL) ) return;

	double vec1[3];
	double vec2[3];

	// COPY the original vector to a vector of three elements.
	          vec1[0] = *aVec1;
	aVec1++;  vec1[1] = *aVec1;
	aVec1++;  vec1[2] = *aVec1;

	// TRANSFORM the vector.
	sdtrans(aBody1,vec1,aBody2,vec2);

	// ASSIGN the returned vector to the return argument.
	          *rVec2 = vec2[0];
	rVec2++;  *rVec2 = vec2[1];
	rVec2++;  *rVec2 = vec2[2];

}

//_____________________________________________________________________________
/**
 * Transform a position from one frame to the inertial frame.
 *
 * @param aBody ID of the body in which the position is originally defined 
 * @param aPos Position of point relative to the COM of aBody, expressed in 
 * the reference frame of aBody.
 * @param rPos Position of point relative to the COM of aBody, expressed in 
 * the inertial frame.
 */
void rdSDFast::
transformPosition(int aBody,const double aPos[3],double rPos[3]) const
{

	// Check that input and return vectors have been defined.
	if ( (aPos==NULL) || (rPos==NULL) ) return;

	double posGlobal[3];
   double posBodyCOMLocal[3] = {0,0,0};
	double posBodyCOMGlobal[3];

	// TRANSFORM the position.
	getPosition(aBody,aPos,posGlobal);
	getPosition(aBody,posBodyCOMLocal,posBodyCOMGlobal);
	Mtx::Subtract(1,3,posGlobal,posBodyCOMGlobal,rPos);

}
//_____________________________________________________________________________
/**
 * Get the inertial position of a point on a body.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Body ID.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rPos Position of the point in the inertial frame.
 *
 * @see setConfiguration()
 */
void rdSDFast::
getPosition(int aBody,const double aPoint[3],double rPos[3]) const
{
	sdpos(aBody,(double *)aPoint,rPos);
}
//_____________________________________________________________________________
/**
 * Get the inertial velocity of a point on a body.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Body ID.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rVel Velocity of the point in the inertial frame.
 *
 * @see setConfiguration()
 */
void rdSDFast::
getVelocity(int aBody,const double aPoint[3],double rVel[3]) const
{
	sdvel(aBody,(double *)aPoint,rVel);
}
//_____________________________________________________________________________
/**
 * Get the inertial acceleration of a point on a body.
 *
 * Note that the configuration of the model must be set and accelerations of
 * the generalized coordinates must be computed before calling this method.
 *
 * @param aBody Body ID.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rAcc Acceleration of the point in the inertial frame.
 *
 * @see set()
 * @see computeAccelerations()
 */
void rdSDFast::
getAcceleration(int aBody,const double aPoint[3],double rAcc[3]) const
{
	sdacc(aBody,(double *)aPoint,rAcc);
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Body ID.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void rdSDFast::
getDirectionCosines(int aBody,double rDirCos[3][3]) const
{
	sdorient(aBody,rDirCos);
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Body ID.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void rdSDFast::
getDirectionCosines(int aBody,double *rDirCos) const
{

	// Check that return vector has been defined.
	if (rDirCos==NULL) return;

	double dirCos[3][3];

	sdorient(aBody,dirCos);

	// Copy the direction cosines to the return vector.
	memcpy(rDirCos,&dirCos[0][0],9*sizeof(double));

}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity of a body in the ground reference frame.
 *
 * @param aBody Body ID.
 * @param rAngVel Angular velocity of the body.
 */
void rdSDFast::
getAngularVelocity(int aBody,double rAngVel[3]) const
{
	sdangvel(aBody,rAngVel);
	sdtrans(aBody,rAngVel,GROUND,rAngVel);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity in the local body reference frame.
 *
 * @param aBody Body ID.
 * @param rAngVel Angular velocity of the body.
 */
void rdSDFast::
getAngularVelocityBodyLocal(int aBody,double rAngVel[3]) const
{
	sdangvel(aBody,rAngVel);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration of a body in the ground reference 
 * frame.
 *
 * @param aBody Body ID.
 * @param rAngAcc Angular acceleration of the body.
 */
void rdSDFast::
getAngularAcceleration(int aBody,double rAngAcc[3]) const
{
	sdangacc(aBody,rAngAcc);
	sdtrans(aBody,rAngAcc,GROUND,rAngAcc);
}
//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration in the local body reference frame.
 *
 * @param aBody Body ID.
 * @param rAngAcc Angular acceleration of the body.
 */
void rdSDFast::
getAngularAccelerationBodyLocal(int aBody,double rAngAcc[3]) const
{
	sdangacc(aBody,rAngAcc);
}


//=============================================================================
// DERIVATIVES
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the derivatives of the generalized coordinates and speeds.
 *
 * @param dqdt Derivatives of generalized coordinates.
 * @param dudt Derivatives of generalized speeds.
 */
int rdSDFast::
computeAccelerations(double *dqdt,double *dudt)
{
	sdderiv(dqdt,dudt);
	Mtx::Assign(1,getNumSpeeds(),dudt,_dudt);
	return(0);
}


//=============================================================================
// LOAD APPLICATION
//=============================================================================
//-----------------------------------------------------------------------------
// FORCES EXPRESSED IN INERTIAL FRAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a force expressed in the inertial frame to a point on a body.
 *
 * @param aBody Body ID.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param aForce Force applied to the body expressed in the inertial frame.
 */
void rdSDFast::
applyForce(int aBody,const double aPoint[3],const double aForce[3])
{
	// TO ALLOW CONSTANT ARGUMENTS
	int i;
	double p[3],f[3];
	for(i=0;i<3;i++) {
		p[i] = aPoint[i];
		f[i] = aForce[i];
	}

	// APPLY
	double force[3];
	sdtrans(GROUND,f,aBody,force);
	sdpointf(aBody,p,force);
}
//_____________________________________________________________________________
/**
 * Apply a set of forces expressed in the inertial frame to a set of bodies.
 *
 * The body points, aPoint, should be expressed in the body-local frame.
 * The forces, aForce, should be expressed in the inertial frame.
 * @param aN Number of applied forces.
 * @param aBodies Array of body ID's.
 * @param aPoints Pointer to a sequence of points expressed in the body-local
 * frame laid out as aPoints[aN][3].
 * @param aForces	Pointer to a sequence of forces expressed in the inertial
 * frame laid out as aForces[aN][3].
 */
void rdSDFast::
applyForces(int aN,const int aBodies[],const double aPoints[][3],const double aForces[][3])
{
	int i;
	for(i=0;i<aN;i++) {
		applyForce(aBodies[i],aPoints[i],aForces[i]);
	}
}
//_____________________________________________________________________________
/**
 * Apply forces expressed in the inertial frame to points  bodies.
 *
 * @param aN Number of applied forces.
 * @param aBodes Array of body ID's.
 * @param aPoints Pointer to a sequence of points expressed in the body-local
 * frame laid out as aPoints[aN][3].
 * @param aForces	Pointer to a sequence of forces expressed in the inertial
 * frame laid out as aForces[aN][3].
 */
void rdSDFast::
applyForces(int aN,const int aBodies[],const double *aPoints,
	const double *aForces)
{
	int i,I;
	for(i=0;i<aN;i++) {
		I = Mtx::ComputeIndex(i,3,0);
		applyForce(aBodies[i],&aPoints[I],&aForces[I]);
	}
}

//-----------------------------------------------------------------------------
// FORCES EXPRESSED IN BODY-LOCAL FRAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a force expressed in the body-local frame to a point on a body.
 *
 * @param aBody Body ID.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param aForce Force applied to the body expressed in the body-local frame.
 */
void rdSDFast::
applyForceBodyLocal(int aBody,const double aPoint[3],const double aForce[3])
{
	// TO ALLOW CONSTANT ARGUMENTS
	int i;
	double p[3],f[3];
	for(i=0;i<3;i++) {
		p[i] = aPoint[i];
		f[i] = aForce[i];
	}

	// APPLY FORCE
	sdpointf(aBody,p,f);
}
//_____________________________________________________________________________
/**
 * Apply a set of forces to a set of bodies.
 *
 * @param aN Number of Forces.
 * @param aBody Array of Body ID's.
 * @param aPoint Array of points on the bodies expressed in the body-local
 * frames.
 * @param aForce Array of forces applied to the body expressed in the
 * body-local frames.
 */
void rdSDFast::
applyForcesBodyLocal(int aN,const int aBodies[],const double aPoints[][3],
	const double aForces[][3])
{
	int i;
	for(i=0;i<aN;i++) {
		applyForceBodyLocal(aBodies[i],aPoints[i],aForces[i]);
	}
}
//_____________________________________________________________________________
/**
 * Apply a set of forces to a set of bodies.
 *
 * @param aN Number of Forces.
 * @param aBody Array of Body ID's.
 * @param aPoint Array of points on the bodies expressed in the body-local
 * frames.
 * @param aForce Array of forces applied to the body expressed in the
 * body-local frames.
 */
void rdSDFast::
applyForcesBodyLocal(int aN,const int aBodies[],const double *aPoints,
	const double *aForces)
{

	// Check that the input vectors have been defined.
	if ( (aPoints==NULL) || (aForces==NULL) ) return;

	int i;
	double point[3];
	double force[3];

	for(i=0;i<aN;i++) {

		            point[0] = *aPoints;
		aPoints++;  point[1] = *aPoints;
		aPoints++;  point[2] = *aPoints;

		            force[0] = *aForces;
		aForces++;  force[1] = *aForces;
		aForces++;  force[2] = *aForces;

		applyForceBodyLocal(aBodies[i],point,force);
	}

}

//-----------------------------------------------------------------------------
// TORQUES EXPRESSED IN INERTIAL FRAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a torque expressed in the inertial frame to a body.
 *
 * @param aBody Body ID.
 * @param aTorque Torque expressed in the inertial frame.
 */
void rdSDFast::
applyTorque(int aBody,const double aTorque[3])
{
	// TO ALLOW CONSTANT ARGUMENTS
	int i;
	double torque[3];
	for(i=0;i<3;i++) {
		torque[i] = aTorque[i];
	}

	sdtrans(GROUND,torque,aBody,torque);
	sdbodyt(aBody,torque);
}
//_____________________________________________________________________________
/**
 * Apply a set of torques expressed in the inertial frame to a set of bodies.
 *
 * @param aN Number of Torques.
 * @param aBody Array of Body ID's.
 * @param aTorques Array of torques applied to the body expressed the inertial 
 * frame.
 */
void rdSDFast::
applyTorques(int aN,const int aBodies[],const double aTorques[][3])
{
	int i;
	for(i=0;i<aN;i++) {
		applyTorque(aBodies[i],aTorques[i]);
	}
}
//_____________________________________________________________________________
/**
 * Apply a set of torques expressed in the inertial frame to a set of bodies.
 *
 * @param aN Number of Torques.
 * @param aBody Array of Body ID's.
 * @param aTorques Array of torques applied to the body expressed the inertial 
 * frame.
 */
void rdSDFast::
applyTorques(int aN,const int aBodies[],const double *aTorques)
{

	// Check that input vector has been defined.
	if (aTorques==NULL) return;

	int i;
	double torque[3];

	for(i=0;i<aN;i++) {

		             torque[0] = *aTorques;
		aTorques++;  torque[1] = *aTorques;
		aTorques++;  torque[2] = *aTorques;

		applyTorque(aBodies[i],torque);
	}
}

//-----------------------------------------------------------------------------
// TORQUES EXPRESSED IN BODY-LOCAL FRAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a torque expressed in the body-local frame to a body.
 *
 * @param aBody Body ID.
 * @param aTorque Torque expressed in the body-local frame.
 */
void rdSDFast::
applyTorqueBodyLocal(int aBody,const double aTorque[3])
{
	// TO ALLOW CONSTANT ARGUMENTS
	int i;
	double torque[3];
	for(i=0;i<3;i++) {
		torque[i] = aTorque[i];
	}

	sdbodyt(aBody,torque);
}
//_____________________________________________________________________________
/**
 * Apply a set of torques expressed in the body-local frame to a set of bodies.
 *
 * @param aN Number of Torques.
 * @param aBody Array of Body ID's.
 * @param aTorques Array of torques applied to the body expressed the 
 * body-local frame.
 */
void rdSDFast::
applyTorquesBodyLocal(int aN,const int aBodies[],const double aTorques[][3])
{
	int i;
	for(i=0;i<aN;i++) {
		applyTorqueBodyLocal(aBodies[i],aTorques[i]);
	}
}
//_____________________________________________________________________________
/**
 * Apply a set of torques expressed in the body-local frame to a set of bodies.
 *
 * @param aN Number of Torques.
 * @param aBody Array of Body ID's.
 * @param aTorques Array of torques applied to the body expressed the 
 * body-local frame.
 */
void rdSDFast::
applyTorquesBodyLocal(int aN,const int aBodies[],const double *aTorques)
{

	// Check that input vector has been defined.
	if (aTorques==NULL) return;

	int i;
	double torque[3];

	for(i=0;i<aN;i++) {

		             torque[0] = *aTorques;
		aTorques++;  torque[1] = *aTorques;
		aTorques++;  torque[2] = *aTorques;

		applyTorqueBodyLocal(aBodies[i],torque);
	}
}

//-----------------------------------------------------------------------------
// GENERALIZED FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a generalized force to a generalized coordinate.
 * Note that depending on the axis type the generalized force can be a
 * torque or a force.
 * @param aU Generalized coordinate.
 * @param aF Applied force.
 */
void rdSDFast::
applyGeneralizedForce(int aU,double aF)
{
	int joint = _u2jMap[aU];
	int axis = _u2aMap[aU];
	sdhinget(joint,axis,aF);
}
//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * The dimension of aF is assumed to be the number of generalized speeds.
 * @param aF Applied force.
 */
void rdSDFast::
applyGeneralizedForces(const double aF[])
{
	int i;
	for(i=0;i<_u.getSize();i++) applyGeneralizedForce(i,aF[i]);
}
//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * The dimension of aF is assumed to be the number of generalized speeds.
 * @param aN Number of generalized forces.
 * @param aU Generalized coordinate.
 * @param aF Applied force.
 */
void rdSDFast::
applyGeneralizedForces(int aN,const int aU[],const double aF[])
{
	int i;
	for(i=0;i<aN;i++) applyGeneralizedForce(aU[i],aF[i]);
}

//-----------------------------------------------------------------------------
// LOAD ACCESS AND COMPUTATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the net applied generalized force.  The returned force is the sum of
 * all applied forces plus any forces needed for any prescribed motions.
 * The methods setState() (or equivalent) and computeAccelerations() must
 * be called prior to calling this method for the returned result to be
 * valid.
 *
 * @param aU Generalized speed (degree of freedom).
 * @return Net applied force/torque at degree of freedom indexed by aU.
 */
double rdSDFast::
getNetAppliedGeneralizedForce(int aU) const
{
	if((aU<0)||(aU>=getNumSpeeds())) return(0.0);

	double f;
	int joint = _u2jMap[aU];
	int axis = _u2aMap[aU];
	sdgetht(joint,axis,&f);
	return(f);
}
//_____________________________________________________________________________
/**
 * Compute the generalized forces necessary to achieve a set of specified
 * accelerations.  If any forces have been applied to the model, the balance
 * of generalized forces needed to achieve the desired accelerations is
 * computed.  Note that constraints are not taken into account by this
 * method.
 *
 * @param aDUDT Array of desired accelerations of the generalized coordinates-
 * should be dimensioned to NU (see getNumSpeeds()).
 * @param rF Array of generalized forces that will achieve aDUDT without
 * enforcing any constraints- should be dimensioned to NU (see getNumSpeeds()).
 */
void rdSDFast::
computeGeneralizedForces(double aDUDT[],double rF[]) const
{
	sdcomptrq(aDUDT,rF);
}
//_____________________________________________________________________________
/**
 * Compute the reaction forces and torques at all the joints in the model.
 *
 * It is necessary to call computeAccelerations() before this method
 * to get valid results.  This method is expensive to call, beyond the
 * expense of computing the accelerations.  So, this method should be
 * called as infrequently as possible.
 *
 * @param rForces Matrix of reaction forces.  The size should be
 * at least NumberOfJoints x 3.
 * @param rTorques Matrix of reaction torques.  The size should be
 * at least NumberOfJoints x 3.
 */
void rdSDFast::
computeReactions(double rForces[][3],double rTorques[][3]) const
{
	sdreac(rForces,rTorques);
}


//=============================================================================
// PRESCRIBED MOTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Turn on or off the prescribed motion of a joint axis.
 *
 * @param aJoint Number of the joint.
 * @param aAxis Axis number of the specified joint.
 * @param aPrescribed 0 turns prescribed motion off, 1 turns it on.
 */
void rdSDFast::
prescribeMotion(int aJoint,int aAxis,int aPrescribed)
{
	sdpres(aJoint,aAxis,aPrescribed);
}


//=============================================================================
// EQUATIONS OF MOTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Form the system mass matrix.
 *
 * @param rI Mass matrix (a square matrix of size NU*NU).
 */
void rdSDFast::
formMassMatrix(double *rI)
{
	sdmassmat(rI);
}
//_____________________________________________________________________________
/**
 * Form the transformation matrix E[3][3] that can be used to express the
 * angular velocity of a body in terms of the time derivatives of the euler
 * angles.  The Euler angle convention is body-fixed 1-2-3.
 * @param aBody Body ID.
 * @param rE Euler angles.
 */
void rdSDFast::
formEulerTransform(int aBody,double *rE) const
{
	if(rE==NULL) return;

	// GET ORIENTATION OF aBody
	double ang[3], dc[3][3];
	sdorient(aBody,dc);	
	sddc2ang(dc,&ang[0],&ang[1],&ang[2]);

	// ROW 1
	       *rE =  cos(ang[2])/cos(ang[1]);
	rE++;  *rE = -sin(ang[2])/cos(ang[1]);
	rE++;  *rE = 0.0;

	// ROW 2
	rE++;  *rE = sin(ang[2]);
	rE++;  *rE = cos(ang[2]);
	rE++;  *rE = 0.0;

	// ROW 3
	rE++;  *rE = -cos(ang[2])*sin(ang[1])/cos(ang[1]);
	rE++;  *rE =  sin(ang[1])*sin(ang[2])/cos(ang[1]);
	rE++;  *rE = 1.0;
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
void rdSDFast::
formJacobian(int aBody,double aPoint[3],double *rJ0)
{
	if(rJ0==NULL) return;

	int i,j,I;
	double trans[3],orien[3];
	for(i=0;i<getNumSpeeds();i++) {

		// GET COLUMN
		sdrel2cart(i,aBody,aPoint,trans,orien);

		// TRANSFORM TO GROUND FRAME
		sdtrans(aBody,trans,GROUND,trans);
		sdtrans(aBody,orien,GROUND,orien);

		// FORM MATRIX
		for(j=0;j<3;j++) {
			I = Mtx::ComputeIndex(j,_nu,i);
			rJ0[I] = trans[j];
			I = Mtx::ComputeIndex(j+3,_nu,i);
			rJ0[I] = orien[j];
		}
	}

	// PRINT
	//Mtx::Print(6,getNumSpeeds(),rJ0,3);
}
 */
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
void rdSDFast::
formJacobianEuler(int aBody,double *rJ)
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
	int I = Mtx::ComputeIndex(3,getNumSpeeds(),0);
	Mtx::Multiply(3,3,getNumSpeeds(),&E[0][0],&rJ[I],&rJ[I]);

	// PRINT
	//Mtx::Print(6,getNumSpeeds(),rJ,3);
}
 */
//_____________________________________________________________________________
/**
 * Form the full Jacobian matrix (J) for the translation of a point on a body.
 *
 * J is laid out as follows:
 *		dPx/dq1	dPx/dq2	dPx/dq3	...  dPx/dqnu
 *		dPy/dq1	dPy/dq2	dPy/dq3	...  dPy/dqnu
 *		dPz/dq1	dPz/dq2	dPz/dq3	...  dPz/dqnu
 *	where P is the point on the body.
 *
 * So, J should have 3 rows and NU columns.  In memory, the column index
 * increments fastest, so the representation is J[3][NU].
 *
 * It is assumed that enough space has been allocated at aJ to hold all
 * of the Jacobian elements.
 *
 * The Jacobian elements can be expressed in terms of the ground frame or
 * a particular body frame.  By default, the elements are expressed in the
 * ground frame.
 *
 * @param aBody Body on which the point resides.
 * @param aPoint Point position expressed in the local frame of body aBody.
 * @param rJ Jacobian of point aPoint.
 * @param aRefBody Body frame with respect to which the Jacobian elements
 * are expressed.  aRefBody has a default value of -1, which results in
 * the Jacobian elements being expressed in the ground frame.
 */
void rdSDFast::
formJacobianTranslation(int aBody,const double aPoint[3],double *rJ,
	int aRefBody) const
{
	if(aPoint==NULL) return;
	if(rJ==NULL) return;

	// REASSIGN POINT
	// Necessary only to have the const modifier for aPoint in the arg list.
	double point[3];
	point[0] = aPoint[0];
	point[1] = aPoint[1];
	point[2] = aPoint[2];

	// REFERENCE BODY FRAME
	if(aRefBody==-1) {
		aRefBody = getGroundID();
	}

	// FORM JACOBIAN
	int i,j,I;
	double trans[3],orien[3];
	for(i=0;i<getNumSpeeds();i++) {

		// GET COLUMN
		sdrel2cart(i,aBody,point,trans,orien);

		// TRANSFORM TO DESIRED REFERENCE FRAME
		sdtrans(aBody,trans,aRefBody,trans);

		// FORM MATRIX
		for(j=0;j<3;j++) {
			I = Mtx::ComputeIndex(j,_u.getSize(),i);
			rJ[I] = trans[j];
		}
	}

	// PRINT
	//Mtx::Print(6,getNumSpeeds(),rJ0,3);
}
//_____________________________________________________________________________
/**
 * Form the full Jacobian matrix (J0) for the orientation of a body.
 *
 * Note that J0 is not appropriate for operations when the body
 * orientation is specified in terms of Euler angles.  When the body
 * is described in terms of Euler angles, the method formJacobianEuler
 * should be used.
 *
 * J0 is laid out as follows:
 *		dOx/dq1	dOx/dq2	dOx/dq3	...  dOx/dqnu
 *		dOy/dq1	dOy/dq2	dOy/dq3	...  dOy/dqnu
 *		dOz/dq1	dOz/dq2	dOz/dq3	...  dOz/dqnu
 *	where O is the orientation of the body.
 *
 * So, JO should have 3 rows and NU columns.  In memory, the column index
 * increments fastest, so the representation is JO[3][NU].
 *
 * It is assumed that enough space has been allocated at rJO to hold all
 * of the Jacobian elements.
 *
 * The Jacobian elements can be expressed in terms of the ground frame or
 * a particular body frame.  By default, the elements are expressed in the
 * ground frame.
 *
 * @param aBody Body whose orientation Jacobian is desired.
 * @param rJO Orientation Jacobian.
 * @param aRefBody Body frame in which to express the Jacobian elements.
 * aRefBody has a default value of -1, which results in the Jacobian
 * elements being expressed in the ground frame.
 */
void rdSDFast::
formJacobianOrientation(int aBody,double *rJ,int aRefBody) const
{
	if(rJ==NULL) return;

	// REFERENCE BODY FRAME
	if(aRefBody==-1) {
		aRefBody = getGroundID();
	}

	// FORM JACOBIAN
	int i,j,I;
	double point[] = { 0.0, 0.0, 0.0 };
	double trans[3],orien[3];
	for(i=0;i<getNumSpeeds();i++) {

		// GET COLUMN
		sdrel2cart(i,aBody,point,trans,orien);

		// TRANSFORM TO DESIRED REFERENCE FRAME
		sdtrans(aBody,orien,aRefBody,orien);

		// FORM MATRIX
		for(j=0;j<3;j++) {
			I = Mtx::ComputeIndex(j,_u.getSize(),i);
			rJ[I] = orien[j];
		}
	}

	// PRINT
	//Mtx::Print(6,getNumSpeeds(),rJ0,3);
}
//_____________________________________________________________________________
/**
 * Form the full Jacobian matrix (JE) for the orientation of a body
 * expressed in terms of Euler angles.
 *
 * JE is laid out as follows:
 *		dEx/dq1	dEx/dq2	dEx/dq3	...  dEx/dqnu
 *		dEy/dq1	dEy/dq2	dEy/dq3	...  dEy/dqnu
 *		dEz/dq1	dEz/dq2	dEz/dq3	...  dEz/dqnu
 *	where E is the orientation of the body using 1-2-3 body-fixed Euler
 * angles.
 *
 * So, JE should have 3 rows and NU columns.  In memory, the column index
 * increments fastest, so the representation is JE[3][NU].
 *
 * It is assumed that enough space has been allocated at rJO to hold all
 * of the Jacobian elements.
 *
 * The Jacobian elements can be expressed in terms of the ground frame or
 * a particular body frame.  By default, the elements are expressed in the
 * ground frame.
 *
 * @param aBody Body whose orientation Jacobian is desired.
 * @param rJE Euler Jacobian.
 * @param aRefBody Body frame in which to express the Jacobian elements.
 * aRefBody has a default value of -1, which results in the Jacobian
 * elements being expressed in the ground frame.
 */
void rdSDFast::
formJacobianEuler(int aBody,double *rJE,int aRefBody) const
{
	// FORM J0
	formJacobianOrientation(aBody,rJE);

	// FORM E
	double E[3][3];
	formEulerTransform(aBody,&E[0][0]);
	//printf("\nSDFast.formJacobianEuler:\n");
	//Mtx::Print(3,3,&E[0][0],3);

	// TRANSFORM J0 TO JE
	Mtx::Multiply(3,3,getNumSpeeds(),&E[0][0],rJE,rJE);

	// PRINT
	//Mtx::Print(6,getNumSpeeds(),rJ,3);
}

//=============================================================================
// UTILITY
//=============================================================================
//-----------------------------------------------------------------------------
// ANGLES / QUATERIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Convert quaterions to angles.
 *
 * @param aQ Array of generalized coordinates, some of which may be
 * quaternions.  The length of aQ must be at least getNumCoordinates().
 * @param rQAng Array of equivalent angles.
 */
void rdSDFast::
convertQuaternionsToAngles(double *aQ,double *rQAng) const
{
	sdst2ang(aQ,rQAng);
}
//_____________________________________________________________________________
/**
 * For all the generalized coordinates held in a storage object, convert the
 * generalized coordinates expressed in quaternions to Euler angles.
 *
 * @param rQStore Storage object of generalized coordinates, some of which
 * may be quaternions.  The length of each state-vector in rQStore must be
 * at least getNumCoordinates().
 */
void rdSDFast::
convertQuaternionsToAngles(Storage *rQStore) const
{
	if(rQStore==NULL) return;

	// NUMBER OF Q'S
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	int dn = nq - nu;
	if(nq<=0) {
		printf("rdSDFast.convertQuaternionsToAngles: ERROR- models has ");
		printf("no generalized coordinates.\n");
		return;
	}

	// LOOP THROUGH STATE-VECTORS
	int i;
	int size,newSize=0;
	double t,*data,*newData=NULL;
	StateVector *vec;
	for(i=0;i<rQStore->getSize();i++) {

		// GET STATE-VECTOR
		vec = rQStore->getStateVector(i);
		if(vec==NULL) continue;

		// CHECK SIZE
		size = vec->getSize();
		if(size<nq) {
			printf("rdSDFast.convertQuaternionsToAngles: WARN- the size of ");
			printf("a state-vector is less than nq(%d).\n",nq);
			continue;
		}

		// GET DATA
		t = vec->getTime();
		data = vec->getData().get();
		if(data==NULL) continue;

		// ALLOCATE NEW DATA IF NECESSARY
		if(newSize<(size-dn)) {
			if(newData!=NULL) delete[] newData;
			newSize = size-dn;
			newData = new double[newSize];
		}

		// CONVERT QUATERNIONS TO ANGLES
		convertQuaternionsToAngles(data,newData);

		// FILL IN THE REST OF THE DATA
		for(i=nu;i<(size-dn);i++) {
			newData[i] = data[i+dn];
		}

		// CHANGE THE STATE-VECTOR
		vec->setStates(t,newSize,newData);
	}

	// CHANGE THE COLUMN LABELS
	cout<<"rdSDFast.convertQuaternionsToAngles: NOTE- the column labels"<<
		" for "<<rQStore->getName()<<" were not changed."<<endl;

	// CLEANUP
	if(newData!=NULL) delete[] newData;
}
//_____________________________________________________________________________
/**
 * Convert angles to quaterions.
 *
 * @param aQAng Array of generalized coordinates expressed in Euler angles.
 * The length of aQAng must be at least getNumSpeeds().
 * @param rQ Vector of equivalent quaternions.
 */
void rdSDFast::
convertAnglesToQuaternions(double *aQAng,double *rQ) const
{
	sdang2st(aQAng,rQ);
}
//_____________________________________________________________________________
/**
 * For all the generalized coordinates held in a storage object, convert the
 * generalized coordinates expressed in Euler angles to quaternions when
 * appropriate.
 *
 * @param rQStore Storage object of generalized coordinates that has all
 * angles expressed as Euler angles in radians.  The length of each
 * state-vector in rQStore must be at least getNumSpeeds().
 */
void rdSDFast::
convertAnglesToQuaternions(Storage *rQStore) const
{
	if(rQStore==NULL) return;

	// NUMBER OF Q'S
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	int dn = nq - nu;
	if(nu<=0) {
		printf("rdSDFast.convertAnglesToQuaternions: ERROR- models has ");
		printf("no generalized coordinates.\n");
		return;
	}
	if(dn<=0) return;

	// LOOP THROUGH THE STATE-VECTORS
	int i,j;
	int size,newSize=0;
	double t,*data,*newData=NULL;
	StateVector *vec;
	for(i=0;i<rQStore->getSize();i++) {

		// GET STATE-VECTOR
		vec = rQStore->getStateVector(i);
		if(vec==NULL) continue;

		// CHECK SIZE
		size = vec->getSize();
		if(size<nu) {
			printf("rdSDFast.convertAnglesToQuaternions: WARN- the size of ");
			printf("a state-vector is less than nu(%d).\n",nu);
			continue;
		}

		// GET DATA
		t = vec->getTime();
		data = vec->getData().get();
		if(data==NULL) continue;

		// ALLOCATE NEW DATA IF NECESSARY
		if(newSize<(size+dn)) {
			if(newData!=NULL) delete[] newData;
			newSize = size+dn;
			newData = new double[newSize];
		}

		// CONVERT QUATERNIONS TO ANGLES
		convertAnglesToQuaternions(data,newData);

		// FILL IN THE REST OF THE DATA
		for(j=nu;j<size;j++) {
			newData[j+dn] = data[j];
		}

		// CHANGE THE STATE-VECTOR
		vec->setStates(t,newSize,newData);
	}

	// CHANGE THE COLUMN LABELS
	cout<<"rdSDFast.convertAnglesToQuaternions: NOTE- the column labels"<<
		" for "<<rQStore->getName()<<" were not changed."<<endl;

	// CLEANUP
	if(newData!=NULL) delete[] newData;
}

//-----------------------------------------------------------------------------
// DEGREES/RADIANS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Convert the rotational generalized coordinates or speeds from units of
 * radians to units of degrees.
 *
 * @param aQRad Array in radians.  The length of aQRad should be
 * at least the number of generalized speeds (@see getNumSpeeds()).
 * @param rQDeg Array in degrees.  The length of rQRad should be
 * at least the number of generalized speeds (@see getNumSpeeds()).
 */
void rdSDFast::
convertRadiansToDegrees(double *aQRad,double *rQDeg) const
{
	// LOOP OVER THE JOINTS
	int i,j,u;
	int info[50],slider[6];
	for(u=i=0;i<getNumJoints();i++) {

		// GET INFO
		sdjnt(i,info,slider);

		// LOOP OVER DOF
		int ndof = info[4];
		for(j=0;j<ndof;j++,u++) {
			if(slider[j]==0) {
				rQDeg[u] = rdMath::RTD * aQRad[u]; 
			} else {
				rQDeg[u] = aQRad[u];
			}
		}
	}
}
//_____________________________________________________________________________
/**
 * Convert the rotational generalized coordinates or speeds from units of
 * radians to units of degrees for all the state-vectors in an Storage
 * object.
 *
 * It is assumed that the first first getNumSpeeds() elements of each state-vector
 * held in the Storage object are the generalized coordinates or speeds.
 *
 * @param rQStore Storage object.
 */
void rdSDFast::
convertRadiansToDegrees(Storage *rQStore) const
{
	if(rQStore==NULL) return;

	// LOOP THROUGH THE STATEVECTORS
	int i,size;
	int nu=getNumSpeeds();
	double *u;
	StateVector *vec;
	for(i=0;i<rQStore->getSize();i++) {

		// STATE-VECTOR
		vec = rQStore->getStateVector(i);
		if(vec==NULL) {
			printf("rdSDFast.convertRadiansToDegrees: WARN- null");
			printf("state-vector.\n");			
			continue;
		}

		// CHECK SIZE
		size = vec->getSize();
		if(size<nu) {
			printf("rdSDFast.convertRadiansToDegrees: WARN- state-vector ");
			printf("has fewer than %d states.\n",nu);
			continue;
		}

		// GET POINTER TO DATA
		u = vec->getData().get();
		if(u==NULL) continue;

		// CONVERT DATA TO DEGREES
		convertRadiansToDegrees(u,u);
	}
}

//_____________________________________________________________________________
/**
 * Convert the rotational generalized coordinates or speeds from units of
 * degrees to units of radiants.
 * @param aQDeg Array in degrees.  The length of aQDeg should be
 * at least the number of generalized speeds (@see getNumSpeeds()).
 * @param rQRad Array in radians.  The length of rQRad should be
 * at least the number of generalized speeds (@see getNumSpeeds()).
 */
void rdSDFast::
convertDegreesToRadians(double *aQDeg,double *rQRad) const
{
	// LOOP OVER THE JOINTS
	int i,j,u;
	int info[50],slider[6];
	for(u=i=0;i<getNumJoints();i++) {

		// GET INFO
		sdjnt(i,info,slider);

		// LOOP OVER DOF
		int ndof = info[4];
		for(j=0;j<ndof;j++,u++) {
			if(slider[j]==0) {
				rQRad[u] = rdMath::DTR * aQDeg[u]; 
			} else {
				rQRad[u] = aQDeg[u];
			}
		}
	}
}
//_____________________________________________________________________________
/**
 * Convert the rotational generalized coordinates or speeds from units of
 * degrees to units of radians for all the state-vectors in an Storage
 * object.
 *
 * It is assumed that the first first getNumSpeeds() elements of each state-vector
 * held in the Storage object are the generalized coordinates or speeds.
 *
 * @param rQStore Storage object.
 */
void rdSDFast::
convertDegreesToRadians(Storage *rQStore) const
{
	if(rQStore==NULL) return;

	// LOOP THROUGH THE STATEVECTORS
	int i,size;
	int nu=getNumSpeeds();
	double *u;
	StateVector *vec;
	for(i=0;i<rQStore->getSize();i++) {

		// STATE-VECTOR
		vec = rQStore->getStateVector(i);
		if(vec==NULL) {
			printf("rdSDFast.convertDegreesToRadians: WARN- null");
			printf("state-vector.\n");			
			continue;
		}

		// CHECK SIZE
		size = vec->getSize();
		if(size<nu) {
			printf("rdSDFast.convertDegreesToRadians: WARN- state-vector ");
			printf("has fewer than %d states.\n",nu);
			continue;
		}

		// GET POINTER TO DATA
		u = vec->getData().get();
		if(u==NULL) continue;

		// CONVERT DATA TO DEGREES
		convertDegreesToRadians(u,u);
	}
}

//-----------------------------------------------------------------------------
// ANGLES / DIRECTION COSINES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Convert angles to direction cosines.
 * @param aE1 1st Euler angle.
 * @param aE2 2nd Euler angle.
 * @param aE3 3rd Euler angle.
 * @param rDirCos Vector of direction cosines.
 */
void rdSDFast::
convertAnglesToDirectionCosines(double aE1,double aE2,double aE3,
	double rDirCos[3][3]) const
{
	sdang2dc(aE1,aE2,aE3,rDirCos);
}
//_____________________________________________________________________________
/**
 * Convert angles to direction cosines.
 * @param aE1 1st Euler angle.
 * @param aE2 2nd Euler angle.
 * @param aE3 3rd Euler angle.
 * @param rDirCos Vector of direction cosines.
 */
void rdSDFast::
convertAnglesToDirectionCosines(double aE1,double aE2,double aE3,
	double *rDirCos) const
{
	if (rDirCos==NULL) return;

	double dirCos[3][3];

	sdang2dc(aE1,aE2,aE3,dirCos);

	// Assign the returned values to the return vector.
	memcpy(rDirCos,&dirCos[0][0],9*sizeof(double));

}
//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void rdSDFast::
convertDirectionCosinesToAngles(double aDirCos[3][3],
	double *rE1,double *rE2,double *rE3) const
{
	sddc2ang(aDirCos,rE1,rE2,rE3);
}
//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void rdSDFast::
convertDirectionCosinesToAngles(double *aDirCos,
	double *rE1,double *rE2,double *rE3) const
{
	if (aDirCos==NULL) return;

	double dirCos[3][3];

	// Copy the input vector to the local one.
	memcpy(&dirCos[0][0],aDirCos,9*sizeof(double));

	sddc2ang(dirCos,rE1,rE2,rE3);
}
//_____________________________________________________________________________
/**
 * Convert direction cosines to quaternions.
 * @param aDirCos Vector of direction cosines.
 * @param rQ1 1st Quaternion.
 * @param rQ2 2nd Quaternion.
 * @param rQ3 3rd Quaternion.
 * @param rQ4 4th Quaternion.
 */
void rdSDFast::
convertDirectionCosinesToQuaternions(double aDirCos[3][3],
	double *rQ1,double *rQ2,double *rQ3,double *rQ4) const
{
	sddc2quat(aDirCos,rQ1,rQ2,rQ3,rQ4);
}
//_____________________________________________________________________________
/**
 * Convert direction cosines to quaternions.
 * @param aDirCos Vector of direction cosines.
 * @param rQ1 1st Quaternion.
 * @param rQ2 2nd Quaternion.
 * @param rQ3 3rd Quaternion.
 * @param rQ4 4th Quaternion.
 */
void rdSDFast::
convertDirectionCosinesToQuaternions(double *aDirCos,
	double *rQ1,double *rQ2,double *rQ3,double *rQ4) const
{
	if (aDirCos==NULL) return;

	double dirCos[3][3];

	// Copy the input vector to the local one.
	memcpy(&dirCos[0][0],aDirCos,9*sizeof(double));

	sddc2quat(dirCos,rQ1,rQ2,rQ3,rQ4);
}
//_____________________________________________________________________________
/**
 * Convert quaternions to direction cosines.
 * @param aQ1 1st Quaternion.
 * @param aQ2 2nd Quaternion.
 * @param aQ3 3rd Quaternion.
 * @param aQ4 4th Quaternion.
 * @param rDirCos Vector of direction cosines.
 */
void rdSDFast::
convertQuaternionsToDirectionCosines(double aQ1,double aQ2,double aQ3,
	double aQ4,double rDirCos[3][3]) const
{
	sdquat2dc(aQ1,aQ2,aQ3,aQ4,rDirCos);
}
//_____________________________________________________________________________
/**
 * Convert quaternions to direction cosines.
 * @param aQ1 1st Quaternion.
 * @param aQ2 2nd Quaternion.
 * @param aQ3 3rd Quaternion.
 * @param aQ4 4th Quaternion.
 * @param rDirCos Vector of direction cosines.
 */
void rdSDFast::
convertQuaternionsToDirectionCosines(double aQ1,double aQ2,double aQ3,
	double aQ4,double *rDirCos) const
{
	if (rDirCos==NULL) return;

	double dirCos[3][3];

	sdquat2dc(aQ1,aQ2,aQ3,aQ4,dirCos);

	// Assign the returned values to the return vector.
	memcpy(rDirCos,&dirCos[0][0],9*sizeof(double));
}
//_____________________________________________________________________________
/**
 * Register the types of objects used by rdSDFast and that need to be registered 
 * for xml serialization purposes
 *
 */
void rdSDFast::
RegisterTypes()
{	
	Model::RegisterTypes();

}


//=============================================================================
// SDFAST STATIC METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply user-supplied forces to the model.
 *
 * This static method is intended to be called from the rdSDFast function
 * sduforce().  sduforce() is a "C" function and, therefore, doesn't know
 * about C++ objects.  Internally, this method uses a static pointer
 * to call the virtual methods Model::applyActuaion() and
 * Model::applyContact().  This method assumes that computeActuation()
 * and computeContact() have already been called.
 */
void rdSDFast::
SDUForce()
{
	cout<<"\n\nSDFast::SDUForce: ... rdSDFast has called sudforce()!\n\n";

	if(_Instance==NULL) {
		cout<<"\n\nSDFast.SDUForce: ERR- no valid rdSDFast object.\n\n";
		return;
	}

	_Instance->applyActuatorForces();
	_Instance->applyContactForces();
}

