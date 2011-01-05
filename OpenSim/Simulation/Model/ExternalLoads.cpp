// ExternalLoads.cpp
// Authors: Frank C. Anderson, Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <algorithm>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Common/GCVSpline.h>
#include "ForceSet.h"
#include "Model.h"
#include "Muscle.h"
#include "PrescribedForce.h"
#include "ExternalLoads.h"

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ExternalLoads::~ExternalLoads()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ExternalLoads::ExternalLoads():
_dataFileName(_dataFileNameProp.getValueStr()),
_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();
}

ExternalLoads::ExternalLoads(Model& model) : 
	ForceSet(model),
	_dataFileName(_dataFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Construct an actuator set from file.
 *
 * @param aFileName Name of the file.
 */
ExternalLoads::ExternalLoads(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode) :
	ForceSet(model, aFileName, false),
	_dataFileName(_dataFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();

	if(aUpdateFromXMLNode)
		updateFromXMLNode();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForceSet ForceSet to be copied.
 */
ExternalLoads::ExternalLoads(const ExternalLoads &aExternalLoads) :
	ForceSet(aExternalLoads),
	_dataFileName(_dataFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();

	// Class Members
	copyData(aExternalLoads);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this ExternalLoads to their null values.
 */
void ExternalLoads::setNull()
{
	// TYPE
	setType("ExternalLoads");
	// NAME
	//setName("ExternalLoads");

	// PROPERTIES
	setupSerializedMembers();

	_actuators.setMemoryOwner(false);

	_muscles.setMemoryOwner(false);
}

//_____________________________________________________________________________
/**
 * Copy this ExternalLoads and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ExternalLoads.
 */
Object* ExternalLoads::copy() const
{
	ExternalLoads *actSet = new ExternalLoads(*this);
	return(actSet);
}

//_____________________________________________________________________________
/**
 * Copy the member variables of the ExternalLoads.
 *
 * @param aAbsForceSet actuator set to be copied
 */
void ExternalLoads::copyData(const ExternalLoads &aAbsExternalLoads)
{
    // ACTUATORS
	_dataFileName = aAbsExternalLoads._dataFileName;
	_externalLoadsModelKinematicsFileName = aAbsExternalLoads._externalLoadsModelKinematicsFileName;
	_lowpassCutoffFrequencyForLoadKinematics = aAbsExternalLoads._lowpassCutoffFrequencyForLoadKinematics;
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ExternalLoads::setupSerializedMembers()
{
	string comment;
	_dataFileNameProp.setName("datafile");
	_dataFileName="";
	comment =	"Storage file (.sto) containing the Forces, Torques and locations of the external loads.";
	_dataFileNameProp.setComment(comment);
	_propertySet.append(&_dataFileNameProp);

	comment =	"Motion file (.mot) or storage file (.sto) containing the model kinematics "
					"corresponding to the external loads.";
	_externalLoadsModelKinematicsFileNameProp.setComment(comment);
	_externalLoadsModelKinematicsFileNameProp.setName("external_loads_model_kinematics_file");
	_propertySet.append( &_externalLoadsModelKinematicsFileNameProp );

	comment = "Low-pass cut-off frequency for filtering the model kinematics corresponding "
				 "to the external loads. A negative value results in no filtering. "
				 "The default value is -1.0, so no filtering.";
	_lowpassCutoffFrequencyForLoadKinematicsProp.setComment(comment);
	_lowpassCutoffFrequencyForLoadKinematicsProp.setName("lowpass_cutoff_frequency_for_load_kinematics");
	_propertySet.append( &_lowpassCutoffFrequencyForLoadKinematicsProp );

}

void ExternalLoads::setup(Model& aModel)
{
	// BASE CLASS
	ForceSet::setup(aModel);
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
ExternalLoads& ExternalLoads::operator=(const ExternalLoads &aAbsForceSet)
{
	// BASE CLASS
	ForceSet::operator=(aAbsForceSet);

	// Class Members
	copyData(aAbsForceSet);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Create a set of prescribed forces from a file.
 * Assumptions:
 *  0. If instance variable _dataFileName is not set already it will be set
 *  1. Sizes of the three arrays startForceColumns, bodyNames, columnCount is identical
 *  2. columnCount is 
 *						9: Force, Point, Torque 
 *						6: Force, Point
 */
void ExternalLoads::createForcesFromFile(const std::string& fileName,
									Array<std::string>& startForceColumns, 
									Array<int>& columnCount,
									Array<std::string>& bodyNames)
{
	setDataFileName(fileName);
	Storage kineticsStore(fileName);
	int forceSize = kineticsStore.getSize();
	if(forceSize<=0) return;

	assert(startForceColumns.getSize()==columnCount.getSize());
	assert(startForceColumns.getSize()==bodyNames.getSize());
	double *t=0;
	kineticsStore.getTimeColumn(t);
	const Array<string>& lbls=kineticsStore.getColumnLabels();

	if (startForceColumns.getSize()==0){	
		// User didn't specify,
		// We'll assume 9 columns for force, point, torque
		int nColumns = lbls.getSize()-1;
		int nForces = nColumns/9;
		for(int i=0; i<nForces; i++){
			startForceColumns.append(lbls[i*9+1]);
			columnCount.append(9);
			bodyNames.append("ground");
		}
	}
	// Make sure that column names are unique, otherwise assert
	// Sort names and then do one pass for more intelligent name finding.
	// This need to be made more efficient using some stl sorting implementation
	bool duplicateIsFound = false;
	std::string duplicateName="";
	for(int i=0; i< startForceColumns.getSize()-1 && !duplicateIsFound; i++){
		for(int j=i+1; j< startForceColumns.getSize()-1 && !duplicateIsFound; i++){
			duplicateIsFound = (startForceColumns.get(i)==startForceColumns.get(j));
			if (duplicateIsFound) duplicateName = startForceColumns.get(i);
		}
	}
	if (duplicateIsFound){
		string msg = "Create forces from file "+fileName+", duplicate column "
			+duplicateName+" found.\nOperation is aborted, please have unique column names and retry.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	{
		double *column1=0;
		double *column2=0;
		double *column3=0;

		for(int i=0; i< startForceColumns.getSize(); i++){
			std::string labelX = startForceColumns[i];
			int storageIndex=lbls.findIndex(labelX);

			kineticsStore.getDataColumn(labelX, column1);
			std::string labelY = lbls[storageIndex+1];
			kineticsStore.getDataColumn(labelY, column2);
			std::string labelZ = lbls[storageIndex+2];
			kineticsStore.getDataColumn(labelZ, column3);

			PrescribedForce* pf= new PrescribedForce();
			pf->setBodyName(bodyNames[i]);
			// If columnCount is 3 then create the forces and we're done
			NaturalCubicSpline* spline1= new NaturalCubicSpline(forceSize, t, column1, labelX);
			NaturalCubicSpline* spline2= new NaturalCubicSpline(forceSize, t, column2, labelY);
			NaturalCubicSpline* spline3= new NaturalCubicSpline(forceSize, t, column3, labelZ);
			delete column1, column2, column3;
			column1 = column2 = column3 = 0;
			//
			pf->setForceFunctions(spline1, spline2, spline3);	// Copies of these functions are made 

			if (columnCount[i]>=6){
				std::string labelX = lbls[storageIndex+3];
				kineticsStore.getDataColumn(labelX, column1);
				std::string labelY = lbls[storageIndex+4];
				kineticsStore.getDataColumn(labelY, column2);
				std::string labelZ = lbls[storageIndex+5];
				kineticsStore.getDataColumn(labelZ, column3);

				// If columnCount is 3 then create the forces and we're done
				NaturalCubicSpline* spline1= new NaturalCubicSpline(forceSize, t, column1, labelX);
				NaturalCubicSpline* spline2= new NaturalCubicSpline(forceSize, t, column2, labelY);
				NaturalCubicSpline* spline3= new NaturalCubicSpline(forceSize, t, column3, labelZ);

				delete column1, column2, column3;
				column1 = column2 = column3 = 0;
				pf->setPointFunctions(spline1, spline2, spline3); 
			}
			if (columnCount[i]==9){
				std::string labelX = lbls[storageIndex+6];
				kineticsStore.getDataColumn(labelX, column1);
				std::string labelY = lbls[storageIndex+7];
				kineticsStore.getDataColumn(labelY, column2);
				std::string labelZ = lbls[storageIndex+8];
				kineticsStore.getDataColumn(labelZ, column3);

				// If columnCount is 3 then create the forces and we're done
				NaturalCubicSpline* spline1= new NaturalCubicSpline(forceSize, t, column1, labelX);
				NaturalCubicSpline* spline2= new NaturalCubicSpline(forceSize, t, column2, labelY);
				NaturalCubicSpline* spline3= new NaturalCubicSpline(forceSize, t, column3, labelZ);

				delete column1, column2, column3;
				column1 = column2 = column3 = 0;
				pf->setTorqueFunctions(spline1, spline2, spline3); 
			}
			append(pf);
		}
	}
	print("externalForces.xml");
}
//-----------------------------------------------------------------------------
// COMPUTE POSITION FUNCTIONS 
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the position and velocity functions that set the position and
 * velocity of the point at which the linear spring applies its force.
 * This method takes the time histories of a point's
 * position and velocity in the inertial frame and converts them to the local
 * (body) frame.
 *
 * @param aQStore Storage containing the time history of generalized
 * coordinates for the model. Note that all generalized coordinates must
 * be specified and in radians and Euler parameters.
 * @param aUStore Storage containing the time history of generalized
 * speeds for the model.  Note that all generalized speeds must
 * be specified and in radians.
 * 
 * @kineticsStore Storage containing the time history for forces.
 */
void ExternalLoads::computeFunctions(SimTK::State& s, 
                                double startTime,
                                double endTime, 
								const Storage& kineticsStore, 
								Storage* aQStore, 
								Storage* aUStore)
{
	// If force is global but applied to non-ground body we need to transfrom to
	// correct frame. 
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int size = (aQStore!=NULL)?aQStore->getSize():0;
	int forceSize = kineticsStore.getSize();
	int startIndex=0;
	int lastIndex=size;
	if (aQStore != NULL){
		if (startTime!= -SimTK::Infinity){	// Start time was actually specified.
			startIndex = aQStore->findIndex(startTime); 
		}
		if (endTime!= SimTK::Infinity){	// Start time was actually specified.
			lastIndex = aQStore->findIndex(endTime);
		}
	}
	// Cycle thru forces to check if xform is needed
	for(int f=0; f <getSize(); f++){
		Force& nextForce = get(f);
		PrescribedForce* pf = dynamic_cast<PrescribedForce*>(&nextForce);
		if (pf){
			// Now we have a PrescribedForce
			// We can set ForceFunctions and TorqueFunctions directly
			if (pf->getPointFunctions().getSize()==3){
				if (pf->getPointIsInGlobalFrame()){
					double *t=0,*x=0,*y=0,*z=0;
					kineticsStore.getTimeColumn(t);
					const FunctionSet& ffSet=pf->getPointFunctions();
					kineticsStore.getDataColumn(ffSet[0].getName(), x);
					kineticsStore.getDataColumn(ffSet[1].getName(), y);
					kineticsStore.getDataColumn(ffSet[2].getName(), z);
					GCVSpline *localrtFx, *localrtFy, *localrtFz;
					localrtFx = new GCVSpline( 3, forceSize, t, x );
					localrtFy = new GCVSpline( 3, forceSize, t, y );
					localrtFz = new GCVSpline( 3, forceSize, t, z );
					pf->setPointFunctions( localrtFx, localrtFy, localrtFz  );
				}
				else{ // need to xform point to proper frame utilizing
					Array<double> localt(0.0,1);
					Array<double> localy(0.0,nq+nu);
					Vec3 originGlobal(0.0),origin(0.0);
					Vec3 pGlobal(0.0, 0.0, 0.0);
					Vec3 pLocal(0.0);
					Vec3 vGlobal(0.0),vLocal(0.0);
					Storage pStore,vStore;
					const FunctionSet& ffSet=pf->getPointFunctions();
					double *t=0,*x=0,*y=0,*z=0;
					VectorGCVSplineR1R3 *aGlobal;
					kineticsStore.getTimeColumn(t);
					kineticsStore.getDataColumn(ffSet[0].getName(),x);
					kineticsStore.getDataColumn(ffSet[1].getName(),y);
					kineticsStore.getDataColumn(ffSet[2].getName(),z);
					aGlobal = new VectorGCVSplineR1R3(3,forceSize,t,x,y,z);
					if (aQStore!= NULL){	// Kinematics specified
						for(int i=startIndex;i<lastIndex;i++) {
							// Set the model state
							aQStore->getTime(i,*(&localt[0]));
							aQStore->getData(i,nq,&localy[0]);
							aUStore->getData(i,nu,&localy[nq]);
							for (int j = 0; j < nq; j++) {
								Coordinate& coord = _model->getCoordinateSet().get(j);
								coord.setValue(s, localy[j], j==nq-1);
								coord.setSpeedValue(s, localy[nq+j]);
							}

							// Position in local frame (i.e. with respect to body's origin, not center of mass)
							_model->getSimbodyEngine().getPosition(s,pf->getBody(),origin,originGlobal);
							aGlobal->calcValue(&localt[0],&pGlobal[0], localt.getSize());
							pLocal=pGlobal-originGlobal; 
							_model->getSimbodyEngine().transform(s,_model->getSimbodyEngine().getGroundBody(),&pLocal[0],pf->getBody(),&pLocal[0]);
							pStore.append(localt[0],3,&pLocal[0]);
						}
					}
					else
						pStore = kineticsStore;
					// CREATE POSITION FUNCTION
					double *time=NULL;
					double *p0=0,*p1=0,*p2=0;
					int padSize = size / 4;
					if(padSize>100) padSize = 100;
					pStore.pad(padSize);
					size = pStore.getTimeColumn(time);
					pStore.getDataColumn(0,p0);
					pStore.getDataColumn(1,p1);
					pStore.getDataColumn(2,p2);
					GCVSpline *xFunc = new GCVSpline(3,size,time,p0);
					GCVSpline *yFunc = new GCVSpline(3,size,time,p1);
					GCVSpline *zFunc = new GCVSpline(3,size,time,p2);
					pf->setPointFunctions(xFunc, yFunc, zFunc);
					delete[] time;
					delete[] p0;
					delete[] p1;
					delete[] p2;

				}
			}
			if (pf->getForceFunctions().getSize()==3){
				double *t=0,*x=0,*y=0,*z=0;
				kineticsStore.getTimeColumn(t);
				const FunctionSet& ffSet=pf->getForceFunctions();
				kineticsStore.getDataColumn(ffSet[0].getName(), x);
				kineticsStore.getDataColumn(ffSet[1].getName(), y);
				kineticsStore.getDataColumn(ffSet[2].getName(), z);
				GCVSpline *localrtFx, *localrtFy, *localrtFz;
				localrtFx = new GCVSpline( 3, forceSize, t, x );
				localrtFy = new GCVSpline( 3, forceSize, t, y );
				localrtFz = new GCVSpline( 3, forceSize, t, z );
				pf->setForceFunctions( localrtFx, localrtFy, localrtFz  );
			}
			if (pf->getTorqueFunctions().getSize()==3){
				double *t=0,*x=0,*y=0,*z=0;
				kineticsStore.getTimeColumn(t);
				const FunctionSet& ffSet=pf->getTorqueFunctions();
				kineticsStore.getDataColumn(ffSet[0].getName(), x);
				kineticsStore.getDataColumn(ffSet[1].getName(), y);
				kineticsStore.getDataColumn(ffSet[2].getName(), z);
				GCVSpline *localrtFx, *localrtFy, *localrtFz;
				localrtFx = new GCVSpline( 3, forceSize, t, x );
				localrtFy = new GCVSpline( 3, forceSize, t, y );
				localrtFz = new GCVSpline( 3, forceSize, t, z );
				pf->setTorqueFunctions( localrtFx, localrtFy, localrtFz  );
			}
		}
	}
}

//_____________________________________________________________________________
/**
 * Compute the position and velocity functions that set the position and
 * velocity of the point at which the linear spring applies its force.
 * This method takes the time histories of a point's
 * position and velocity in the inertial frame and converts them to the local
 * (body) frame.
 *
 * @param aQStore Storage containing the time history of generalized
 * coordinates for the model. Note that all generalized coordinates must
 * be specified and in radians and Euler parameters.
 * @param aUStore Storage containing the time history of generalized
 * speeds for the model.  Note that all generalized speeds must
 * be specified and in radians.
 * @param aPStore Storage containing the time history of the position at
 * which the force is to be applied in the global frame.
 * DEPRECATED
 */
void ExternalLoads::computePointFunctions(SimTK::State& s, 
                     double startTime,
                     double endTime, 
                     const OpenSim::Body& body,
                     const Storage& aQStore,
                     const Storage& aUStore,
                     VectorGCVSplineR1R3& aPGlobal,
                     GCVSpline*& xFunc, 
                     GCVSpline*& yFunc, 
                     GCVSpline*& zFunc)
{
	int i;
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int size = aQStore.getSize();
	Array<double> t(0.0,1);
	Array<double> y(0.0,nq+nu);
	Vec3 originGlobal(0.0),origin(0.0);
	Vec3 pGlobal(0.0, 0.0, 0.0);
	Vec3 pLocal(0.0);
	Vec3 vGlobal(0.0),vLocal(0.0);
	Storage pStore,vStore;
	int startIndex=0;
	int lastIndex=size;
	if (startTime!= -SimTK::Infinity){	// Start time was actually specified.
		startIndex = aQStore.findIndex(startTime); 
	}
	if (endTime!= SimTK::Infinity){	// Start time was actually specified.
		lastIndex = aQStore.findIndex(endTime);
	}
	for(i=startIndex;i<lastIndex;i++) {
		// Set the model state
		aQStore.getTime(i,*(&t[0]));
		aQStore.getData(i,nq,&y[0]);
		aUStore.getData(i,nu,&y[nq]);
        for (int j = 0; j < nq; j++) {
    		Coordinate& coord = _model->getCoordinateSet().get(j);
            coord.setValue(s, y[j], j==nq-1);
            coord.setSpeedValue(s, y[nq+j]);
        }

		// Position in local frame (i.e. with respect to body's origin, not center of mass)
		_model->getSimbodyEngine().getPosition(s,body,origin,originGlobal);
		aPGlobal.calcValue(&t[0],&pGlobal[0], t.getSize());
		pLocal=pGlobal-originGlobal; //Mtx::Subtract(1,3,&pGlobal[0],&originGlobal[0],&pLocal[0]);
		_model->getSimbodyEngine().transform(s,_model->getSimbodyEngine().getGroundBody(),&pLocal[0],body,&pLocal[0]);
		pStore.append(t[0],3,&pLocal[0]);
	}

	// CREATE POSITION FUNCTION
	double *time=NULL;
	double *p0=0,*p1=0,*p2=0;
	int padSize = size / 4;
	if(padSize>100) padSize = 100;
	pStore.pad(padSize);
	size = pStore.getTimeColumn(time);
	pStore.getDataColumn(0,p0);
	pStore.getDataColumn(1,p1);
	pStore.getDataColumn(2,p2);
	xFunc = new GCVSpline(3,size,time,p0);
	yFunc = new GCVSpline(3,size,time,p1);
	zFunc = new GCVSpline(3,size,time,p2);
	delete[] time;
	delete[] p0;
	delete[] p1;
	delete[] p2;

}