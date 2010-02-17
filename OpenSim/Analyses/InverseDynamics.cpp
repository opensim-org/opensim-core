// InverseDynamics.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Eran Guendelman, Jeff Reinbolt
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
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
#include <string>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/OpenSimForceSubsystem.h>
#include <OpenSim/Simulation/Model/CustomForce.h>
#include <SimTKmath.h>
#include <SimTKlapack.h>
#include "InverseDynamics.h"



using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
InverseDynamics::~InverseDynamics()
{
	deleteStorage();
	delete _modelWorkingCopy;
	if(_ownsForceSet) delete _forceSet;
}
//_____________________________________________________________________________
/**
 */
InverseDynamics::InverseDynamics(Model *aModel) :
	Analysis(aModel),
	_useModelForceSet(_useModelForceSetProp.getValueBool()),
	_modelWorkingCopy(NULL),
	_numCoordinateActuators(0)
{
	setNull();

	if(aModel) {
		setModel(*aModel);
	}
	else allocateStorage();
}
// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
InverseDynamics::InverseDynamics(const InverseDynamics &aInverseDynamics):
	Analysis(aInverseDynamics),
	_useModelForceSet(_useModelForceSetProp.getValueBool()),
	_modelWorkingCopy(NULL),
	_numCoordinateActuators(aInverseDynamics._numCoordinateActuators)
{
	setNull();
	// COPY TYPE AND NAME
	*this = aInverseDynamics;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* InverseDynamics::copy() const
{
	InverseDynamics *object = new InverseDynamics(*this);
	return(object);

}
//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
InverseDynamics& InverseDynamics::
operator=(const InverseDynamics &aInverseDynamics)
{
	// BASE CLASS
	Analysis::operator=(aInverseDynamics);

	_useModelForceSet = aInverseDynamics._useModelForceSet;
	_modelWorkingCopy = aInverseDynamics._modelWorkingCopy;
	_numCoordinateActuators = aInverseDynamics._numCoordinateActuators;
	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void InverseDynamics::
setNull()
{
	setupProperties();

	// OTHER VARIABLES
	_useModelForceSet = true;
	_storage = NULL;
	_ownsForceSet = false;
	_forceSet = NULL;
	_numCoordinateActuators = 0;

	setType("InverseDynamics");
	setName("InverseDynamics");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void InverseDynamics::
setupProperties()
{
	_useModelForceSetProp.setComment("If true, the model's own force set will be used in the inverse dynamics computation.  "
													"Otherwise, inverse dynamics coordinate actuators will be computed for all unconstrained degrees of freedom.");
	_useModelForceSetProp.setName("use_model_force_set");
	_propertySet.append(&_useModelForceSetProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the inverse dynamics files.
 */
void InverseDynamics::
constructDescription()
{
	string descrip = "This file contains inverse dynamics results.\n\n";
	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the inverse dynamics files.
 */
void InverseDynamics::
constructColumnLabels()
{
	Array<string> labels;
	labels.append("time");
	if(_modelWorkingCopy) {
        if( _useModelForceSet ) {
           for (int i=0; i < _numCoordinateActuators; i++) {
		      Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(i));
			  if( act )labels.append(act->getName());
           }
        } else {
		   const CoordinateSet& cs = _modelWorkingCopy->getCoordinateSet();
		   for (int i=0; i < _numCoordinateActuators; i++) {
		  	  Force& force = _forceSet->get(i);
			  for(int i=0; i<cs.getSize(); i++) {
				 Coordinate& coord = cs.get(i);
				 if(coord.getName()==force.getName()) {
					if(coord.getMotionType() == Coordinate::Rotational) {
						labels.append(force.getName()+"_moment");
					} else if (coord.getMotionType() == Coordinate::Translational) {
						labels.append(force.getName()+"_force");
					} else {
						labels.append(force.getName());
					}
				 }
			  }
		   }
        }
	}
	setColumnLabels(labels);

}

//_____________________________________________________________________________
/**
 * Allocate storage for the inverse dynamics.
 */
void InverseDynamics::
allocateStorage()
{
	_storage = new Storage(1000,"Inverse Dynamics");
	_storage->setDescription(getDescription());
	_storage->setColumnLabels(getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void InverseDynamics::
deleteStorage()
{
	delete _storage; _storage = NULL;
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which the inverse dynamics are to be computed.
 *
 * @param aModel Model pointer
 */
void InverseDynamics::
setModel(Model& aModel)
{
    
	Analysis::setModel(aModel);
	//SimTK::State& s = aModel->getSystem()->updDefaultState();
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the inverse dynamics force storage.
 *
 * @return Inverse dynamics force storage.
 */
Storage* InverseDynamics::
getStorage()
{
	return(_storage);
}

//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of all storage instances.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capacities run out.
 */
void InverseDynamics::
setStorageCapacityIncrements(int aIncrement)
{
	_storage->setCapacityIncrement(aIncrement);
}

//=============================================================================
// ANALYSIS
//=============================================================================
//
void InverseDynamics::
computeAcceleration(const SimTK::State& s, double *aF,double *rAccel) const
{
 
	for(int i=0,j=0; i<_forceSet->getSize(); i++) {
        Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(i));
        if( act ) {
            act->setIsControlled(false);
            act->setForce(s,aF[j++]);
			s.invalidateAll(SimTK::Stage::Velocity);
        }
	}
	_modelWorkingCopy->setAllControllersEnabled(false); // used to be controls=0

	// NEED TO APPLY OTHER FORCES (e.g. Prescribed) FROM ORIGINAL MODEL 

	_modelWorkingCopy->getSystem().realize(s,SimTK::Stage::Acceleration);

	SimTK::Vector udot = _modelWorkingCopy->getMatterSubsystem().getUDot(s);

	for(int i=0; i<_accelerationIndices.getSize(); i++) 
		rAccel[i] = udot[_accelerationIndices[i]];

}

//_____________________________________________________________________________
/**
 * Record the inverse dynamics forces.
 */
int InverseDynamics::
record(const SimTK::State& s)
{
	if(!_modelWorkingCopy) return -1;

//cout << "\nInverse Dynamics record() : \n" << endl;
	// Set model Q's and U's
	SimTK::State sWorkingCopy = _modelWorkingCopy->getSystem().updDefaultState();
	sWorkingCopy.setTime(s.getTime());
	sWorkingCopy.setQ(s.getQ());
	sWorkingCopy.setU(s.getU());

	int nf = _numCoordinateActuators;
	int nacc = _accelerationIndices.getSize();
	int nq = _modelWorkingCopy->getNumCoordinates();

//cout << "\nQ= " << s.getQ() << endl;
//cout << "\nU= " << s.getU() << endl;
	// Build linear constraint matrix and constant constraint vector
	SimTK::Vector f(nf), c(nacc);
	f = 0;
	computeAcceleration(sWorkingCopy, &f[0], &_constraintVector[0]);
//cout << "\n_constraintVector  : \n" << _constraintVector  << endl << endl;
	//char t='t';
	//cout << "NEW Constraint Vector = " << endl;
	//_constraintVector.dump(&t);

//cout << "c  : " <<  endl;
	for(int j=0; j<nf; j++) {
		f[j] = 1;
		computeAcceleration(sWorkingCopy, &f[0], &c[0]);
//cout <<  c  << endl;
		//cout << "NEW Acceleration vector, j=" << j <<" is " << endl;
		//c.dump(&t);
		for(int i=0; i<nacc; i++) _constraintMatrix(i,j) = (c[i] - _constraintVector[i]);
		//cout << "NEW ConstraintMatrix[" << j << "]=" <<  endl;
		//_constraintMatrix.dump(&t);
		f[j] = 0;
	}
	for(int i=0; i<nacc; i++) {
		Coordinate& coord = _modelWorkingCopy->getCoordinateSet().get(_accelerationIndices[i]);
		GCVSpline& presribedFunc = dynamic_cast<GCVSpline&>(_statesSplineSet.get(_statesStore->getStateIndex(coord.getName()+"_u",0)));
		std::vector<int> firstDerivComponents(1);
		firstDerivComponents[0]=0;
        double targetAcceleration = presribedFunc.calcDerivative( firstDerivComponents, SimTK::Vector(1,sWorkingCopy.getTime()));
//cout <<  coord.getName() << " t=" << sWorkingCopy.getTime() << "  acc=" << targetAcceleration << " index=" << _accelerationIndices[i] << endl; 
		_constraintVector[i] = targetAcceleration - _constraintVector[i];
	}
	//cout << "NEW Constraint Vector Adjusted = " << endl;
	//_constraintVector.dump(&t);

	// LAPACK SOLVER
	// NOTE: It destroys the matrices/vectors we pass to it, so we need to pass it copies of performanceMatrix and performanceVector (don't bother making
	// copies of _constraintMatrix/Vector since those are reinitialized each time anyway)
	int info;
	SimTK::Matrix performanceMatrixCopy = _performanceMatrix;
	SimTK::Vector performanceVectorCopy = _performanceVector;
//cout << "performanceMatrixCopy : " << performanceMatrixCopy << endl;
//cout << "performanceVectorCopy : " << performanceVectorCopy << endl;
//cout << "_constraintMatrix : " << _constraintMatrix << endl;
//cout << "_constraintVector : " << _constraintVector << endl;
//cout << "nf=" << nf << "  nacc=" << nacc << endl;
	dgglse_(nf, nf, nacc, &performanceMatrixCopy(0,0), nf, &_constraintMatrix(0,0), nacc, &performanceVectorCopy[0], &_constraintVector[0], &f[0], &_lapackWork[0], _lapackWork.size(), info);

	// Record inverse dynamics forces
	_storage->append(sWorkingCopy.getTime(),nf,&f[0]);

//cout << "\n ** f : " << f << endl << endl;

	return 0;
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int InverseDynamics::
begin(const SimTK::State& s )
{
	if(!proceed()) return(0);

	// Make a working copy of the model
	delete _modelWorkingCopy;
	_modelWorkingCopy = dynamic_cast<Model*>(_model->copy());
	_modelWorkingCopy->updAnalysisSet().setSize(0);
	//_modelWorkingCopy = _model->clone();
	//_modelWorkingCopy = new Model(*_model);

	// Replace model force set with only generalized forces
	if(_model) {
		SimTK::State& sWorkingCopyTemp = _modelWorkingCopy->initSystem();
		// Update the _forceSet we'll be computing inverse dynamics for
		if(_ownsForceSet) delete _forceSet;
		if(_useModelForceSet) {
			// Set pointer to model's internal force set
			_forceSet = &_modelWorkingCopy->updForceSet();
		    _numCoordinateActuators = _modelWorkingCopy->getActuators().getSize();
		} else {
			ForceSet& as = _modelWorkingCopy->updForceSet();
			// Keep a copy of forces that are not muscles to restore them back.
			ForceSet* saveForces = (ForceSet*)as.copy();
			// Generate an force set consisting of a coordinate actuator for every unconstrained degree of freedom
			_forceSet = CoordinateActuator::CreateForceSetOfCoordinateActuatorsForModel(sWorkingCopyTemp,*_modelWorkingCopy,1,false);
		    _numCoordinateActuators = _forceSet->getSize();
			// Copy whatever forces that are not muscles back into the model
			
			for(int i=0; i<saveForces->getSize(); i++){
				const Force& f=saveForces->get(i);
				if ((dynamic_cast<const Muscle*>(&saveForces->get(i)))==NULL)
					as.append((Force*)saveForces->get(i).copy());
			}
		}
	    _modelWorkingCopy->setAllControllersEnabled(false);
		_ownsForceSet = false;

		SimTK::State& sWorkingCopy = _modelWorkingCopy->initSystem();

		// Gather indices into speed set corresponding to the unconstrained degrees of freedom (for which we will set acceleration constraints)
		_accelerationIndices.setSize(0);
		const CoordinateSet& coordSet = _model->getCoordinateSet();
		for(int i=0; i<coordSet.getSize(); i++) {
			const Coordinate& coord = coordSet.get(i);
			if(!coord.getLocked(sWorkingCopy) && !coord.isConstrained()) {
				_accelerationIndices.append(i);
			}
		}

		_dydt.setSize(_modelWorkingCopy->getNumCoordinates() + _modelWorkingCopy->getNumSpeeds());

		int nf = _numCoordinateActuators;
		int nacc = _accelerationIndices.getSize();

		if(nf < nacc) 
			throw(Exception("InverseDynamics: ERROR- overconstrained system -- need at least as many forces as there are degrees of freedom.\n"));

		_constraintMatrix.resize(nacc,nf);
		_constraintVector.resize(nacc);

		_performanceMatrix.resize(nf,nf);
		_performanceMatrix = 0;
		for(int i=0,j=0; i<nf; i++) {
            Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(i));
            if( act ) {
                act->setForce(sWorkingCopy,1);


			    _performanceMatrix(j,j) = act->getStress(sWorkingCopy);
                j++;
             }
		}

		_performanceVector.resize(nf);
		_performanceVector = 0;

		int lwork = nf + nf + nacc;
		_lapackWork.resize(lwork);
	}

	_statesSplineSet=GCVSplineSet(5,_statesStore);

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	deleteStorage();
	allocateStorage();

	// RESET STORAGE
	_storage->reset(s.getTime());

	// RECORD
	int status = 0;
	if(_storage->getSize()<=0) {
		status = record(s);
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s state of sytem
 *
 * @return -1 on error, 0 otherwise.
 */
int InverseDynamics::
step(const SimTK::State& s, int stepNumber )
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int InverseDynamics::
end(const SimTK::State& s )
{
	if(!proceed()) return(0);

	record(s);

    // reset the force multipliers and deltas back to defaults
    for(int i=0; i<_model->getActuators().getSize(); i++) {
        Actuator& act = _model->getActuators().get(i);
        act.setIsControlled(true);
    }

	return(0);
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int InverseDynamics::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// ACCELERATIONS
	_storage->scaleTime(_modelWorkingCopy->getTimeNormConstant());
	Storage::printResult(_storage,aBaseName+"_"+getName(),aDir,aDT,aExtension);

	return(0);
}


