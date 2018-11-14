/* -------------------------------------------------------------------------- *
 *                       OpenSim:  InverseDynamics.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Eran Guendelman, Jeffrey A. Reinbolt                            *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
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
    _numCoordinateActuators(0),
    _useModelForceSet(_useModelForceSetProp.getValueBool()),
    _modelWorkingCopy(NULL)
{
    setNull();

    if(aModel) {
        setModel(*aModel);
    }
    else allocateStorage();
}
// Copy constructor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
InverseDynamics::InverseDynamics(const InverseDynamics &aInverseDynamics):
    Analysis(aInverseDynamics),
    _numCoordinateActuators(aInverseDynamics._numCoordinateActuators),
    _useModelForceSet(_useModelForceSetProp.getValueBool()),
    _modelWorkingCopy(NULL)
{
    setNull();
    // COPY TYPE AND NAME
    *this = aInverseDynamics;
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
    _modelWorkingCopy = 0;
    _numCoordinateActuators = aInverseDynamics._numCoordinateActuators;
    _forceSet = 0;
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
           auto coordinates = _modelWorkingCopy->getCoordinatesInMultibodyTreeOrder();
           for (int i=0; i < _numCoordinateActuators; i++) {
              Force& force = _forceSet->get(i);
              for(size_t i=0u; i<coordinates.size(); ++i) {
                 const Coordinate& coord = *coordinates[i];
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
    // Keep references o all storages in a list for uniform access from GUI
    _storageList.append(_storage);
    _storageList.setMemoryOwner(false);
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
computeAcceleration(SimTK::State& s, double *aF,double *rAccel) const
{
    for(int i=0,j=0; i<_forceSet->getSize(); i++) {
        ScalarActuator* act = dynamic_cast<ScalarActuator*>(&_forceSet->get(i));
        if( act ) {
            act->setOverrideActuation(s, aF[j++]);
        }
    }

    // NEED TO APPLY OTHER FORCES (e.g. Prescribed) FROM ORIGINAL MODEL 

    _modelWorkingCopy->getMultibodySystem().realize(s,SimTK::Stage::Acceleration);

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
    SimTK::State sWorkingCopy = _modelWorkingCopy->getWorkingState();

    // Set modeling options for Actuators to be overridden
    for(int i=0; i<_forceSet->getSize(); i++) {
        ScalarActuator* act = dynamic_cast<ScalarActuator*>(&_forceSet->get(i));
        if( act ) {
            act->overrideActuation(sWorkingCopy, true);
        }
    }

    // Having updated the model, at least re-realize Model stage!
    _modelWorkingCopy->getMultibodySystem().realize(sWorkingCopy, SimTK::Stage::Model);

    sWorkingCopy.setTime(s.getTime());
    sWorkingCopy.setQ(s.getQ());
    sWorkingCopy.setU(s.getU());


    // Having updated the states, at least realize to velocity!
    _modelWorkingCopy->getMultibodySystem().realize(sWorkingCopy, SimTK::Stage::Velocity);

    int nf = _numCoordinateActuators;
    int nacc = _accelerationIndices.getSize();
    // int nq = _modelWorkingCopy->getNumCoordinates();

//cout << "\nQ= " << s.getQ() << endl;
//cout << "\nU= " << s.getU() << endl;
    // Build linear constraint matrix and constant constraint vector
    SimTK::Vector f(nf), c(nacc);
    f = 0;
    computeAcceleration(sWorkingCopy, &f[0], &_constraintVector[0]);

    for(int j=0; j<nf; j++) {
        f[j] = 1;
        computeAcceleration(sWorkingCopy, &f[0], &c[0]);
        for(int i=0; i<nacc; i++) _constraintMatrix(i,j) = (c[i] - _constraintVector[i]);
        f[j] = 0;
    }

    auto coordinates = _modelWorkingCopy->getCoordinatesInMultibodyTreeOrder();

    for(int i=0; i<nacc; i++) {
        const Coordinate& coord = *coordinates[_accelerationIndices[i]];
        int ind = _statesStore->getStateIndex(coord.getSpeedName(), 0);
        if (ind < 0){
            // get the full coordinate speed state variable path name
            string fullname = coord.getStateVariableNames()[1];
            ind = _statesStore->getStateIndex(fullname, 0);
            if (ind < 0){
                string msg = "InverseDynamics::record(): \n";
                msg += "target motion for coordinate '";
                msg += coord.getName() + "' not found.";
                throw Exception(msg);
            }
        }
        Function& targetFunc = _statesSplineSet.get(ind);
        std::vector<int> firstDerivComponents(1);
        firstDerivComponents[0]=0;
        double targetAcceleration = targetFunc.calcDerivative(firstDerivComponents, SimTK::Vector(1, sWorkingCopy.getTime()));
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
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int InverseDynamics::begin(const SimTK::State& s )
{
    if(!proceed()) return(0);

    SimTK::Matrix massMatrix;
    _model->getMatterSubsystem().calcM(s, massMatrix);

    //massMatrix.dump("mass matrix is:");
    // Check that m is full rank
    try {
        SimTK::FactorLU lu(massMatrix);
    } catch (const SimTK::Exception::Base&) {
        throw Exception("InverseDynamics: ERROR- mass matrix is singular  ",__FILE__,__LINE__);
    }
    // enable the line below when simmath is fixed
    //bool singularMassMatrix = lu.isSingular();

    //cout << "Mass matrix is " << (singularMassMatrix?"singular":"Non-singular");

    // Make a working copy of the model
    delete _modelWorkingCopy;
    _modelWorkingCopy = _model->clone();
    _modelWorkingCopy->updAnalysisSet().setSize(0);

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
            ForceSet* saveForces = as.clone();
            // Generate an force set consisting of a coordinate actuator for every unconstrained degree of freedom
            _forceSet = CoordinateActuator::CreateForceSetOfCoordinateActuatorsForModel(sWorkingCopyTemp,*_modelWorkingCopy,1,false);
            _numCoordinateActuators = _forceSet->getSize();
            // Copy whatever forces that are not muscles back into the model
            
            for(int i=0; i<saveForces->getSize(); i++){
                // const Force& f=saveForces->get(i);
                if ((dynamic_cast<const Muscle*>(&saveForces->get(i)))==NULL)
                    as.append(saveForces->get(i).clone());
            }
        }
        _modelWorkingCopy->setAllControllersEnabled(false);
        _ownsForceSet = false;

        SimTK::State& sWorkingCopy = _modelWorkingCopy->initSystem();

        // Gather indices into speed set corresponding to the unconstrained degrees of freedom (for which we will set acceleration constraints)
        _accelerationIndices.setSize(0);
        auto coordinates = _modelWorkingCopy->getCoordinatesInMultibodyTreeOrder();
        for(size_t i=0u; i<coordinates.size(); ++i) {
            const Coordinate& coord = *coordinates[i];
            if(!coord.isConstrained(sWorkingCopy)) {
                _accelerationIndices.append(static_cast<int>(i));
            }
        }

        _dydt.setSize(_modelWorkingCopy->getNumCoordinates() + _modelWorkingCopy->getNumSpeeds());

        int nf = _numCoordinateActuators;
        int nacc = _accelerationIndices.getSize();

        if(nf < nacc) 
            throw(Exception("InverseDynamics: ERROR- over-constrained system -- need at least as many forces as there are degrees of freedom.\n"));

        // Realize to velocity in case there are any velocity dependent forces
        _modelWorkingCopy->getMultibodySystem().realize(sWorkingCopy, SimTK::Stage::Velocity);

        _constraintMatrix.resize(nacc,nf);
        _constraintVector.resize(nacc);

        _performanceMatrix.resize(nf,nf);
        _performanceMatrix = 0;
        for(int i=0,j=0; i<nf; i++) {
            ScalarActuator* act = dynamic_cast<ScalarActuator*>(&_forceSet->get(i));
            if( act ) {
                act->setActuation(sWorkingCopy, 1);
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
 * This method should be overridden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s state of system
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
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int InverseDynamics::
end(const SimTK::State&s )
{
    if(!proceed()) return(0);

    record(s);

    _model->overrideAllActuators(*const_cast<SimTK::State*>(&s), false );

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
    Storage::printResult(_storage,aBaseName+"_"+getName(),aDir,aDT,aExtension);

    return(0);
}


