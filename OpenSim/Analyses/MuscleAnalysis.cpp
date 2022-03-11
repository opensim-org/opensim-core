/* -------------------------------------------------------------------------- *
 *                        OpenSim:  MuscleAnalysis.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Katherine R. S. Holzbaur, Frank C. Anderson, Ajay Seth,         *
 *            Matthew Millard                                                 *
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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "MuscleAnalysis.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
MuscleAnalysis::~MuscleAnalysis()
{
    // Individual storages where added to the Analysis' _storageList
    // which takes ownership of the Storage objects and deletes them.
}
//_____________________________________________________________________________
/**
 * Construct a MuscleAnalysis object for recording the MuscleAnalysis of
 * a model's generalized coordinates during a simulation.
 *
 * @param aModel Model for which the MuscleAnalysis are to be recorded.
 */
MuscleAnalysis::MuscleAnalysis(Model *aModel) :
    Analysis(aModel)
{
    // NULL
    setNull();

    // CHECK MODEL
    if(_model==NULL) return;

    // STORAGE
    allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
MuscleAnalysis::MuscleAnalysis(const std::string &aFileName):
Analysis(aFileName, false)
{
    setNull();
    updateFromXMLDocument();
    allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
MuscleAnalysis::MuscleAnalysis(const MuscleAnalysis &aMuscleAnalysis):
Analysis(aMuscleAnalysis)
{
    setNull();
    *this = aMuscleAnalysis;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void MuscleAnalysis::setNull()
{
    setAuthors("Ajay Seth, Matthew Millard, Katherine Holzbaur, Frank Anderson");
    setName("MuscleAnalysis");
    setupProperties();
    constructDescription();

    // STORAGE
    _pennationAngleStore = NULL;
    _lengthStore = NULL;
    _fiberLengthStore = NULL;
    _normalizedFiberLengthStore = NULL;
    _tendonLengthStore = NULL;

    _fiberVelocityStore = NULL;
    _normFiberVelocityStore = NULL;
    _pennationAngularVelocityStore = NULL;

    _forceStore = NULL;
    _fiberForceStore = NULL;
    _activeFiberForceStore = NULL;
    _passiveFiberForceStore = NULL;
    _activeFiberForceAlongTendonStore = NULL;
    _passiveFiberForceAlongTendonStore = NULL;

    _fiberActivePowerStore  =   NULL;
    _fiberPassivePowerStore =   NULL;
    _tendonPowerStore       =   NULL;
    _musclePowerStore       =   NULL;

    // DEFAULT VALUES
    _muscleListProp.getValueStrArray().setSize(1);
    _muscleListProp.getValueStrArray().updElt(0) = "all";
    _coordinateListProp.getValueStrArray().setSize(1);
    _coordinateListProp.getValueStrArray().updElt(0) = "all";
    setComputeMoments(true);
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void MuscleAnalysis::setupProperties()
{
    _muscleListProp.setComment( "List of muscles for which to "
                                "perform the analysis. Use 'all' to perform"
                                " the analysis for all muscles.");
    _muscleListProp.setName("muscle_list");
    _propertySet.append( &_muscleListProp );

    _coordinateListProp.setComment("List of generalized coordinates for which"
        " to compute moment arms. Use 'all' to compute for all coordinates.");
    _coordinateListProp.setName("moment_arm_coordinate_list");
    _propertySet.append( &_coordinateListProp );

    _computeMomentsProp.setComment("Flag indicating whether moment-arms and/or "
                                   "moments should be computed.");
    _computeMomentsProp.setName("compute_moments");
    _propertySet.append( &_computeMomentsProp );

}
//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the MuscleAnalysis files.
 */
void MuscleAnalysis::constructDescription()
{
    char descrip[1024];

    strcpy(descrip, "\nThis analysis gathers basic information about muscles ");
    strcat(descrip, "during a simulation (e.g., forces, tendon lengths,"
        " moment arms, etc).");

    strcat(descrip, "\nUnits are S.I. units (second, meters, Newtons, ...)");
    strcat(descrip, "\nIf the header above contains a line with ");
    strcat(descrip, "'inDegrees', this indicates whether rotational values ");
    strcat(descrip, "are in degrees (yes) or radians (no).");
    strcat(descrip, "\n\n");
    setDescription(descrip);
}
//_____________________________________________________________________________
/**
 * Allocate storage for the muscle variables.
 */
void MuscleAnalysis::allocateStorageObjects()
{
    if(_model==NULL) return;

    // CLEAR EXISTING WORK ARRAYS
    _storageList.setMemoryOwner(true);
    _storageList.setSize(0);
    _momentArmStorageArray.setMemoryOwner(true);
    _momentArmStorageArray.setSize(0);
    _muscleArray.setMemoryOwner(false);
    _muscleArray.setSize(0);

    // FOR MOMENT ARMS AND MOMENTS
    if(getComputeMoments()) {
        const CoordinateSet& qSet = _model->getCoordinateSet();
        _coordinateList = _coordinateListProp.getValueStrArray();

        if(IO::Lowercase(_coordinateList[0]) == "all"){
            _coordinateList.setSize(0);
            for(int i=0; i < qSet.getSize(); ++i) {
                _coordinateList.append(qSet[i].getName());
            }
        } 
        else{
            int i=0;
            while(i <_coordinateList.getSize()){
                int found = qSet.getIndex(_coordinateList[i]);
                if(found < 0){
                    log_warn("MuscleAnalysis: coordinate {} is not part of the "
                             "model.",
                            _coordinateList[i]);
                    _coordinateList.remove(i);
                }
                else{
                    ++i;
                }
            }
        }

        int nq = _coordinateList.getSize();
        Storage* store;
        for(int i=0;i<nq;i++) {
            string name = "MomentArm_" + _coordinateList[i];
            store = new Storage(1000,name);
            store->setDescription(getDescription());
            _storageList.append(store);
        }
        for(int i=0;i<nq;i++) {
            string name = "Moment_" + _coordinateList[i];
            store = new Storage(1000,name);
            store->setDescription(getDescription());
            _storageList.append(store);
        }

        // POPULATE ACTIVE MOMENT ARM ARRAY
        _momentArmStorageArray.setSize(0);

        for(int i=0; i<nq; i++) {
            int found = qSet.getIndex(_coordinateList[i]);
            if(found >=0){
                StorageCoordinatePair *pair = new StorageCoordinatePair();
                pair->q = &qSet[found];
                pair->momentArmStore = _storageList[i];
                pair->momentStore = _storageList[i+nq];
                _momentArmStorageArray.append(pair);
            }
        }
    }

    // EVERYTHING ELSE
    //_storageList.setMemoryOwner(false);
    _pennationAngleStore = new Storage(1000,"PennationAngle");
    _pennationAngleStore->setDescription(getDescription());
    _storageList.append(_pennationAngleStore );

    _lengthStore = new Storage(1000,"Length");
    _lengthStore->setDescription(getDescription());
    _storageList.append(_lengthStore );

    _fiberLengthStore = new Storage(1000,"FiberLength");
    _fiberLengthStore->setDescription(getDescription());
    _storageList.append(_fiberLengthStore );

    _normalizedFiberLengthStore = new Storage(1000,"NormalizedFiberLength");
    _normalizedFiberLengthStore->setDescription(getDescription());
    _storageList.append(_normalizedFiberLengthStore );

    _tendonLengthStore = new Storage(1000,"TendonLength");
    _tendonLengthStore->setDescription(getDescription());
    _storageList.append(_tendonLengthStore );

    _fiberVelocityStore = new Storage(1000,"FiberVelocity");
    _fiberVelocityStore->setDescription(getDescription());
    _storageList.append(_fiberVelocityStore );

    _normFiberVelocityStore = new Storage(1000,"NormFiberVelocity");
    _normFiberVelocityStore->setDescription(getDescription());
    _storageList.append(_normFiberVelocityStore );

    _pennationAngularVelocityStore = new Storage(1000,
        "PennationAngularVelocity");
    _pennationAngularVelocityStore->setDescription(getDescription());
    _storageList.append(_pennationAngularVelocityStore );

    _forceStore = new Storage(1000,"TendonForce");
    _forceStore->setDescription(getDescription());
    _storageList.append(_forceStore );

    _fiberForceStore = new Storage(1000,"FiberForce");
    _fiberForceStore->setDescription(getDescription());
    _storageList.append(_fiberForceStore );

    _activeFiberForceStore = new Storage(1000,"ActiveFiberForce");
    _activeFiberForceStore->setDescription(getDescription());
    _storageList.append(_activeFiberForceStore );

    _passiveFiberForceStore = new Storage(1000,"PassiveFiberForce");
    _passiveFiberForceStore->setDescription(getDescription());
    _storageList.append(_passiveFiberForceStore );

    _activeFiberForceAlongTendonStore = new Storage(1000,
        "ActiveFiberForceAlongTendon");
    _activeFiberForceAlongTendonStore->setDescription(getDescription());
    _storageList.append(_activeFiberForceAlongTendonStore );

    _passiveFiberForceAlongTendonStore = new Storage(1000,
        "PassiveFiberForceAlongTendon");
    _passiveFiberForceAlongTendonStore->setDescription(getDescription());
    _storageList.append(_passiveFiberForceAlongTendonStore );

    _fiberActivePowerStore = new Storage(1000,"FiberActivePower");
    _fiberActivePowerStore->setDescription(getDescription());
    _storageList.append(_fiberActivePowerStore );

    _fiberPassivePowerStore = new Storage(1000,"FiberPassivePower");
    _fiberPassivePowerStore->setDescription(getDescription());
    _storageList.append(_fiberPassivePowerStore );

    _tendonPowerStore = new Storage(1000,"TendonPower");
    _tendonPowerStore->setDescription(getDescription());
    _storageList.append(_tendonPowerStore );

    _musclePowerStore = new Storage(1000,"MuscleActuatorPower");
    _musclePowerStore->setDescription(getDescription());
    _storageList.append(_musclePowerStore );

    // POPULATE MUSCLE LIST FOR "all"
    ForceSet& fSet = _model->updForceSet();
    _muscleList = _muscleListProp.getValueStrArray();
    int nm = _muscleList.getSize();
    if((nm==1) && (_muscleList.get(0)=="all")) {
        _muscleList.setSize(0);
        int nf = fSet.getSize();
        for(int i=0;i<nf;i++) {
            Muscle *m = dynamic_cast<Muscle*>(&fSet.get(i));
            if( m ) _muscleList.append(m->getName());
        }
    }
    // POPULATE ACTIVE MUSCLE ARRAY
    Array<string> tmpMuscleList("");
    nm = _muscleList.getSize();
    _muscleArray.setSize(0);
    for(int i=0; i<nm; i++) {
        if(fSet.contains(_muscleList[i])) {
            Muscle* mus = dynamic_cast<Muscle*>( &fSet.get(_muscleList[i]) );
            if(mus){
                _muscleArray.append(mus);
                tmpMuscleList.append(mus->getName());
            }
        }
    }
    _muscleList = tmpMuscleList;

    // CONSTRUCT AND SET COLUMN LABELS
    constructColumnLabels();

    int size = _storageList.getSize();
    Storage* store;
    for(int i=0;i<size;i++) {
        store = _storageList[i];
        if(store==NULL) continue;
        store->setColumnLabels(getColumnLabels());
    }
}

//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the column labels for the MuscleAnalysis storage files.
 */
void MuscleAnalysis::constructColumnLabels()
{
    if(!_model) return;
    int size = _muscleList.getSize();
    Array<string> labels("",size+1);
    labels[0] = "time";
    for(int i=0; i<size; i++) {
        labels[i+1] = _muscleList[i];
    }
    setColumnLabels(labels);
}


//=============================================================================
// OPERATORS
//=============================================================================
MuscleAnalysis& MuscleAnalysis::operator=(const MuscleAnalysis &aAnalysis)
{
    // BASE CLASS
    Analysis::operator=(aAnalysis);

    // MEMBER VARIABLES
    _muscleListProp = aAnalysis._muscleListProp;
    _coordinateListProp = aAnalysis._coordinateListProp;
    _computeMomentsProp = aAnalysis._computeMomentsProp;
    allocateStorageObjects();

    return (*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void MuscleAnalysis::setModel(Model& aModel)
{
    Super::setModel(aModel);
    allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Set the list of muscles to analyze.
 *
 * @param aMuscles is the array of names of muscles to analyze.
 */
void MuscleAnalysis::setMuscles(OpenSim::Array<std::string>& aMuscles)
{
    int size = aMuscles.getSize();
    _muscleListProp.getValueStrArray().setSize(aMuscles.getSize());
    for(int i=0; i<size; i++){
        _muscleListProp.getValueStrArray().updElt(i) = aMuscles.get(i);
    }
}
//_____________________________________________________________________________
/**
 * Set the list of coordinates.
 *
 * @param aCoordinates Array of coordinates about which to compute moment arms.
 */
void MuscleAnalysis::
setCoordinates(OpenSim::Array<std::string>& aCoordinates)
{
    int size = aCoordinates.getSize();
    _coordinateListProp.getValueStrArray().setSize(size);
    for(int i=0; i<size; i++){
        _coordinateListProp.getValueStrArray().updElt(i) = aCoordinates[i];
    }
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
void MuscleAnalysis::
setStorageCapacityIncrements(int aIncrement)
{
    if(!_model) return;
    Storage *store;
    int size = _storageList.getSize();
    for(int i=0;i<size;i++) {
        store = _storageList[i];
        if(store==NULL) continue;
        store->setCapacityIncrement(aIncrement);
    }
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the MuscleAnalysis quantities.
 */
int MuscleAnalysis::record(const SimTK::State& s)
{
    if(_model==NULL) return -1;
    if (!getOn()) return -1;

    // MAKE SURE ALL ACTUATION QUANTITIES ARE VALID
    // COMPUTE DERIVATIVES
    // ----------------------------------
    // TIME NORMALIZATION
    double tReal = s.getTime();
    // ----------------------------------
    // LOOP THROUGH MUSCLES
    int nm = _muscleArray.getSize();

    double nan = SimTK::NaN;
    // Angles and lengths
    Array<double> penang(nan,nm);
    Array<double> len(nan,nm), tlen(nan,nm);
    Array<double> fiblen(nan,nm), normfiblen(nan,nm);

    // Muscle velocity information
    Array<double> fibVel(nan,nm), normFibVel(nan,nm);
    Array<double> penAngVel(nan,nm);

    // Muscle component forces
    Array<double> force(nan, nm), fibforce(nan, nm);
    Array<double> actfibforce(nan,nm), passfibforce(nan,nm);
    Array<double> actfibforcealongten(nan,nm), passfibforcealongten(nan,nm);

    // Muscle and component powers
    Array<double> fibActivePower(nan,nm), fibPassivePower(nan,nm),
                  tendonPower(nan,nm), muscPower(nan,nm);

    double sysMass = _model->getMatterSubsystem().calcSystemMass(s);
    bool hasMass = sysMass > SimTK::Eps;

    // Just warn once per instant
    bool lengthWarning = false;
    bool forceWarning = false;
    bool dynamicsWarning = false;

    for(int i=0; i<nm; ++i) {
        try{
            len[i] = _muscleArray[i]->getLength(s);
            tlen[i] = _muscleArray[i]->getTendonLength(s);
            fiblen[i] = _muscleArray[i]->getFiberLength(s);
            normfiblen[i] = _muscleArray[i]->getNormalizedFiberLength(s);
            penang[i] = _muscleArray[i]->getPennationAngle(s);
        }
        catch (const std::exception& e) {
            if(!lengthWarning){
                log_warn("MuscleAnalysis::record() unable to evaluate muscle "
                         "length at time {} for reason: {}", s.getTime(),
                         e.what());
                lengthWarning = true;
            }
            continue;
        }

        try{
            // Compute muscle forces that are dependent on Positions, Velocities
            // so that later quantities are valid and setForce is called
            _muscleArray[i]->computeActuation(s);
            force[i] = _muscleArray[i]->getActuation(s);
            fibforce[i] = _muscleArray[i]->getFiberForce(s);
            actfibforce[i] = _muscleArray[i]->getActiveFiberForce(s);
            passfibforce[i] = _muscleArray[i]->getPassiveFiberForce(s);
            actfibforcealongten[i] = _muscleArray[i]->getActiveFiberForceAlongTendon(s);
            passfibforcealongten[i] = _muscleArray[i]->getPassiveFiberForceAlongTendon(s);
        }
        catch (const std::exception& e) {
            if(!forceWarning){
                log_warn("MuscleAnalysis::record() unable to evaluate muscle "
                         "forces at time {} for reason: {}",
                        s.getTime(), e.what());
                forceWarning = true;
            }
            continue;
        }
    }

    // Cannot compute system dynamics without mass
    if(hasMass){
        // state derivatives (activation rate and fiber velocity) evaluated at dynamics
        _model->getMultibodySystem().realize(s,SimTK::Stage::Dynamics);

        for(int i=0; i<nm; ++i) {
            try{
                //Velocities
                fibVel[i] = _muscleArray[i]->getFiberVelocity(s);
                normFibVel[i] =  _muscleArray[i]->getNormalizedFiberVelocity(s);
                penAngVel[i] =  _muscleArray[i]->getPennationAngularVelocity(s);
                //Powers
                fibActivePower[i] = _muscleArray[i]->getFiberActivePower(s);
                fibPassivePower[i] = _muscleArray[i]->getFiberPassivePower(s);
                tendonPower[i] = _muscleArray[i]->getTendonPower(s);
                muscPower[i] = _muscleArray[i]->getMusclePower(s);
            }
            catch (const std::exception& e) {
                if(!dynamicsWarning){
                    log_warn("MuscleAnalysis::record() unable to evaluate "
                             "muscle forces at time {} for reason: {}",
                             s.getTime(), e.what());
                    dynamicsWarning = true;
                }
            continue;
            }
        }
    }
    else {
        if(!dynamicsWarning){
            log_warn("MuscleAnalysis::record() unable to evaluate muscle "
                     "dynamics at time {} because model has no mass and system "
                     "dynamics cannot be computed.");
            dynamicsWarning = true;
        }
    }

    // APPEND TO STORAGE
    _pennationAngleStore->append(tReal,penang.getSize(),&penang[0]);
    _lengthStore->append(tReal,len.getSize(),&len[0]);
    _fiberLengthStore->append(tReal,fiblen.getSize(),&fiblen[0]);
    _normalizedFiberLengthStore
        ->append(tReal,normfiblen.getSize(),&normfiblen[0]);
    _tendonLengthStore->append(tReal,tlen.getSize(),&tlen[0]);

    _fiberVelocityStore->append(tReal,fibVel.getSize(),&fibVel[0]);
    _normFiberVelocityStore->append(tReal,normFibVel.getSize(),&normFibVel[0]);
    _pennationAngularVelocityStore
        ->append(tReal,penAngVel.getSize(),&penAngVel[0]);

    _forceStore->append(tReal,force.getSize(),&force[0]);
    _fiberForceStore->append(tReal,fibforce.getSize(),&fibforce[0]);
    _activeFiberForceStore->append(tReal,actfibforce.getSize(),&actfibforce[0]);
    _passiveFiberForceStore
        ->append(tReal,passfibforce.getSize(),&passfibforce[0]);
    _activeFiberForceAlongTendonStore
        ->append(tReal,actfibforcealongten.getSize(),&actfibforcealongten[0]);
    _passiveFiberForceAlongTendonStore
        ->append(tReal,passfibforcealongten.getSize(),&passfibforcealongten[0]);

    _fiberActivePowerStore
        ->append(tReal,fibActivePower.getSize(),&fibActivePower[0]);
    _fiberPassivePowerStore
        ->append(tReal,fibPassivePower.getSize(),&fibPassivePower[0]);
    _tendonPowerStore->append(tReal,tendonPower.getSize(),&tendonPower[0]);
    _musclePowerStore->append(tReal,muscPower.getSize(),&muscPower[0]);

    if (getComputeMoments()){
        // LOOP OVER ACTIVE MOMENT ARM STORAGE OBJECTS
        Coordinate *q = NULL;
        Storage *maStore=NULL, *mStore=NULL;
        int nq = _momentArmStorageArray.getSize();
        Array<double> ma(0.0,nm),m(0.0,nm);

        for(int i=0; i<nq; i++) {

            q = _momentArmStorageArray[i]->q;
            maStore = _momentArmStorageArray[i]->momentArmStore;
            mStore = _momentArmStorageArray[i]->momentStore;
           
            // bool locked = q->getLocked(s);

            _model->getMultibodySystem().realize(s, s.getSystemStage());
            // LOOP OVER MUSCLES
            for(int j=0; j<nm; j++) {
                ma[j] = _muscleArray[j]->computeMomentArm(s,*q);
                m[j] = ma[j] * force[j];
            }
            maStore->append(s.getTime(),nm,&ma[0]);
            mStore->append(s.getTime(),nm,&m[0]);
        }
    }
    return 0;
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the beginning of an integration 
 *
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current system state
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::begin(const SimTK::State&s )
{
    if(!proceed()) return 0;

    allocateStorageObjects();

    // RESET STORAGE
    Storage *store;
    int size = _storageList.getSize();
    for(int i=0;i<size;i++) {
        store = _storageList[i];
        if(store==NULL) continue;
        store->purge();
    }

    // RECORD
    int status = 0;
    // Make sure coordinates are not locked
    if (getComputeMoments()){
    // LOOP OVER ACTIVE MOMENT ARM STORAGE OBJECTS
        Coordinate *q = NULL;
        int nq = _momentArmStorageArray.getSize();
        for(int i=0; i<nq; i++) {
            q = _momentArmStorageArray[i]->q;
            if (q->getLocked(s)) {
                log_warn("MuscleAnalysis: coordinate {} is locked and can't be "
                         "varied.",
                        q->getName());
            }
        }
    }
    if(_storageList.getSize()> 0 && _storageList.get(0)->getSize() <= 0) status = record(s);

    return status;
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called 
 *
 * This method should be overridden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s current state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::step(const SimTK::State& s, int stepNumber )
{
    if(!proceed(stepNumber)) return 0;

    /*int status = */record(s);

    return 0;
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration
 *
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::end(const SimTK::State& s )
{
    if (!proceed()) return 0;
    record(s);
    return 0;
}

//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration 
 *
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    if(!getOn()) {
        log_info("MuscleAnalysis.printResults: Off- not printing.");
        return 0;
    }

    std::string prefix = aBaseName + "_" + getName() + "_";
    for(int i=0; i<_storageList.getSize(); ++i){
        Storage::printResult(_storageList[i],prefix+_storageList[i]->getName(),aDir,aDT,aExtension);
    }

    int size = _momentArmStorageArray.getSize();
    for(int i=0;i<size;i++) {
        string fileName = prefix + _momentArmStorageArray.get(i)
            ->momentArmStore->getName();
        Storage::printResult(_momentArmStorageArray.get(i)
            ->momentArmStore,fileName,aDir,aDT,aExtension);
        fileName = prefix + _momentArmStorageArray.get(i)
            ->momentStore->getName();
        Storage::printResult(_momentArmStorageArray.get(i)
            ->momentStore,fileName,aDir,aDT,aExtension);
    }

    return 0;
}

