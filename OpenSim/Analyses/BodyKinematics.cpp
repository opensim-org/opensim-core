/* -------------------------------------------------------------------------- *
 *                        OpenSim:  BodyKinematics.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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
#include "BodyKinematics.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
#define MAXLEN 10000
#define CENTER_OF_MASS_NAME string("center_of_mass")


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyKinematics::~BodyKinematics()
{
    deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an BodyKinematics instance for recording the kinematics of
 * the bodies of a model during a simulation.
 *
 * @param aModel Model for which the analyses are to be recorded.
 */
BodyKinematics::BodyKinematics(Model *aModel, bool aInDegrees) :
    Analysis(aModel),
    _bodies(_bodiesProp.getValueStrArray()),
    _expressInLocalFrame(_expressInLocalFrameProp.getValueBool())
{
     setNull();

    // STORAGE
    allocateStorage();

    if (_model ==0)
        return;
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
BodyKinematics::BodyKinematics(const std::string &aFileName):
    Analysis(aFileName, false),
    _bodies(_bodiesProp.getValueStrArray()),
    _expressInLocalFrame(_expressInLocalFrameProp.getValueBool())
{
    setNull();

    // Serialize from XML
    updateFromXMLDocument();

}

// Copy constructor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
BodyKinematics::BodyKinematics(const BodyKinematics &aBodyKinematics):
    Analysis(aBodyKinematics),
    _bodies(_bodiesProp.getValueStrArray()),
    _expressInLocalFrame(_expressInLocalFrameProp.getValueBool())
{
    setNull();
    // COPY TYPE AND NAME
    *this = aBodyKinematics;
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
BodyKinematics& BodyKinematics::
operator=(const BodyKinematics &aBodyKinematics)
{
    // BASE CLASS
    Analysis::operator=(aBodyKinematics);
    _bodies = aBodyKinematics._bodies;
    _expressInLocalFrame = aBodyKinematics._expressInLocalFrame;
    return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void BodyKinematics::
setNull()
{
    setupProperties();

    // POINTERS
    _pStore = NULL;
    _vStore = NULL;
    _aStore = NULL;
    _bodies.setSize(1);
    _bodies[0] = "all";
    _recordCenterOfMass = true;

    // OTHER VARIABLES

    //?_body
    setName("BodyKinematics");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BodyKinematics::
setupProperties()
{
    _bodiesProp.setName("bodies");
    _bodiesProp.setComment("Names of bodies to record kinematics for.  Use 'all' to record all bodies."
                                  "  The special name '"+CENTER_OF_MASS_NAME+"' refers to the combined center of mass.");
    _propertySet.append( &_bodiesProp );

    _expressInLocalFrameProp.setName("express_results_in_body_local_frame");
    _expressInLocalFrameProp.setComment("Flag (true or false) indicating whether"
        " to express results in the global frame or local-frames of the bodies. "
        "Body positions and center of mass results are always given in the global frame. "
        "This flag is set to false by default.");
    _expressInLocalFrameProp.setValue(false);
    _propertySet.append(&_expressInLocalFrameProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void BodyKinematics::
constructDescription()
{
    char descrip[1024];
    char tmp[MAXLEN];

    strcpy(descrip, "\nThis file contains the kinematics ");
    strcat(descrip, "(positions and orientations,\n");
    strcat(descrip, "velocities and angular velocities, or");
    strcat(descrip, " accelerations and angular accelerations)\n");
    strcat(descrip, "of the centers of mass");
    sprintf(tmp, " of the body segments in model %s.\n",
        _model->getName().c_str());
    strcat(descrip, tmp);
    strcat(descrip, "\nBody segment orientations are described using");
    strcat(descrip, " body-fixed X-Y-Z Euler angles.\n");
    if (_expressInLocalFrame) {
        strcat(descrip, "\nAngular velocities and accelerations are");
        strcat(descrip, " expressed in the body-local frame.\n");
    } 
    else {
        strcat(descrip, "\nAngular velocities and accelerations are");
        strcat(descrip, " expressed in the ground frame.\n");
    }
    strcat(descrip, "\nUnits are S.I. units (seconds, meters, Newtons, ...)");
    strcat(descrip, "\nIf the header above contains a line with ");
    strcat(descrip, "'inDegrees', this indicates whether rotational values ");
    strcat(descrip, "are in degrees (yes) or radians (no).");
    strcat(descrip, "\n\n");

    setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body kinematics files.
 */
void BodyKinematics::
constructColumnLabels()
{
    Array<string> labels;
    labels.append("time");

    BodySet& bs = _model->updBodySet();
    for(int i=0; i<_bodyIndices.getSize(); i++) {
        Body& body = bs.get(_bodyIndices[i]);
        labels.append(body.getName() + "_X");
        labels.append(body.getName() + "_Y");
        labels.append(body.getName() + "_Z");
        labels.append(body.getName() + "_Ox");
        labels.append(body.getName() + "_Oy");
        labels.append(body.getName() + "_Oz");
    }

    if(_recordCenterOfMass) {
        // ADD NAMES FOR POSITION, VELOCITY, AND ACCELERATION OF WHOLE BODY
        labels.append(CENTER_OF_MASS_NAME + "_X");
        labels.append(CENTER_OF_MASS_NAME + "_Y");
        labels.append(CENTER_OF_MASS_NAME + "_Z");
    }

    setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void BodyKinematics::
allocateStorage()
{
    // ACCELERATIONS
    _aStore = new Storage(1000,"Accelerations");
    _aStore->setDescription(getDescription());
    _aStore->setColumnLabels(getColumnLabels());

    // VELOCITIES
    _vStore = new Storage(1000,"Velocities");
    _vStore->setDescription(getDescription());
    _vStore->setColumnLabels(getColumnLabels());

    // POSITIONS
    _pStore = new Storage(1000,"Positions");
    _pStore->setDescription(getDescription());
    _pStore->setColumnLabels(getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void BodyKinematics::
deleteStorage()
{
    if(_aStore!=NULL) { delete _aStore;  _aStore=NULL; }
    if(_vStore!=NULL) { delete _vStore;  _vStore=NULL; }
    if(_pStore!=NULL) { delete _pStore;  _pStore=NULL; }
}

//_____________________________________________________________________________
/**
 * Update bodies to record
 */
void BodyKinematics::
updateBodiesToRecord()
{
    if(!_model) {
        _bodyIndices.setSize(0);
        _kin.setSize(0);
        return;
    }

    BodySet& bs = _model->updBodySet();
    _recordCenterOfMass = false;
    _bodyIndices.setSize(0);
    for(int i=0; i<_bodies.getSize(); i++) {
        if(_bodies[i] == "all") {
            _bodyIndices.setSize(bs.getSize());
            for(int j=0;j<bs.getSize();j++) _bodyIndices[j]=j;
            _recordCenterOfMass = true;
            break;
        }
        if(_bodies[i] == CENTER_OF_MASS_NAME) {
            _recordCenterOfMass = true;
            continue;
        }
        int index = bs.getIndex(_bodies[i]);
        if(index<0) 
            throw Exception("BodyKinematics: ERR- Could not find body named '"+_bodies[i]+"'",__FILE__,__LINE__);
        _bodyIndices.append(index);
    }
    _kin.setSize(6*_bodyIndices.getSize()+(_recordCenterOfMass?3:0));

    if(_kin.getSize()==0) {
        log_warn("BodyKinematics analysis has no bodies to record kinematics "
                 "for");
    }
}



//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which the body kinematics are to be computed.
 *
 * @param aModel Model pointer
 */
void BodyKinematics::
setModel(Model& aModel)
{
    Analysis::setModel(aModel);


    // DESCRIPTION AND LABELS
    constructDescription();
    updateBodiesToRecord();
    constructColumnLabels();

    deleteStorage();
    allocateStorage();
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the acceleration storage.
 *
 * @return Acceleration storage.
 */
Storage* BodyKinematics::
getAccelerationStorage()
{
    return(_aStore);
}
//_____________________________________________________________________________
/**
 * Get the velocity storage.
 *
 * @return Velocity storage.
 */
Storage* BodyKinematics::
getVelocityStorage()
{
    return(_vStore);
}
//_____________________________________________________________________________
/**
 * Get the position storage.
 *
 * @return Position storage.
 */
Storage* BodyKinematics::
getPositionStorage()
{
    return(_pStore);
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
void BodyKinematics::
setStorageCapacityIncrements(int aIncrement)
{
    _aStore->setCapacityIncrement(aIncrement);
    _vStore->setCapacityIncrement(aIncrement);
    _pStore->setCapacityIncrement(aIncrement);
}

//-----------------------------------------------------------------------------
// ANGULAR VELOCITY IN LOCAL FRAME FLAG
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a flag indicating that the angular velocities should be output in the 
 * the body local reference frame.
 *
 * @param aTrueFalse  False will result in the angular velocities being output
 *      in the global reference frame
 */
void BodyKinematics::
setExpressResultsInLocalFrame(bool aTrueFalse)
{
    _expressInLocalFrame = aTrueFalse;
}

/**
 * Get a flag indicating whether the angular velocities should be output in the 
 * the body local reference frame.
 *
 * @param rTrueFalse  False indicates the angular velocities being output
 *      in the global reference frame
 */
bool BodyKinematics::
getExpressResultsInLocalFrame()
{
    return(_expressInLocalFrame);
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the kinematics.
 */
int BodyKinematics::
record(const SimTK::State& s)
{

    // Realize to Acceleration first since we'll ask for Accelerations 
    _model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    // VARIABLES
    SimTK::Vec3 vec,angVec;
    double Mass = 0.0;

    // GROUND BODY
    const Ground &ground = _model->getGround();

    // POSITION
    BodySet& bs = _model->updBodySet();

    for(int i=0;i<_bodyIndices.getSize();i++) {
        Body& body = bs.get(_bodyIndices[i]);
        const SimTK::Vec3& com = body.get_mass_center();
        // GET POSITIONS AND EULER ANGLES
        vec = body.findStationLocationInGround(s, com);
        angVec = body.getTransformInGround(s).R().convertRotationToBodyFixedXYZ();

        // CONVERT TO DEGREES?
        if(getInDegrees()) {
            angVec[0] *= SimTK_RADIAN_TO_DEGREE;
            angVec[1] *= SimTK_RADIAN_TO_DEGREE;
            angVec[2] *= SimTK_RADIAN_TO_DEGREE;
        }           

        // FILL KINEMATICS ARRAY
        int I=6*i;
        memcpy(&_kin[I],&vec[0],3*sizeof(double));
        memcpy(&_kin[I+3],&angVec[0],3*sizeof(double));
    }

    if(_recordCenterOfMass) {
        double rP[3] = { 0.0, 0.0, 0.0 };
        for(int i=0;i<bs.getSize();i++) {
            Body& body = bs.get(i);
            const SimTK::Vec3& com = body.get_mass_center();
            vec = body.findStationLocationInGround(s, com);
            // ADD TO WHOLE BODY MASS
            Mass += body.get_mass();
            rP[0] += body.get_mass() * vec[0];
            rP[1] += body.get_mass() * vec[1];
            rP[2] += body.get_mass() * vec[2];
        }

        //COMPUTE COM OF WHOLE BODY AND ADD TO ARRAY
        rP[0] /= Mass;
        rP[1] /= Mass;
        rP[2] /= Mass;
        int I = 6*_bodyIndices.getSize();
        memcpy(&_kin[I],rP,3*sizeof(double));
    }
    
    _pStore->append(s.getTime(),_kin.getSize(),&_kin[0]);

    // VELOCITY
    for(int i=0;i<_bodyIndices.getSize();i++) {
        Body& body = bs.get(_bodyIndices[i]);
        const SimTK::Vec3& com = body.get_mass_center();
        // GET VELOCITIES AND ANGULAR VELOCITIES
        vec = body.findStationVelocityInGround(s, com);
        angVec = body.getVelocityInGround(s)[0];
        if (_expressInLocalFrame) {
            vec = ground.expressVectorInAnotherFrame(s, vec, body);
            angVec = ground.expressVectorInAnotherFrame(s, angVec, body);
        }

        // CONVERT TO DEGREES?
        if(getInDegrees()) {
            angVec[0] *= SimTK_RADIAN_TO_DEGREE;
            angVec[1] *= SimTK_RADIAN_TO_DEGREE;
            angVec[2] *= SimTK_RADIAN_TO_DEGREE;
        }           

        // FILL KINEMATICS ARRAY
        int I = 6*i;
        memcpy(&_kin[I],&vec[0],3*sizeof(double));
        memcpy(&_kin[I+3],&angVec[0],3*sizeof(double));
    }

    if(_recordCenterOfMass) {
        double rV[3] = { 0.0, 0.0, 0.0 };
        for(int i=0;i<bs.getSize();i++) {
            Body& body = bs.get(i);
            const SimTK::Vec3& com = body.get_mass_center();
            vec = body.findStationVelocityInGround(s, com);
            rV[0] += body.get_mass() * vec[0];
            rV[1] += body.get_mass() * vec[1];
            rV[2] += body.get_mass() * vec[2];
        }

        //COMPUTE VELOCITY OF COM OF WHOLE BODY AND ADD TO ARRAY
        rV[0] /= Mass;
        rV[1] /= Mass;
        rV[2] /= Mass;
        int I = 6*_bodyIndices.getSize();
        memcpy(&_kin[I],rV,3*sizeof(double));
    }

    _vStore->append(s.getTime(),_kin.getSize(),&_kin[0]);

    // ACCELERATIONS
    for(int i=0;i<_bodyIndices.getSize();i++) {
        Body& body = bs.get(_bodyIndices[i]);
        const SimTK::Vec3& com = body.get_mass_center();

        // GET ACCELERATIONS AND ANGULAR ACCELERATIONS
        vec = body.findStationAccelerationInGround(s, com);
        angVec = body.getAccelerationInGround(s)[0];
        if(_expressInLocalFrame) {
            vec = ground.expressVectorInAnotherFrame(s, vec, body);
            angVec = ground.expressVectorInAnotherFrame(s, angVec, body);
        }

        // CONVERT TO DEGREES?
        if(getInDegrees()) {
            angVec[0] *= SimTK_RADIAN_TO_DEGREE;
            angVec[1] *= SimTK_RADIAN_TO_DEGREE;
            angVec[2] *= SimTK_RADIAN_TO_DEGREE;
        }           

        // FILL KINEMATICS ARRAY
        int I = 6*i;
        memcpy(&_kin[I],&vec[0],3*sizeof(double));
        memcpy(&_kin[I+3],&angVec[0],3*sizeof(double));
    }

    if(_recordCenterOfMass) {
        double rA[3] = { 0.0, 0.0, 0.0 };
        for(int i=0;i<bs.getSize();i++) {
            Body& body = bs.get(i);
            const SimTK::Vec3& com = body.get_mass_center();
            vec = body.findStationAccelerationInGround(s, com);
            rA[0] += body.get_mass() * vec[0];
            rA[1] += body.get_mass() * vec[1];
            rA[2] += body.get_mass() * vec[2];
        }

        //COMPUTE ACCELERATION OF COM OF WHOLE BODY AND ADD TO ARRAY
        rA[0] /= Mass;
        rA[1] /= Mass;
        rA[2] /= Mass;
        int I = 6*_bodyIndices.getSize();
        memcpy(&_kin[I],rA,3*sizeof(double));
    }

    _aStore->append(s.getTime(),_kin.getSize(),&_kin[0]);

    //printf("BodyKinematics:\taT:\t%.16f\trA[1]:\t%.16f\n",s.getTime(),rA[1]);
    return(0);
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
 * @param s System state
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
begin(const SimTK::State& s )
{
    if(!proceed()) return(0);

    // RESET STORAGE
    _pStore->reset(s.getTime());
    _vStore->reset(s.getTime());
    _aStore->reset(s.getTime());

    // RECORD
    int status = 0;
    if(_pStore->getSize()<=0) {
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
 * When called during an integration, this method is meant to be called 
 *
 * This method should be overridden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s System state
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
step(const SimTK::State& s, int stepNumber)
{
    if(!proceed(stepNumber )) return(0);

    record(s);

    return(0);
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
 * @param s System state
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
end(const SimTK::State&s )
{
    if(!proceed()) return(0);

    record(s);

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
int BodyKinematics::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    string suffix;
    if(_expressInLocalFrame) suffix = "_bodyLocal";
    else suffix = "_global";

    // Set the file headers just before printing in case the flag
    // _expressInLocalFrame has changed or inDegrees
    constructDescription();
    _aStore->setInDegrees(getInDegrees());
    _vStore->setInDegrees(getInDegrees());
    _pStore->setInDegrees(getInDegrees());

    // ACCELERATIONS
    Storage::printResult(_aStore,aBaseName+"_"+getName()+"_acc"+suffix,aDir,aDT,aExtension);

    // VELOCITIES
    Storage::printResult(_vStore,aBaseName+"_"+getName()+"_vel"+suffix,aDir,aDT,aExtension);

    // POSITIONS
    Storage::printResult(_pStore,aBaseName+"_"+getName()+"_pos_global",aDir,aDT,aExtension);

    return(0);
}


