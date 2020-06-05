/* -------------------------------------------------------------------------- *
 *                       OpenSim:  PointKinematics.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
#include <string>
#include <OpenSim/Simulation/Model/Model.h>
#include "PointKinematics.h"


using namespace OpenSim;
using namespace std;
using SimTK::Vec3;

//=============================================================================
// CONSTANTS
//=============================================================================
const int PointKinematics::NAME_LENGTH = PointKinematicsNAME_LENGTH;
const int PointKinematics::BUFFER_LENGTH = PointKinematicsBUFFER_LENGTH;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PointKinematics::~PointKinematics()
{
    deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an PointKinematics instance for recording the kinematics of
 * the bodies of a model during a simulation. Also serves as a default constructor
 *
 * @param aModel Model for which the analyses are to be recorded.
 */
PointKinematics::PointKinematics(Model *aModel) :
Analysis(aModel),
_body(NULL),
_relativeToBody(NULL),
_bodyName(_bodyNameProp.getValueStr()),
_point(_pointProp.getValueDblVec()),
_pointName(_pointNameProp.getValueStr()),
_relativeToBodyName(_relativeToBodyNameProp.getValueStr())
{
    // NULL
    setNull();

    // STORAGE
    allocateStorage();

    if (aModel==0)
        return;

}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//=============================================================================
// Object Overrides
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
PointKinematics::PointKinematics(const std::string &aFileName):
Analysis(aFileName, false),
_body(NULL),
_relativeToBody(NULL),
_bodyName(_bodyNameProp.getValueStr()),
_point(_pointProp.getValueDblVec()),
_pointName(_pointNameProp.getValueStr()),
_relativeToBodyName(_relativeToBodyNameProp.getValueStr())
{
    setNull();

    // Serialize from XML
    updateFromXMLDocument();

    /* The rest will be done by setModel().
    // CONSTRUCT DESCRIPTION AND LABELS
    constructDescription();
    constructColumnLabels();

    // STORAGE
    allocateStorage();
    */
}

// Copy constructor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
PointKinematics::PointKinematics(const PointKinematics &aPointKinematics):
Analysis(aPointKinematics),
_body(aPointKinematics._body),
_relativeToBody(aPointKinematics._relativeToBody),
_bodyName(_bodyNameProp.getValueStr()),
_point(_pointProp.getValueDblVec()),
_pointName(_pointNameProp.getValueStr()),
_relativeToBodyName(_relativeToBodyNameProp.getValueStr())
{
    setNull();

    // COPY TYPE AND NAME
    *this = aPointKinematics;
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void PointKinematics::
setNull()
{
    // POINTERS
    _kin = NULL;
    _pStore = NULL;
    _vStore = NULL;
    _aStore = NULL;

    // OTHER VARIABLES

    //?_body
    setName("PointKinematics");

    // POINT INFORMATION
    //_point.setSize(3);
    Vec3 zero3(0.0);    
    _bodyNameProp.setName("body_name");
    _bodyNameProp.setValue("ground");
    _propertySet.append( &_bodyNameProp );

    _relativeToBodyNameProp.setName("relative_to_body_name");
    _relativeToBodyNameProp.setValue("none");
    _propertySet.append( &_relativeToBodyNameProp );

    _pointNameProp.setName("point_name");
    _pointNameProp.setValue("NONAME");
    _propertySet.append( &_pointNameProp );

    _pointProp.setName("point");
    _pointProp.setValue(zero3);
    _propertySet.append( &_pointProp );
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
PointKinematics& PointKinematics::
operator=(const PointKinematics &aPointKinematics)
{
    // BASE CLASS
    Analysis::operator=(aPointKinematics);
    _body = aPointKinematics._body;
    _relativeToBody = aPointKinematics._relativeToBody;
    _point = aPointKinematics._point;
    _pointName = aPointKinematics._pointName;
    _bodyName = aPointKinematics._bodyName;
    _relativeToBodyName = aPointKinematics._relativeToBodyName;

    // STORAGE
    deleteStorage();
    allocateStorage();

    return(*this);
}
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files. (needs a model)
 */
void PointKinematics::
constructDescription()
{
    char descrip[BUFFER_LENGTH];
    char tmp[BUFFER_LENGTH];

    strcpy(descrip,"\nThis file contains the kinematics ");
    strcat(descrip,"(position, velocity, or acceleration) of\n");
    
    if(_relativeToBody){
        sprintf(tmp,"point (%lf, %lf, %lf) on body %s relative to body %s of model %s.\n",
            _point[0],_point[1],_point[2],_body->getName().c_str(),
            _relativeToBody->getName().c_str(), _model->getName().c_str());
    }
    else{
        sprintf(tmp,"point (%lf, %lf, %lf) on the %s of model %s.\n",
            _point[0],_point[1],_point[2],_body->getName().c_str(),
            _model->getName().c_str());
    }
    strcat(descrip,tmp);
    strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons,...)\n\n");

    setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body kinematics files.
 */
void PointKinematics::
constructColumnLabels()
{
    Array<string> labels;
    labels.append("time");
    labels.append(getPointName() + "_X");
    labels.append(getPointName() + "_Y");
    labels.append(getPointName() + "_Z");
    setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void PointKinematics::
allocateStorage()
{
    // ACCELERATIONS
    _aStore = new Storage(1000,"PointAcceleration");
    _aStore->setDescription(getDescription());
    _aStore->setColumnLabels(getColumnLabels());

    // VELOCITIES
    _vStore = new Storage(1000,"PointVelocity");
    _vStore->setDescription(getDescription());
    _vStore->setColumnLabels(getColumnLabels());

    // POSITIONS
    _pStore = new Storage(1000,"PointPosition");
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
void PointKinematics::
deleteStorage()
{
    if(_aStore!=NULL) { delete _aStore;  _aStore=NULL; }
    if(_vStore!=NULL) { delete _vStore;  _vStore=NULL; }
    if(_pStore!=NULL) { delete _pStore;  _pStore=NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/* Set the model for which the point kinematics are to be computed. */
void PointKinematics::setModel(Model& model)
{
    Analysis::setModel(model);

    _body = nullptr;
    _relativeToBody = nullptr;

    // Map name to index
    if (model.hasComponent<PhysicalFrame>(_bodyName))
        _body = &model.getComponent<PhysicalFrame>(_bodyName);
    else if (model.hasComponent<PhysicalFrame>("./bodyset/" + _bodyName))
        _body = &model.getComponent<PhysicalFrame>("./bodyset/"+_bodyName);

    if (model.hasComponent<PhysicalFrame>(_relativeToBodyName))
        _relativeToBody = &model.getComponent<PhysicalFrame>(
            _relativeToBodyName);
    else if(model.hasComponent<PhysicalFrame>("./bodyset/" + _relativeToBodyName))
        _relativeToBody = &model.getComponent<PhysicalFrame>(
            "./bodyset/" + _relativeToBodyName);

    // DESCRIPTION AND LABELS
    constructDescription();
    constructColumnLabels();
}
//-----------------------------------------------------------------------------
// BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the body for which the point kinematics are to be computed and the point on the body
 * represented in local frame. Both params are required to avoid the limbo state where either body
 * or point are undefined..
 *
 * @param aBody Body name
 * @double[3] aPoint point coordinates
 */
void PointKinematics::
setBodyPoint(const std::string& aBody, const SimTK::Vec3& aPoint)
{
    if (_model == 0)
        return;
    setBody(&_model->updBodySet().get(aBody));
    setPoint(aPoint);

}
//_____________________________________________________________________________
/**
 * Set the body for which the induced accelerations are to be computed.
 *
 * @param aBody Body ID
 */
void PointKinematics::setBody(const PhysicalFrame* aBody)
{
    // CHECK
    if (aBody==NULL) {
        log_warn("PointKinematics.setBody: null body pointer.");
        _body = NULL;
        return;
    }

    // SET
    _body = aBody;
    _bodyName = _body->getName();
    log_info("PointKinematics.setBody: set body to {}.", _bodyName);
}
void PointKinematics::setRelativeToBody(const PhysicalFrame* aBody)
{
    // CHECK
    if (aBody==NULL) {
        log_warn("PointKinematics.setRelativeToBody: null body pointer.");
        _body = NULL;
        return;
    }

    // SET
    _relativeToBody = aBody;
    _relativeToBodyName = aBody->getName();
    log_info("PointKinematics.setRelativeToBody: set relative-to body to {}.",
            _bodyName);
}

//_____________________________________________________________________________
/**
 * Get the body for which the induced accelerations are to be computed.
 *
 * @return Body pointer
 */
const PhysicalFrame* PointKinematics::getBody() const
{
    return(_body);
}

const PhysicalFrame* PointKinematics::getRelativeToBody() const
{
    return(_relativeToBody);
}

//-----------------------------------------------------------------------------
// POINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the point for which the induced accelerations are to be computed.
 *
 * @param aPoint X-Y-Z Point
 */
void PointKinematics::
setPoint(const SimTK::Vec3& aPoint)
{
    _point = aPoint;
}
//_____________________________________________________________________________
/**
 * Get the point for which the induced accelerations are to be computed.
 *
 * @param rPoint X-Y-Z Point
 */
void PointKinematics::
getPoint(SimTK::Vec3& rPoint)
{
    rPoint = _point;
}

//-----------------------------------------------------------------------------
// POINT NAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a name for the point.
 *
 * @param aName Name for the point.
 */
void PointKinematics::
setPointName(const string &aName)
{
    _pointName = aName;
    constructColumnLabels();
    if(_aStore!=NULL) _aStore->setColumnLabels(getColumnLabels());
    if(_vStore!=NULL) _vStore->setColumnLabels(getColumnLabels());
    if(_pStore!=NULL) _pStore->setColumnLabels(getColumnLabels());
}
//_____________________________________________________________________________
/**
 * Get the point name.
 *
 * @param aName Name for the point.
 */
const std::string &PointKinematics::
getPointName()
{
    return(_pointName);
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
Storage* PointKinematics::
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
Storage* PointKinematics::
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
Storage* PointKinematics::
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
void PointKinematics::
setStorageCapacityIncrements(int aIncrement)
{
    _aStore->setCapacityIncrement(aIncrement);
    _vStore->setCapacityIncrement(aIncrement);
    _pStore->setCapacityIncrement(aIncrement);
}



//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the kinematics.
 */
int PointKinematics::
record(const SimTK::State& s)
{
    // VARIABLES
    SimTK::Vec3 vec;

    const double& time = s.getTime();
    const Ground& ground = _model->getGround();

    // POSITION
    vec = _body->findStationLocationInGround(s, _point);
    if(_relativeToBody){
        vec = ground.findStationLocationInAnotherFrame(s, vec, *_relativeToBody);
    }

    _pStore->append(time, vec);

    // VELOCITY
    vec = _body->findStationVelocityInGround(s, _point);
    if(_relativeToBody){
        vec = ground.expressVectorInAnotherFrame(s, vec, *_relativeToBody);
    }

    _vStore->append(time, vec);

    // ACCELERATIONS
    _model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    vec = _body->findStationAccelerationInGround(s, _point);
    if(_relativeToBody){
        vec = ground.expressVectorInAnotherFrame(s, vec, *_relativeToBody);
    }

    _aStore->append(time, vec);

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
 * @param s current system state
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int PointKinematics::
begin( const SimTK::State& s)
{
    if(!proceed()) return(0);

    // RESET STORAGE
    _pStore->reset(s.getTime());
    _vStore->reset(s.getTime());
    _aStore->reset(s.getTime());

    int status = record(s);

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
 * @param s current state of system
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int PointKinematics::
step(const SimTK::State& s, int stepNumber)
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
 * This method is meant to be called at the end of an integration 
 *
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current state of system
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int PointKinematics::
end( const SimTK::State& s)
{
    if(!proceed()) return(0);
    record(s);
    log_info("PointKinematics.end: Finalizing analysis {}.", getName());
    return 0 ;
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
int PointKinematics::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    // ACCELERATIONS
    Storage::printResult(_aStore,aBaseName+"_"+getName()+"_"+getPointName()+"_acc",aDir,aDT,aExtension);

    // VELOCITIES
    Storage::printResult(_vStore,aBaseName+"_"+getName()+"_"+getPointName()+"_vel",aDir,aDT,aExtension);

    // POSITIONS
    Storage::printResult(_pStore,aBaseName+"_"+getName()+"_"+getPointName()+"_pos",aDir,aDT,aExtension);

    return(0);
}


