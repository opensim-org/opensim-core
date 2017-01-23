/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CMC_Point.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Contributor(s): Frank C. Anderson, Jeffrey A. Reinbolt                     *
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include "CMC_Point.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CMC_Point::~CMC_Point()
{
}
//_____________________________________________________________________________
/**
 * Construct a task for a specified point.
 *
 */
CMC_Point::CMC_Point(const SimTK::Vec3 &aPoint) :
    _point(_propPoint.getValueDblVec())
{
    setNull();
    setPoint(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Point task to be copied.
 */
CMC_Point::CMC_Point(const CMC_Point &aTask) :
    CMC_Task(aTask),
    _point(_propPoint.getValueDblVec())
{
    setNull();
    copyData(aTask);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void CMC_Point::
setNull()
{
    setupProperties();

    _nTrk = 3;
    _p = 0;
    _v = 0;
}
//_____________________________________________________________________________
/**
 * Set up serialized member variables.
 */
void CMC_Point::
setupProperties()
{
    _propPoint.setComment("Point in body frame with respect to which an objective is tracked.");
    _propPoint.setName("point");
    _propPoint.setValue(SimTK::Vec3(0));
    _propertySet.append(&_propPoint);
}

//_____________________________________________________________________________
/**
 * Copy only the member data of specified object.
 */
void CMC_Point::
copyData(const CMC_Point &aTask)
{
    setPoint(aTask.getPoint());
}


//_____________________________________________________________________________
/**
 * Update work variables
 */
void CMC_Point::
updateWorkVariables(const SimTK::State& s)
{
    _p = 0;
    _v = 0;
    if(_model) {

        BodySet& bs = _model->updBodySet();
    
        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

        if(_wrtBodyName == "center_of_mass") {
            SimTK::Vec3 pVec,vVec;
            double Mass = 0.0;
            // double rP[3] = { 0.0, 0.0, 0.0 };
            for(int i=0;i<bs.getSize();i++) {
                Body& body = bs.get(i);
                const SimTK::Vec3& com = body.get_mass_center();
                pVec = body.findStationLocationInGround(s, com);
                if(pVec[0] != pVec[0]) throw Exception("CMC_Point.computeAccelerations: ERROR- point task '" + getName() 
                                            + "' references invalid acceleration components",__FILE__,__LINE__);
                vVec = body.findStationVelocityInGround(s, com);
                if(vVec[0] != vVec[0]) throw Exception("CMC_Point.computeAccelerations: ERROR- point task '" + getName() 
                                            + "' references invalid acceleration components",__FILE__,__LINE__);
                // ADD TO WHOLE BODY MASS
                Mass += body.get_mass();
                _p += body.get_mass() * pVec;
                _v += body.get_mass() * vVec;
            }

            //COMPUTE COM OF WHOLE BODY
            _p /= Mass;
            _v /= Mass;

        } else {

            _wrtBody =  &bs.get(_wrtBodyName);

            _p = _wrtBody->findStationLocationInGround(s, _point);
            if(_p[0] != _p[0]) throw Exception("CMC_Point.updateWorkVariables: ERROR- point task '" + getName() 
                                                + "' references invalid position components",__FILE__,__LINE__);
            _v = _wrtBody->findStationVelocityInGround(s, _point);
            if(_v[0] != _v[0]) throw Exception("CMC_Point.updateWorkVariables: ERROR- point task '" + getName() 
                                                + "' references invalid velocity components",__FILE__,__LINE__);

        }
    }
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @param aTask Object to be copied.
 * @return  Reference to the altered object.
 */
CMC_Point& CMC_Point::
operator=(const CMC_Point &aTask)
{
    // BASE CLASS
    CMC_Task::operator =(aTask);

    // DATA
    copyData(aTask);

    return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Initializes pointers to the Coordinate and Speed in the model given the 
 * coordinate name assigned to this task.
 *
 * @param aModel Model.
 */
void CMC_Point::
setModel(Model& aModel)
{
    CMC_Task::setModel(aModel);
}
//-----------------------------------------------------------------------------
// Point
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the point that is to be tracked.
 *
 * @param aPoint Components of the tracked point.
 */
void CMC_Point::
setPoint(const SimTK::Vec3 &aPoint)
{
    _point = aPoint;
}
//_____________________________________________________________________________
/**
 * Get the point that is to be tracked.
 *
 * @return Components of the tracked point.
 */
SimTK::Vec3 CMC_Point::
getPoint() const
{
    return(_point);
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the position and velocity errors.
 * This method assumes the states have been set for the model.
 * 
 * @param aT Current time in real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void CMC_Point::
computeErrors(const SimTK::State& s, double aT)
{
    updateWorkVariables(s);
    // COMPUTE ERRORS
    //std::cout<<getName()<<std::endl;
    //std::cout<<"_pTrk[0]->evaluate(0,aT) = "<<_pTrk[0]->evaluate(0,aT)<<std::endl;
    //std::cout<<"_pTrk[1]->evaluate(0,aT) = "<<_pTrk[1]->evaluate(0,aT)<<std::endl;
    //std::cout<<"_pTrk[2]->evaluate(0,aT) = "<<_pTrk[2]->evaluate(0,aT)<<std::endl;
    //std::cout<<"_inertialPTrk[0] = "<<_inertialPTrk[0]<<std::endl;
    //std::cout<<"_inertialPTrk[1] = "<<_inertialPTrk[1]<<std::endl;
    //std::cout<<"_inertialPTrk[2] = "<<_inertialPTrk[2]<<std::endl;
    //std::cout<<"_p = "<<_p<<std::endl;

    BodySet& bs = _model->updBodySet();

    _inertialPTrk = 0;
    _inertialVTrk = 0;
    if(_expressBodyName == "ground") {

        for(int i=0;i<3;i++) {
            _inertialPTrk[i] = _pTrk[i]->calcValue(SimTK::Vector(1,aT));
            if(_vTrk[i]==NULL) {
                std::vector<int> derivComponents(1);
                derivComponents[0]=0;
                _inertialVTrk[i] = _pTrk[i]->calcDerivative(derivComponents,SimTK::Vector(1,aT));
            } else {
                _inertialVTrk[i] = _vTrk[i]->calcValue(SimTK::Vector(1,aT));
            }
        }

    } else {

        _expressBody =  &bs.get(_expressBodyName);

        SimTK::Vec3 pVec,vVec,origin;

        for(int i=0;i<3;i++) {
            pVec(i) = _pTrk[i]->calcValue(SimTK::Vector(1,aT));
        }
        _inertialPTrk = _expressBody->findStationLocationInGround(s, pVec);
        if(_vTrk[0]==NULL) {
            _inertialVTrk = _expressBody->findStationVelocityInGround(s, pVec);
        } else {
            for(int i=0;i<3;i++) {
                vVec(i) = _vTrk[i]->calcValue(SimTK::Vector(1,aT));
            }
            _inertialVTrk = _expressBody->findStationVelocityInGround(s, origin); // get velocity of _expressBody origin in inertial frame
            _inertialVTrk += vVec; // _vTrk is velocity in _expressBody, so it is simply added to velocity of _expressBody origin in inertial frame
        }

    }

    _pErr[0] = 0.0;
    _vErr[0] = 0.0;
    for(int j=0; j<3; j++) {
        _pErr[0] += _inertialPTrk[j]*_r0[j] - _p[j]*_r0[j];
        _vErr[0] += _inertialVTrk[j]*_r0[j] - _v[j]*_r0[j];
    }
    _pErr[1] = 0.0;
    _vErr[1] = 0.0;
    for(int j=0; j<3; j++) {
        _pErr[1] += _inertialPTrk[j]*_r1[j] - _p[j]*_r1[j];
        _vErr[1] += _inertialVTrk[j]*_r1[j] - _v[j]*_r1[j];
    }
    _pErr[2] = 0.0;
    _vErr[2] = 0.0;
    for(int j=0; j<3; j++) {
        _pErr[2] += _inertialPTrk[j]*_r2[j] - _p[j]*_r2[j];
        _vErr[2] += _inertialVTrk[j]*_r2[j] - _v[j]*_r2[j];
    }

}
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 */
void CMC_Point::
computeDesiredAccelerations(const SimTK::State& s, double aT)
{
    _aDes=SimTK::NaN;

    // CHECK
    if(_model==NULL) return;
    if(_pTrk[0]==NULL) return;

    // COMPUTE ERRORS
    computeErrors(s, aT);

    // DESIRED ACCELERATION
    double p;
    double v;
    double a;
    for(int i=0; i<3; i++) {
        p = (_kp)[0]*_pErr[i];
        v = (_kv)[0]*_vErr[i];
        if(_aTrk[i]==NULL) {
            std::vector<int> derivComponents(2);
            derivComponents[0]=0;
            derivComponents[1]=0;
            a = (_ka)[0]*_pTrk[i]->calcDerivative(derivComponents,SimTK::Vector(1,aT));
        } else {
            a = (_ka)[0]*_aTrk[i]->calcValue(SimTK::Vector(1,aT));
        }
        _aDes[i] = a + v + p;
    }

    // PRINT
    //printf("CMC_Point.computeDesiredAcceleration:\n");
    //printf("%s:  t=%lf aDes=%lf a=%lf vErr=%lf pErr=%lf\n",getName(),t,_aDes[0],
    //  _pTrk[0]->evaluate(2,t),_vErr[0],_pErr[0]);
}
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 * This method assumes that the states have been set for the model.
 *
 * @param aTI Initial time of the controlled interval in real time units.
 * @param aTF Final time of the controlled interval in real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void CMC_Point::
computeDesiredAccelerations(const SimTK::State& s, double aTI,double aTF)
{
    _aDes=SimTK::NaN;

    // CHECK
    if(_model==NULL) return;
    if(_pTrk[0]==NULL) return;

    // COMPUTE ERRORS
    computeErrors(s, aTI);

    // DESIRED ACCELERATION
    double p;
    double v;
    double a;
    for(int i=0; i<3; i++) {
        p = (_kp)[0]*_pErr[i];
        v = (_kv)[0]*_vErr[i];
        if(_aTrk[i]==NULL) {
            std::vector<int> derivComponents(2);
            derivComponents[0]=0;
            derivComponents[1]=0;
            a = (_ka)[0]*_pTrk[i]->calcDerivative(derivComponents,SimTK::Vector(1,aTF));
        } else {
            a = (_ka)[0]*_aTrk[i]->calcValue(SimTK::Vector(1,aTF));
        }
        _aDes[i] = a + v + p;
    }

    // PRINT
    //printf("CMC_Point.computeDesiredAcceleration:\n");
    //printf("%s:  t=%lf aDes=%lf a=%lf vErr=%lf pErr=%lf\n",getName(),t,_aDes[0],
    //  _pTrk[0]->evaluate(2,t),_vErr[0],_pErr[0]);
}

//_____________________________________________________________________________
/**
 * Compute the acceleration of the appropriate point.
 * For the computed accelerations to be correct,
 * Model::computeAccelerations() must have already been called.
 *
 * For joints (i.e., generalized coordinates), the acceleration is
 * not computed.  It has already been computed and is simply retrieved
 * from the model.
 *
 * @see Model::computeAccelerations()
 * @see suTrackObject::getAcceleration()
 */
void CMC_Point::
computeAccelerations(const SimTK::State& s )
{
    // CHECK
    if(_model==NULL) return;

    // ACCELERATION
    _a = 0;
    BodySet& bs = _model->updBodySet();
    if(_wrtBodyName == "center_of_mass") {

        SimTK::Vec3 pVec,vVec,aVec,com;
        double Mass = 0.0;
        for(int i=0;i<bs.getSize();i++) {
            Body& body = bs.get(i);
            com = body.get_mass_center();
            aVec = body.findStationAccelerationInGround(s, com);
            if(aVec[0] != aVec[0]) throw Exception("CMC_Point.computeAccelerations: ERROR- point task '" + getName() 
                                            + "' references invalid acceleration components",__FILE__,__LINE__);
            // ADD TO WHOLE BODY MASS
            Mass += body.get_mass();
            _a += body.get_mass() * aVec;
        }

        //COMPUTE COM ACCELERATION OF WHOLE BODY
        _a /= Mass;

    } else {

        _wrtBody =  &bs.get(_wrtBodyName);
        _a = _wrtBody->findStationAccelerationInGround(s, _point);
        if(_a[0] != _a[0]) throw Exception("CMC_Point.computeAccelerations: ERROR- point task '" + getName() 
                                            + "' references invalid acceleration components",__FILE__,__LINE__);
    }
}

//=============================================================================
// XML
//=============================================================================
//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 *
 * @param aDeep If true, update this object and all its child objects
 * (that is, member variables that are Object's); if false, update only
 * the member variables that are not Object's.
 */
void CMC_Point::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    CMC_Task::updateFromXMLNode(aNode, versionNumber);
    setPoint(_point);
}
