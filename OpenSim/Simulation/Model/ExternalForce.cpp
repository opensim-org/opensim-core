/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ExternalForce.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/GCVSpline.h>

#include "ExternalForce.h"

//==============================================================================
// USING
//==============================================================================
using namespace OpenSim;
using SimTK::Vec3;
using namespace std;

//==============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
/**
 * Default constructor.
 */
ExternalForce::ExternalForce() : Force()
{
    setNull();
    constructProperties();
}

/**
 * Convenience Constructor of an ExternalForce. 
 * 
 */
ExternalForce::ExternalForce
   (const Storage &dataSource, const string& forceIdentifier, 
    const string& pointIdentifier, const string& torqueIdentifier, 
    const string& appliedToBodyName, const string& forceExpressedInBodyName, 
    const string& pointExpressedInBodyName)
{
    setNull();
    constructProperties();
    _dataSource = &dataSource;

    set_applied_to_body(appliedToBodyName);
    set_force_expressed_in_body(forceExpressedInBodyName);
    set_point_expressed_in_body(pointExpressedInBodyName);
    set_data_source_name(dataSource.getName());
    set_force_identifier(forceIdentifier);
    set_point_identifier(pointIdentifier);
    set_torque_identifier(torqueIdentifier);
}


//_____________________________________________________________________________
/**
 * Constructor from XML file
 */
ExternalForce::ExternalForce(SimTK::Xml::Element& node) : Super(node)
{
    setNull();
    constructProperties();
    updateFromXMLNode(node, -1);
}

void ExternalForce::setNull()
{
    setAuthors("Ajay Seth");
    _dataSource = nullptr;
    _appliedToBody = nullptr;
    _forceExpressedInBody = nullptr;
    _pointExpressedInBody = nullptr; 
}

/**
 * Connect properties to local pointers.
 */
void ExternalForce::constructProperties()
{
    constructProperty_applied_to_body("");
    constructProperty_force_expressed_in_body("ground");
    constructProperty_point_expressed_in_body("ground");
    constructProperty_force_identifier();
    constructProperty_point_identifier();
    constructProperty_torque_identifier();
    constructProperty_data_source_name();
}

void ExternalForce::setDataSource(const Storage &dataSource)
{ 
    _dataSource = &dataSource;

    log_info("ExternalForce::{} Data source being set to {}", 
        getName(), _dataSource->getName());   

    set_data_source_name(_dataSource->getName());
}

void ExternalForce::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    if( getProperty_force_identifier().empty() 
        && getProperty_torque_identifier().empty()){
        OPENSIM_THROW_FRMOBJ(InvalidPropertyValue, 
            getName(), "ExternalForce:: no force or torque identified.");
    }
    
    _appliesForce = appliesForce();
    _specifiesPoint = specifiesPoint();
    _appliesTorque = appliesTorque();
}

void ExternalForce::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    //TODO Use Sockets!
    const string& appliedToBodyName = get_applied_to_body();
    const string& forceExpressedInBodyName = get_force_expressed_in_body();
    const string& pointExpressedInBodyName =  get_point_expressed_in_body();

    // This might not have been supplied in which case it will have size()==0.
    const Property<string>& dataSourceProp = getProperty_data_source_name();

    // hook up body pointers from names
    _appliedToBody.reset();
    _forceExpressedInBody.reset();
    _pointExpressedInBody.reset();
    if (hasModel()) {
        if (getModel().hasComponent<PhysicalFrame>(appliedToBodyName))
            _appliedToBody = &_model->getComponent<PhysicalFrame>(
                    appliedToBodyName);
        else if(getModel().hasComponent<PhysicalFrame>(
            "./bodyset/" + appliedToBodyName))
            _appliedToBody = &getModel().getComponent<PhysicalFrame>(
                "./bodyset/" + appliedToBodyName);

        if (getModel().hasComponent<PhysicalFrame>(forceExpressedInBodyName))
            _forceExpressedInBody = &getModel().getComponent<PhysicalFrame>(
                    forceExpressedInBodyName);
        else if(getModel().hasComponent<PhysicalFrame>(
            "./bodyset/" + forceExpressedInBodyName))
            _forceExpressedInBody = &_model->getComponent<PhysicalFrame>(
                "./bodyset/" + forceExpressedInBodyName);

        if (_specifiesPoint) {
            if (getModel().hasComponent<PhysicalFrame>(
                pointExpressedInBodyName))
                _pointExpressedInBody = &_model->getComponent<PhysicalFrame>(
                    pointExpressedInBodyName);
            else if (getModel().hasComponent<PhysicalFrame>(
                "./bodyset/" + pointExpressedInBodyName))
                _pointExpressedInBody = &_model->getComponent<PhysicalFrame>(
                    "./bodyset/" + pointExpressedInBodyName);
        }
    }

    if(!_appliedToBody){
        throw(Exception("ExternalForce: Could not find body '" + appliedToBodyName +
            "' to apply force to." ));
    }
    if(!_forceExpressedInBody) {
        log_warn("ExternalForce could not find body '{}' that force is "
                 "expressed in-ground is being assumed.",
                forceExpressedInBodyName);
        _forceExpressedInBody = &_model->getGround();
    }
    if(_specifiesPoint && !_pointExpressedInBody){
        log_warn("ExternalForce could not find body '{}' that point is "
                 "expressed in-ground is being assumed.",
                get_point_expressed_in_body());
        _pointExpressedInBody = &_model->getGround();
    }

    if(!_dataSource){
        throw Exception("ExternalForce: No Data source has been set.");
        // TODO: Load the data from dataSourceProp. Currently this is overly
        // complicated and handled by the ExternalLoads class.
    }
    else if(dataSourceProp.size()) {
        const string& dataSourceName = dataSourceProp.getValue();
        if (_dataSource->getName() != dataSourceName)
            throw(Exception("ExternalForce: Data source "+dataSourceName
              +" specified by name, but "+_dataSource->getName()+" was set." ));
    }
    else{
         set_data_source_name(_dataSource->getName());
    }

    // temporary data arrays
    Array<double> time;
    Array<Array<double> > force;
    Array<Array<double> > point;
    Array<Array<double> > torque;

    // Get data
    _dataSource->getTimeColumn(time);
    int nt = time.getSize();
    if( nt < 1)
        throw(Exception("ExternalForce: No times found in data source: "+_dataSource->getName()));

    // have to apply either a force or a torque
    if(!_appliesForce && !_appliesTorque)
        throw(Exception("ExternalForce:"+getName()+" does not apply neither a force nor a torque.")); 

    // if a force is not being applied then specifying a point makes no sense
    if(!_appliesForce && _specifiesPoint)
        throw(Exception("ExternalForce:"+getName()+" Point is specified for no applied force.")); 

    
    if(_appliesForce){ // if applying force MUST have 3 components
        _dataSource->getDataForIdentifier(get_force_identifier(), force);
        if(force.getSize() != 3)
            throw(Exception("ExternalForce: 3 unique force components could not be found, for force identifier: "
                +get_force_identifier()+
                "\n. Please make sure data file contains exactly 3 unique columns with this common prefix."));
    }

    if(_specifiesPoint){
        _dataSource->getDataForIdentifier(get_point_identifier(), point);
        if(point.getSize() != 3) // if specifying a point of application, it MUST have 3 components
            throw(Exception("ExternalForce: 3 unique point components could not be found, for point identifier: "
            +get_point_identifier()+
            "\n. Please make sure data file contains exactly 3 unique columns with this common prefix."));
    }

    if(_appliesTorque){
        _dataSource->getDataForIdentifier(get_torque_identifier(), torque);
        if(torque.getSize() != 3) // if specifying a torque to be applied, it MUST have 3 components
        throw(Exception("ExternalForce: 3 unique torque components could not be identified for torque identifier: "
            +get_torque_identifier()+
            "\n. Please make sure data file contains exactly 3 unique columns with this common prefix."));
    }

    // clear out functions from previous data source
    _forceFunctions.clearAndDestroy();
    _pointFunctions.clearAndDestroy();
    _torqueFunctions.clearAndDestroy();

    // Create functions now that we should have good data remaining
    if(_appliesForce){
        for(int i=0; i<3; ++i){
            switch(nt) {
                case 1 :
                    _forceFunctions.append(new Constant(force[i][0]));
                    break;
                case 2 :
                    _forceFunctions.append(new PiecewiseLinearFunction(force[i].getSize(), &time[0], &(force[i][0])) );
                    break;
                case 3 :
                    _forceFunctions.append(new PiecewiseLinearFunction(force[i].getSize(), &time[0], &(force[i][0])) );
                    break;
                default:
                    _forceFunctions.append(new GCVSpline( 3, force[i].getSize(), &time[0], &(force[i][0])) );
            }   
        }

        if(_specifiesPoint){
            for(int i=0; i<3; ++i){
                switch(nt) {
                case 1:
                    _pointFunctions.append(new Constant(point[i][0]));
                    break;
                case 2:
                    _pointFunctions.append(new PiecewiseLinearFunction(point[i].getSize(), &time[0], &(point[i][0])) );
                    break;
                case 3:
                    _pointFunctions.append(new PiecewiseLinearFunction(point[i].getSize(), &time[0], &(point[i][0])) );
                    break;
                default:
                    _pointFunctions.append(new GCVSpline( 3, point[i].getSize(), &time[0], &(point[i][0])) );
                }
            }
        }
    }
    if(_appliesTorque){
        for(int i=0; i<3; ++i){
            switch(nt) {
                case 1:
                    _torqueFunctions.append(new Constant(torque[i][0]));
                    break;
                case 2:
                    _torqueFunctions.append(new PiecewiseLinearFunction(torque[i].getSize(), &time[0], &(torque[i][0])) );
                    break;
                case 3:
                    _torqueFunctions.append(new PiecewiseLinearFunction(torque[i].getSize(), &time[0], &(torque[i][0])) );
                    break;
                default:
                    _torqueFunctions.append(new GCVSpline( 3, torque[i].getSize(), &time[0], &(torque[i][0])) );
            }
        }
    }
}



//-----------------------------------------------------------------------------
// FORCE METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void ExternalForce::computeForce(const SimTK::State& state, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    double time = state.getTime();

    assert(_appliedToBody!=nullptr);

    if (_appliesForce) {
        Vec3 force = getForceAtTime(time);
        force = _forceExpressedInBody->expressVectorInGround(state, force);
        Vec3 point(0); // Default is body origin.
        if (_specifiesPoint) {
            point = getPointAtTime(time);
            point = _pointExpressedInBody->
                findStationLocationInAnotherFrame(state, point, *_appliedToBody);
        }
        applyForceToPoint(state, *_appliedToBody, point, force, bodyForces);
    }

    if (_appliesTorque) {
        Vec3 torque = getTorqueAtTime(time);
        torque = _forceExpressedInBody->expressVectorInGround(state, torque);
        applyTorque(state, *_appliedToBody, torque, bodyForces);
    }
}

/**
 * Convenience methods to access prescribed force functions
 */
Vec3 ExternalForce::getForceAtTime(double aTime) const  
{
    SimTK::Vector timeAsVector(1, aTime);
    const Function* forceX=NULL;
    const Function* forceY=NULL;
    const Function* forceZ=NULL;
    if (_forceFunctions.size()==3){
        forceX=_forceFunctions[0];  forceY=_forceFunctions[1];  forceZ=_forceFunctions[2];
    }
    Vec3 force(forceX?forceX->calcValue(timeAsVector):0.0, 
        forceY?forceY->calcValue(timeAsVector):0.0, 
        forceZ?forceZ->calcValue(timeAsVector):0.0);
    return force;
}

Vec3 ExternalForce::getPointAtTime(double aTime) const
{
    SimTK::Vector timeAsVector(1, aTime);
    const Function* pointX=NULL;
    const Function* pointY=NULL;
    const Function* pointZ=NULL;
    if (_pointFunctions.size()==3){
        pointX=_pointFunctions[0];  pointY=_pointFunctions[1];  pointZ=_pointFunctions[2];
    }
    Vec3 point(pointX?pointX->calcValue(timeAsVector):0.0, 
        pointY?pointY->calcValue(timeAsVector):0.0, 
        pointZ?pointZ->calcValue(timeAsVector):0.0);
    return point;
}

Vec3 ExternalForce::getTorqueAtTime(double aTime) const
{
    SimTK::Vector timeAsVector(1, aTime);
    const Function* torqueX=NULL;
    const Function* torqueY=NULL;
    const Function* torqueZ=NULL;
    if (_torqueFunctions.size()==3){
        torqueX=_torqueFunctions[0];    torqueY=_torqueFunctions[1];    torqueZ=_torqueFunctions[2];
    }
    Vec3 torque(torqueX?torqueX->calcValue(timeAsVector):0.0, 
        torqueY?torqueY->calcValue(timeAsVector):0.0, 
        torqueZ?torqueZ->calcValue(timeAsVector):0.0);
    return torque;
}


//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------

OpenSim::Array<std::string> ExternalForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");

    const string& appliedToBodyName = get_applied_to_body();

    if (_appliesForce) {
        labels.append(appliedToBodyName+"_"+getName()+"_Fx");
        labels.append(appliedToBodyName+"_"+getName()+"_Fy");
        labels.append(appliedToBodyName+"_"+getName()+"_Fz");

        if (_specifiesPoint) {
            labels.append(appliedToBodyName+"_"+getName()+"_px");
            labels.append(appliedToBodyName+"_"+getName()+"_py");
            labels.append(appliedToBodyName+"_"+getName()+"_pz");
        }
    }
    if (_appliesTorque){
        labels.append(appliedToBodyName+"_"+getName()+"_Tx");
        labels.append(appliedToBodyName+"_"+getName()+"_Ty");
        labels.append(appliedToBodyName+"_"+getName()+"_Tz");
    }
    return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> ExternalForce::getRecordValues(const SimTK::State& state) const
{
    OpenSim::Array<double>  values(SimTK::NaN);
    double time = state.getTime();

    if (_appliesForce) {
        Vec3 force = getForceAtTime(time);
        force = _forceExpressedInBody->expressVectorInGround(state, force);
        for(int i=0; i<3; ++i)
            values.append(force[i]);
    
        if (_specifiesPoint) {
            Vec3 point = getPointAtTime(time);
            point = _pointExpressedInBody->
                findStationLocationInAnotherFrame(state, point, *_appliedToBody);
            for(int i=0; i<3; ++i)
                values.append(point[i]);
        }
    }
    if (_appliesTorque){
        Vec3 torque = getTorqueAtTime(time);
        torque = _forceExpressedInBody->expressVectorInGround(state, torque);
        for(int i=0; i<3; ++i)
            values.append(torque[i]);
    }

    return values;
};


