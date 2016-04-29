/* -------------------------------------------------------------------------- *
 *                       OpenSim:  PrescribedForce.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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



//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using SimTK::Vec3;
using namespace std;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
// default destructor, copy constructor, copy assignment
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PrescribedForce::PrescribedForce(Body* body)
{
    setNull();
    constructProperties();

    _body = body;
    if (_body)
        setBodyName(_body->getName());
}

//_____________________________________________________________________________
/**
 * Constructor from XML file
 */
PrescribedForce::PrescribedForce(SimTK::Xml::Element& aNode) : Super(aNode)
{
    setNull();
    constructProperties();
    updateFromXMLNode(aNode);
}


//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 */
void PrescribedForce::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    // Base class
    Super::updateFromXMLNode(aNode, versionNumber);

    const FunctionSet& forceFunctions  = getForceFunctions();
    const FunctionSet& pointFunctions  = getPointFunctions();
    const FunctionSet& torqueFunctions = getTorqueFunctions();

    //Specify all or none of the components
    if(forceFunctions.getSize() != 3 && forceFunctions.getSize() != 0)
    {
        throw Exception("PrescribedForce:: three components of the force must be specified.");
    }

    if(pointFunctions.getSize() != 3 && pointFunctions.getSize() != 0)
    {
        throw Exception("PrescribedForce:: three components of the point must be specified.");
    }

    if(torqueFunctions.getSize() != 3 && torqueFunctions.getSize() != 0)
    {
        throw Exception("PrescribedForce:: three components of the torque must be specified.");
    }
}   


/*
 * Construct and initialize properties.
 */
void PrescribedForce::constructProperties()
{
    constructProperty_body();
    constructProperty_pointIsGlobal(false);
    constructProperty_forceIsGlobal(true);
    constructProperty_forceFunctions(FunctionSet());
    constructProperty_pointFunctions(FunctionSet());
    constructProperty_torqueFunctions(FunctionSet());
}

void PrescribedForce::setForceFunctions(Function* forceX, Function* forceY, Function* forceZ)
{
    FunctionSet& forceFunctions = updForceFunctions();

    forceFunctions.setSize(0);
    forceFunctions.cloneAndAppend(*forceX);
    forceFunctions.cloneAndAppend(*forceY);
    forceFunctions.cloneAndAppend(*forceZ);
}


void PrescribedForce::setPointFunctions(Function* pointX, Function* pointY, Function* pointZ)
{
    FunctionSet& pointFunctions = updPointFunctions();

    pointFunctions.setSize(0);
    pointFunctions.cloneAndAppend(*pointX);
    pointFunctions.cloneAndAppend(*pointY);
    pointFunctions.cloneAndAppend(*pointZ);
}

void PrescribedForce::setTorqueFunctions(Function* torqueX, Function* torqueY, Function* torqueZ)
{
    FunctionSet& torqueFunctions = updTorqueFunctions();

    torqueFunctions.setSize(0);
    torqueFunctions.cloneAndAppend(*torqueX);
    torqueFunctions.cloneAndAppend(*torqueY);
    torqueFunctions.cloneAndAppend(*torqueZ);

}

void PrescribedForce::setTorqueFunctionNames
   (const OpenSim::Array<std::string>& aFunctionNames, 
    const Storage& kineticsStore)  
{
    FunctionSet& torqueFunctions = updTorqueFunctions();

    int forceSize = kineticsStore.getSize();
    if(forceSize<=0) return;
    double *t=0;
    // Expected column labels for the file
    kineticsStore.getTimeColumn(t);
    double *column=0;
    SimmSpline** tSpline = new SimmSpline*[3];
    for(int i=0;i<aFunctionNames.getSize();i++)
    {
        kineticsStore.getDataColumn(aFunctionNames[i], column);
        tSpline[i]= new SimmSpline((forceSize>10?10:forceSize), t, column, aFunctionNames[i]);
    }
    setTorqueFunctions(tSpline[0], tSpline[1], tSpline[2]);
    for (int i=0; i<aFunctionNames.getSize();i++)
        torqueFunctions[i].setName(aFunctionNames.get(i));
}
void PrescribedForce::setForceFunctionNames
   (const OpenSim::Array<std::string>& aFunctionNames, 
    const Storage& kineticsStore)  
{
    FunctionSet& forceFunctions = updForceFunctions();

    int forceSize = kineticsStore.getSize();
    if(forceSize<=0) return;
    double *t=0;
    // Expected column labels for the file
    kineticsStore.getTimeColumn(t);
    double *column=0;
    SimmSpline** tSpline = new SimmSpline*[3];
    for(int i=0;i<aFunctionNames.getSize();i++)
    {
        kineticsStore.getDataColumn(aFunctionNames[i], column);
        tSpline[i]= new SimmSpline((forceSize>10?10:forceSize), t, column, aFunctionNames[i]);
    }
    setForceFunctions(tSpline[0], tSpline[1], tSpline[2]);
    for (int i=0; i<aFunctionNames.getSize();i++)
        forceFunctions[i].setName(aFunctionNames.get(i));
}
void PrescribedForce::setPointFunctionNames
   (const OpenSim::Array<std::string>& aFunctionNames, 
    const Storage& kineticsStore)  
{
    FunctionSet& pointFunctions = updPointFunctions();

    int forceSize = kineticsStore.getSize();
    if(forceSize<=0) return;
    double *t=0;
    // Expected column labels for the file
    kineticsStore.getTimeColumn(t);
    double *column=0;
    SimmSpline** tSpline = new SimmSpline*[3];
    for(int i=0;i<aFunctionNames.getSize();i++)
    {
        kineticsStore.getDataColumn(aFunctionNames[i], column);
        tSpline[i]= new SimmSpline((forceSize>10?10:forceSize), 
                                           t, column, aFunctionNames[i]);
    }
    setPointFunctions(tSpline[0], tSpline[1], tSpline[2]);
    for (int i=0; i<aFunctionNames.getSize();i++)
        pointFunctions[i].setName(aFunctionNames.get(i));
}


//-----------------------------------------------------------------------------
// ABSTRACT METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void PrescribedForce::computeForce(const SimTK::State& state, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    const bool pointIsGlobal = get_pointIsGlobal();
    const bool forceIsGlobal = get_forceIsGlobal();
    const FunctionSet& forceFunctions = getForceFunctions();
    const FunctionSet& pointFunctions = getPointFunctions();
    const FunctionSet& torqueFunctions = getTorqueFunctions();

    double time = state.getTime();
    const SimbodyEngine& engine = getModel().getSimbodyEngine();
    SimTK::Vector  timeAsVector(1, time);

    const bool hasForceFunctions  = forceFunctions.getSize()==3;
    const bool hasPointFunctions  = pointFunctions.getSize()==3;
    const bool hasTorqueFunctions = torqueFunctions.getSize()==3;

    assert(_body!=0);
    if (hasForceFunctions) {
        Vec3 force(forceFunctions[0].calcValue(timeAsVector), 
                   forceFunctions[1].calcValue(timeAsVector), 
                   forceFunctions[2].calcValue(timeAsVector));
        if (!forceIsGlobal)
            engine.transform(state, *_body,                 force, 
                                    getModel().getGround(), force);
        Vec3 point(0); // Default is body origin.
        if (hasPointFunctions) {
            // Apply force to a specified point on the body.
            point = Vec3(pointFunctions[0].calcValue(timeAsVector), 
                         pointFunctions[1].calcValue(timeAsVector), 
                         pointFunctions[2].calcValue(timeAsVector));
            if (pointIsGlobal)
                engine.transformPosition(state, getModel().getGround(), point,
                                                *_body,                 point);
        }
        applyForceToPoint(state, *_body, point, force, bodyForces);
    }
    if (hasTorqueFunctions){
        Vec3 torque(torqueFunctions[0].calcValue(timeAsVector), 
                    torqueFunctions[1].calcValue(timeAsVector), 
                    torqueFunctions[2].calcValue(timeAsVector));
        if (!forceIsGlobal)
            engine.transform(state, *_body,                 torque, 
                                    getModel().getGround(), torque);
        applyTorque(state, *_body, torque, bodyForces);
    }
}

/**
 * Convenience methods to access prescribed force functions
 */
Vec3 PrescribedForce::getForceAtTime(double aTime) const    
{
    const FunctionSet& forceFunctions = getForceFunctions();

    if (forceFunctions.getSize() != 3)
        return Vec3(0);

    const SimTK::Vector timeAsVector(1, aTime);
    const Vec3 force(forceFunctions[0].calcValue(timeAsVector), 
                     forceFunctions[1].calcValue(timeAsVector), 
                     forceFunctions[2].calcValue(timeAsVector));
    return force;
}

Vec3 PrescribedForce::getPointAtTime(double aTime) const
{
    const FunctionSet& pointFunctions = getPointFunctions();

    if (pointFunctions.getSize() != 3)
        return Vec3(0);

    const SimTK::Vector timeAsVector(1, aTime);
    const Vec3 point(pointFunctions[0].calcValue(timeAsVector), 
                     pointFunctions[1].calcValue(timeAsVector), 
                     pointFunctions[2].calcValue(timeAsVector));
    return point;
}

Vec3 PrescribedForce::getTorqueAtTime(double aTime) const
{
    const FunctionSet& torqueFunctions = getTorqueFunctions();

    if (torqueFunctions.getSize() != 3)
        return Vec3(0);

    const SimTK::Vector timeAsVector(1, aTime);
    const Vec3 torque(torqueFunctions[0].calcValue(timeAsVector), 
                      torqueFunctions[1].calcValue(timeAsVector), 
                      torqueFunctions[2].calcValue(timeAsVector));
    return torque;
}


//-----------------------------------------------------------------------------
// Reporting
//-----------------------------------------------------------------------------

OpenSim::Array<std::string> PrescribedForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");

    const bool forceIsGlobal = get_forceIsGlobal();

    const FunctionSet& forceFunctions = getForceFunctions();
    const FunctionSet& pointFunctions = getPointFunctions();
    const FunctionSet& torqueFunctions = getTorqueFunctions();

    const bool appliesForce   = forceFunctions.getSize()==3;
    const bool pointSpecified = pointFunctions.getSize()==3;
    const bool appliesTorque  = torqueFunctions.getSize()==3;

    std::string BodyToReport = (forceIsGlobal?"ground":_body->getName());
    if (appliesForce) {
        labels.append(BodyToReport+"_"+getName()+"_fx");
        labels.append(BodyToReport+"_"+getName()+"_fy");
        labels.append(BodyToReport+"_"+getName()+"_fz");
    }
    if (pointSpecified) {
        labels.append(BodyToReport+"_"+getName()+"_px");
        labels.append(BodyToReport+"_"+getName()+"_py");
        labels.append(BodyToReport+"_"+getName()+"_pz");
    }
    if (appliesTorque) {
        labels.append(BodyToReport+"_"+getName()+"_torque_x");
        labels.append(BodyToReport+"_"+getName()+"_torque_y");
        labels.append(BodyToReport+"_"+getName()+"_torque_z");
    }
    return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> PrescribedForce::getRecordValues(const SimTK::State& state) const {
    OpenSim::Array<double>  values(SimTK::NaN);
    assert(_body!=0);

    const bool pointIsGlobal = get_pointIsGlobal();
    const bool forceIsGlobal = get_forceIsGlobal();

    const FunctionSet& forceFunctions = getForceFunctions();
    const FunctionSet& pointFunctions = getPointFunctions();
    const FunctionSet& torqueFunctions = getTorqueFunctions();

    const bool appliesForce   = forceFunctions.getSize()==3;
    const bool pointSpecified = pointFunctions.getSize()==3;
    const bool appliesTorque  = torqueFunctions.getSize()==3;

    // This is bad as it duplicates the code in computeForce we'll cleanup after it works!
    const double time = state.getTime();
    const SimbodyEngine& engine = getModel().getSimbodyEngine();
    const SimTK::Vector timeAsVector(1, time);

    if (appliesForce) {
        Vec3 force(forceFunctions[0].calcValue(timeAsVector), 
                   forceFunctions[1].calcValue(timeAsVector), 
                   forceFunctions[2].calcValue(timeAsVector));
        if (!forceIsGlobal)
            engine.transform(state, *_body, force, 
                             getModel().getGround(), force);
        if (!pointSpecified) {
            //applyForce(*_body, force);
            for (int i=0; i<3; i++) values.append(force[i]);
        } else {
            Vec3 point(pointFunctions[0].calcValue(timeAsVector), 
                       pointFunctions[1].calcValue(timeAsVector), 
                       pointFunctions[2].calcValue(timeAsVector));
            if (pointIsGlobal)
                engine.transformPosition(state, getModel().getGround(), point, 
                                         *_body, point);
            //applyForceToPoint(*_body, point, force);
            for (int i=0; i<3; i++) values.append(force[i]);
            for (int i=0; i<3; i++) values.append(point[i]);
        }
    }
    if (appliesTorque) {
        Vec3 torque(torqueFunctions[0].calcValue(timeAsVector), 
                    torqueFunctions[1].calcValue(timeAsVector), 
                    torqueFunctions[2].calcValue(timeAsVector));
        if (!forceIsGlobal)
            engine.transform(state, *_body, torque, 
                             getModel().getGround(), torque);
        for (int i=0; i<3; i++) values.append(torque[i]);
        //applyTorque(*_body, torque);
    }
    return values;
};

void PrescribedForce::setNull()
{
    setAuthors("Peter Eastman, Matt DeMers");
    _body = NULL;
}

void PrescribedForce::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    // hook up body pointer to name
    if (_model)
        _body = &_model->updBodySet().get(getBodyName());
}
