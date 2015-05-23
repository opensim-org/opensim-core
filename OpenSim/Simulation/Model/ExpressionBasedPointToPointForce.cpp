/* -------------------------------------------------------------------------- *
 *               OpenSim:  ExpressionBasedPointToPointForce.cpp               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "ExpressionBasedPointToPointForce.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

//_____________________________________________________________________________
//Default constructor.
ExpressionBasedPointToPointForce::ExpressionBasedPointToPointForce()
{
    setNull();
    constructProperties();
}
//_____________________________________________________________________________
// Convenience constructor for API users.
ExpressionBasedPointToPointForce::ExpressionBasedPointToPointForce(
                const string& body1Name, const SimTK::Vec3& point1, 
                const string& body2Name, const SimTK::Vec3& point2, 
                const string& expression)
{
    setNull();
    constructProperties();

    // Set properties to the passed-in values.
    setBody1Name(body1Name);
    setBody2Name(body2Name);

    setPoint1(point1);
    setPoint2(point2);

    setExpression(expression);
}

// Set the expression for the force function and create it's lepton program 
void ExpressionBasedPointToPointForce::setExpression(const string& expression) 
{
    set_expression(expression);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this force to their null values.
 */
void ExpressionBasedPointToPointForce::setNull()
{
    setAuthors("Ajay Seth"); 
}


//_____________________________________________________________________________
/**
 * Construct properties and initialize to their default values.
 */
void ExpressionBasedPointToPointForce::constructProperties()
{
    constructProperty_body1();
    constructProperty_body2();

    const SimTK::Vec3 bodyOrigin(0.0, 0.0, 0.0);
    constructProperty_point1(bodyOrigin);
    constructProperty_point2(bodyOrigin);

    std::string zero = "0.0";
    constructProperty_expression( zero );
}

//_____________________________________________________________________________
/**
 * Get the visible object used to represent the spring.
 */
VisibleObject* ExpressionBasedPointToPointForce::getDisplayer() const
{ 
    return const_cast<VisibleObject*>(&_displayer); 
}

void ExpressionBasedPointToPointForce::updateDisplayer(const SimTK::State& s)
{
    SimTK::Vec3 globalLocation1, globalLocation2;
    const OpenSim::Body& body1 = _model->getBodySet().get(getBody1Name());
    const OpenSim::Body& body2 = _model->getBodySet().get(getBody2Name());
    _model->getSimbodyEngine().transformPosition(s, body1, getPoint1(), 
                                                 globalLocation1);
    _model->getSimbodyEngine().transformPosition(s, body2, getPoint2(),
                                                 globalLocation2);

    if (_displayer.countGeometry()==0){
        Geometry *g = new LineGeometry();
        g->setFixed(false);
        _displayer.addGeometry(g);
    }
    ((LineGeometry *)_displayer.getGeometry(0))->
        setPoints(globalLocation1, globalLocation2);
}

void ExpressionBasedPointToPointForce::updateGeometry(const SimTK::State& s)
{
    if (_displayer.countGeometry()==0){
        Geometry *g = new LineGeometry();
        g->setFixed(false);
        _displayer.addGeometry(g);
    }
    updateDisplayer(s);
}

//=============================================================================
// Connect this force element to the rest of the model.
//=============================================================================
void ExpressionBasedPointToPointForce::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model); // Let base class connect first.

    // Look up the two bodies being connected by bushing by name in the
    // model. TODO: use Connectors
    const string& body1Name = getBody1Name();
    const string& body2Name = getBody2Name();

    _body1 =
        static_cast<const PhysicalFrame*>(&getModel().getComponent(body1Name));
    _body2 =
        static_cast<const PhysicalFrame*>(&getModel().getComponent(body2Name));

    if(getName() == "")
        setName("expressionP2PForce_"+body1Name+"To"+body2Name);

    string& expression = upd_expression();
    expression.erase(
            remove_if(expression.begin(), expression.end(), ::isspace), 
                      expression.end() );
    
    _forceProg = Lepton::Parser::parse(expression).optimize().createProgram();
}

//=============================================================================
// Create the underlying system component(s)
//=============================================================================
void ExpressionBasedPointToPointForce::
extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);    // Base class first.

    addCacheVariable<double>("force_magnitude", 0.0, SimTK::Stage::Velocity);

    // Beyond the const Component get access to underlying SimTK elements
    ExpressionBasedPointToPointForce* mutableThis =
        const_cast<ExpressionBasedPointToPointForce *>(this);

    // Get underlying mobilized bodies
    mutableThis->_b1 = _body1->getMobilizedBody();
    mutableThis->_b2 = _body2->getMobilizedBody();
}

//=============================================================================
// Computing
//=============================================================================
// Compute and apply the force
void ExpressionBasedPointToPointForce::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    using namespace SimTK;

    const Transform& X_GB1 = _b1->getBodyTransform(s);
    const Transform& X_GB2 = _b2->getBodyTransform(s);

    const Vec3 s1_G = X_GB1.R() * getPoint1();
    const Vec3 s2_G = X_GB2.R() * getPoint2();

    const Vec3 p1_G = X_GB1.p() + s1_G; // point measured from ground origin
    const Vec3 p2_G = X_GB2.p() + s2_G;
    const Vec3 r_G = p2_G - p1_G; // vector from point1 to point2
    const double d = r_G.norm();  // distance between the points

    const Vec3 v1_G = _b1->findStationVelocityInGround(s, getPoint1());
    const Vec3 v2_G = _b2->findStationVelocityInGround(s, getPoint2());
    const Vec3 vRel = v2_G - v1_G; // relative velocity

    //speed along the line connecting the two bodies
    const double ddot = dot(vRel, r_G)/d;

    std::map<std::string, double> forceVars;
    forceVars["d"] = d;
    forceVars["ddot"] = ddot;

    double forceMag = _forceProg.evaluate(forceVars);
    setCacheVariableValue<double>(s, "force_magnitude", forceMag);

    const Vec3 f1_G = (forceMag/d) * r_G;

    bodyForces[_b1->getMobilizedBodyIndex()] +=  SpatialVec(s1_G % f1_G, f1_G);
    bodyForces[_b2->getMobilizedBodyIndex()] -=  SpatialVec(s2_G % f1_G, f1_G);
}

// get the force magnitude that has already been computed
const double& ExpressionBasedPointToPointForce::
    getForceMagnitude(const SimTK::State& s)
{
    return getCacheVariableValue<double>(s, "force_magnitude");
}


//=============================================================================
// Reporting
//=============================================================================
// Provide names of the quantities (column labels) of the force value(s) 
// reported.
OpenSim::Array<std::string> ExpressionBasedPointToPointForce::getRecordLabels() const 
{
    const string& body1Name = getBody1Name();
    const string& body2Name = getBody2Name();

    OpenSim::Array<std::string> labels("");
    labels.append(getName()+"."+body1Name+".force.X");
    labels.append(getName()+"."+body1Name+".force.Y");
    labels.append(getName()+"."+body1Name+".force.Z");
    labels.append(getName()+"."+body1Name+".point.X");
    labels.append(getName()+"."+body1Name+".point.Y");
    labels.append(getName()+"."+body1Name+".point.Z");
    labels.append(getName()+"."+body2Name+".force.X");
    labels.append(getName()+"."+body2Name+".force.Y");
    labels.append(getName()+"."+body2Name+".force.Z");
    labels.append(getName()+"."+body2Name+".point.X");
    labels.append(getName()+"."+body2Name+".point.Y");
    labels.append(getName()+"."+body2Name+".point.Z");

    return labels;
}

// Provide the value(s) to be reported that correspond to the labels.
OpenSim::Array<double> ExpressionBasedPointToPointForce::
getRecordValues(const SimTK::State& state) const 
{
    OpenSim::Array<double> values(1);

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);


    //get the net force added to the system contributed by the Spring
    _model->getForceSubsystem().getForce(_index)
        .calcForceContribution(state, bodyForces, particleForces, mobilityForces);
    
    SimTK::Vec3 forces = bodyForces(_body1->getMobilizedBodyIndex())[1];
    values.append(3, &forces[0]);

    SimTK::Vec3 gpoint(0);
    _model->getSimbodyEngine().getPosition(state, *_body1, getPoint1(), gpoint);
    values.append(3, &gpoint[0]);

    forces = bodyForces(_body2->getMobilizedBodyIndex())[1];
    values.append(3, &forces[0]);

    _model->getSimbodyEngine().getPosition(state, *_body2, getPoint2(), gpoint);
    values.append(3, &gpoint[0]);

    return values;
}

