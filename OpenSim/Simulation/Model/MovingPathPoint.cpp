/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MovingPathPoint.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
#include "MovingPathPoint.h"
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor.
 */
MovingPathPoint::MovingPathPoint() : Super()
{
    constructProperties();
}

//_____________________________________________________________________________
/*
 * Destructor.
 */
MovingPathPoint::~MovingPathPoint()
{
}

//_____________________________________________________________________________
/*
* Connect properties to local pointers.
*/
void MovingPathPoint::constructProperties()
{
    constructProperty_x_location(Constant(0.0));
    constructProperty_y_location(Constant(0.0));
    constructProperty_z_location(Constant(0.0));
}

bool MovingPathPoint::hasXCoordinate() const
{
    return getSocket<Coordinate>("x_coordinate").isConnected();
}
bool MovingPathPoint::hasYCoordinate() const
{
    return getSocket<Coordinate>("y_coordinate").isConnected();
}
bool MovingPathPoint::hasZCoordinate() const
{
    return getSocket<Coordinate>("z_coordinate").isConnected();
}

const Coordinate& MovingPathPoint::getXCoordinate() const
{
    return _xCoordinate.getRef();
}

const Coordinate& MovingPathPoint::getYCoordinate() const
{
    return _yCoordinate.getRef();
}

const Coordinate& MovingPathPoint::getZCoordinate() const
{
    return _zCoordinate.getRef();
}

void MovingPathPoint::setXCoordinate(const Coordinate& coordinate)
{
    connectSocket_x_coordinate(coordinate);
}
void MovingPathPoint::setYCoordinate(const Coordinate& coordinate)
{
    connectSocket_y_coordinate(coordinate);
}
void MovingPathPoint::setZCoordinate(const Coordinate& coordinate)
{
    connectSocket_z_coordinate(coordinate);
}

void MovingPathPoint::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
    // Hang on to references to the Coordinates instead of
    // finding the socket each time we need a Coordinate value
    if (getSocket<Coordinate>("x_coordinate").isConnected()) {
        _xCoordinate.reset(&getConnectee<Coordinate>("x_coordinate"));
    }
    if (getSocket<Coordinate>("y_coordinate").isConnected()) {
        _yCoordinate.reset(&getConnectee<Coordinate>("y_coordinate"));
    }
    if (getSocket<Coordinate>("z_coordinate").isConnected()) {
        _zCoordinate.reset(&getConnectee<Coordinate>("z_coordinate"));
    }

    // As OpenSim 3.2 we correct for the Work along a Coordinate due to
    // the generalized force that enforces the "gearing" that moves the point
    // under tension. The work is attributed to a single Coordinate, so 
    // we temporarily do not support independent components.
    // TODO: If the coordinates are different then, getdPointdQ() should return a 
    // 3x3 of partials or return a Vec3 w.r.t one coordinate at a time, where the
    // specific Coordinate is an argument. 
    OPENSIM_THROW_IF(!((_xCoordinate == _yCoordinate) && (_xCoordinate == _zCoordinate)),
        Exception,
        "MovingPathPoint:: Components of the path point location "
        "must depend on the same Coordinate. Condition: "
        "x_coordinate == y_coordinate == z_coordinate  FAILED.");
}

//_____________________________________________________________________________
/*
 * Override default implementation by Object to intercept and fix the XML node
 * underneath the MovingPathPoint to match the current version.
 */
void MovingPathPoint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if (documentVersion < 30000) {
        if (Object::getDebugLevel()>=1)
            cout << "Updating MovingPathPoint object to latest format..." << endl;
        XMLDocument::renameChildNode(aNode, "XAttachment", "x_location");
        XMLDocument::renameChildNode(aNode,"YAttachment", "y_location");
        XMLDocument::renameChildNode(aNode,"ZAttachment", "z_location");
    }
    if (documentVersion < 30505) {
        // replace old properties with latest use of Connectors
        SimTK::Xml::element_iterator xCoord = aNode.element_begin("x_coordinate");
        SimTK::Xml::element_iterator yCoord = aNode.element_begin("y_coordinate");
        SimTK::Xml::element_iterator zCoord = aNode.element_begin("z_coordinate");

        std::string xCoord_name(""), yCoord_name(""), zCoord_name("");
        // If default constructed then elements not serialized since they are default
        // values. Check that we have associated elements, then extract their values.
        if (xCoord != aNode.element_end())
            xCoord->getValueAs<std::string>(xCoord_name);
        if (yCoord != aNode.element_end())
            yCoord->getValueAs<std::string>(yCoord_name);
        if (zCoord != aNode.element_end())
            zCoord->getValueAs<std::string>(zCoord_name);
        XMLDocument::addConnector(aNode, "Connector_Coordinate_", 
            "x_coordinate", xCoord_name);
        XMLDocument::addConnector(aNode, "Connector_Coordinate_", 
            "y_coordinate", yCoord_name);
        XMLDocument::addConnector(aNode, "Connector_Coordinate_", 
            "z_coordinate", zCoord_name);
    }

    // Call base class now assuming _node has been corrected for current version
    Super::updateFromXMLNode(aNode, versionNumber);
}

SimTK::Vec3 MovingPathPoint::getLocation(const SimTK::State& s) const
{
    SimTK::Vec3 pInF(0);
    if (!_xCoordinate.empty()) {
        const double xval = SimTK::clamp(_xCoordinate->getRangeMin(),
            _xCoordinate->getValue(s),
            _xCoordinate->getRangeMax());
        pInF[0] = get_x_location().calcValue(SimTK::Vector(1, xval));
    }
    else // assume a Constant
        pInF[0] = get_x_location().calcValue(SimTK::Vector(1, 0.0));

    if (!_yCoordinate.empty()) {
        const double yval = SimTK::clamp(_yCoordinate->getRangeMin(),
            _yCoordinate->getValue(s),
            _yCoordinate->getRangeMax());
        pInF[1] = get_y_location().calcValue(SimTK::Vector(1, yval));
    }
    else // type == Constant
        pInF[1] = get_y_location().calcValue(SimTK::Vector(1, 0.0));

    if (!_zCoordinate.empty()) {
        const double zval = SimTK::clamp(_zCoordinate->getRangeMin(),
            _zCoordinate->getValue(s),
            _zCoordinate->getRangeMax());
        pInF[2] = get_z_location().calcValue(SimTK::Vector(1, zval));
    }
    else // type == Constant
        pInF[2] = get_z_location().calcValue(SimTK::Vector(1, 0.0));

    return pInF;
}


SimTK::Vec3 MovingPathPoint::getVelocity(const SimTK::State& s) const
{
    std::vector<int> derivComponents;
    derivComponents.push_back(0);

    SimTK::Vec3 vInF(0);

    if (!_xCoordinate.empty()){
        //Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
        vInF[0] = get_x_location().calcDerivative(
            derivComponents, SimTK::Vector(1, _xCoordinate->getValue(s)))*
                _xCoordinate->getSpeedValue(s);
    }
    else
        vInF[0] = 0.0;

    if (!_yCoordinate.empty()){
        //Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
        vInF[1] = get_y_location().calcDerivative(
            derivComponents, SimTK::Vector(1, _yCoordinate->getValue(s)))*
                _yCoordinate->getSpeedValue(s);
    }
    else
        vInF[1] = 0.0;

    if (!_zCoordinate.empty()){
        //Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
        vInF[2] = get_z_location().calcDerivative(
            derivComponents, SimTK::Vector(1, _zCoordinate->getValue(s)))*
                _zCoordinate->getSpeedValue(s);
    }
    else
        vInF[2] = 0.0;

    return vInF;
}

//_____________________________________________________________________________
/*
 * Get the velocity of the point in the body's local reference frame.
 */
SimTK::Vec3 MovingPathPoint::getdPointdQ(const SimTK::State& s) const
{
    SimTK::Vec3 dPdq_B(0);

    std::vector<int> derivComponents;
    derivComponents.push_back(0);

    if (!_xCoordinate.empty()){
        //Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
        dPdq_B[0] = get_x_location().calcDerivative(derivComponents, 
            SimTK::Vector(1, _xCoordinate->getValue(s)));
    }
    if (!_yCoordinate.empty()){
        //Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
        dPdq_B[1] = get_y_location().calcDerivative(derivComponents,
            SimTK::Vector(1, _yCoordinate->getValue(s)));
    }
    if (!_zCoordinate.empty()){
        //Multiply the partial (derivative of point coordinate w.r.t. gencoord) by genspeed
        dPdq_B[2] = get_z_location().calcDerivative(derivComponents,
            SimTK::Vector(1, _zCoordinate->getValue(s)));
    }

    return dPdq_B;
}


void MovingPathPoint::scale(const SimTK::Vec3& aScaleFactors)
{
    if (!_xCoordinate.empty()) {
        // If the function is already a MultiplierFunction, just update its scale factor.
        // Otherwise, make a MultiplierFunction from it and make the muscle point use
        // the new MultiplierFunction.
        MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(&upd_x_location());
        if (mf) {
            mf->setScale(mf->getScale() * aScaleFactors[0]);
        } else {
            // Make a copy of the original function and delete the original
            // (so its node will be removed from the XML document).
            set_x_location(MultiplierFunction(get_x_location().clone(), aScaleFactors[0]));
        }
    }

    if (!_yCoordinate.empty()) {
        // If the function is already a MultiplierFunction, just update its scale factor.
        // Otherwise, make a MultiplierFunction from it and make the muscle point use
        // the new MultiplierFunction.
        MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(&upd_y_location());
        if (mf) {
            mf->setScale(mf->getScale() * aScaleFactors[1]);
        } else {
            // Make a copy of the original function and delete the original
            // (so its node will be removed from the XML document).
            set_y_location(MultiplierFunction(get_y_location().clone(), aScaleFactors[1]));
        }
    }

    if (!_zCoordinate.empty()) {
        // If the function is already a MultiplierFunction, just update its scale factor.
        // Otherwise, make a MultiplierFunction from it and make the muscle point use
        // the new MultiplierFunction.
        MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(&upd_z_location());
        if (mf) {
            mf->setScale(mf->getScale() * aScaleFactors[2]);
        } else {
            set_z_location(MultiplierFunction(get_z_location().clone(), aScaleFactors[2]));
        }
    }
}


SimTK::Vec3 MovingPathPoint::calcLocationInGround(const SimTK::State& s) const
{
    return getParentFrame().getTransformInGround(s)*getLocation(s);
}

SimTK::Vec3 MovingPathPoint::calcVelocityInGround(const SimTK::State& s) const
{
    // compute the local position vector, r, of the moving point in its
    // parent reference frame, F, expressed in Ground, G.
    const auto& R_GF = getParentFrame().getTransformInGround(s).R();
    const Vec3 r = R_GF*getLocation(s);
    
    // express the local velocity of the moving point in Ground
    const Vec3 v = R_GF*getVelocity(s);

    // get the velocity of the parent frame, F,  in Ground, G.
    const SimTK::SpatialVec& V_GF = getParentFrame().getVelocityInGround(s);
    
    // The velocity of the moving point in Ground is a combination of its parent
    // frame's linear (v_GF = V_GF[1]) and angular (omega_GF = V_GF[0]) velocity,
    // and its local velocity in F, such that: 
    // v_GP = v_GF + omega_GF x r_FP_G + v_FP_G
    return V_GF[1] + V_GF[0] % r + v;
}

SimTK::Vec3 MovingPathPoint::calcAccelerationInGround(const SimTK::State& state) const
{
    //TODO: Enable Exception or Implement the method and add accompanying test.
    //      Exception is disabled because testComponents automatically verifies that
    //      Outputs are functional. My preference is to implement the method
    //OPENSIM_THROW(Exception, "MovingPathPoint::calcAccelerationInGround not implemented.");
    std::cerr << "MovingPathPoint::calcAccelerationInGround() not implemented. "
        << "It returns NaN" << std::endl;
    return Vec3(SimTK::NaN);
}
