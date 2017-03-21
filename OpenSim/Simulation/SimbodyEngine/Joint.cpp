/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Joint.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Frank C. Anderson                                    *
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
#include "Joint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Common/ScaleSet.h>
#include "simbody/internal/Constraint.h"
#include "simbody/internal/MobilizedBody_Ground.h"

//==============================================================================
// STATICS
//==============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//==============================================================================
// CONSTRUCTORS AND DESTRUCTOR
//==============================================================================
Joint::~Joint()
{
}

Joint::Joint() : Super()
{
    setNull();
    constructProperties();
}

Joint::Joint(const std::string&    name,
             const PhysicalFrame&  parent,
             const PhysicalFrame&  child) : Joint()
{
    OPENSIM_THROW_IF(name.empty(), ComponentHasNoName, getClassName());

    setName(name);
    connectSocket_parent_frame(parent);
    connectSocket_child_frame(child);
}

Joint::Joint(const std::string&    name,
             const PhysicalFrame&  parent,
             const SimTK::Vec3&    locationInParent,
             const SimTK::Vec3&    orientationInParent,
             const PhysicalFrame&  child,
             const SimTK::Vec3&    locationInChild,
             const SimTK::Vec3&    orientationInChild) : Joint()
{
    OPENSIM_THROW_IF(name.empty(), ComponentHasNoName, getClassName());

    setName(name);

    // PARENT TRANSFORM
    Rotation parentRotation(BodyRotationSequence,
        orientationInParent[0], XAxis,
        orientationInParent[1], YAxis,
        orientationInParent[2], ZAxis);
    SimTK::Transform parentTransform(parentRotation, locationInParent);

    // Define the offset frame that the joint connects to in the parent
    PhysicalOffsetFrame pInPo( parent.getName() + "_offset",
                               parent,
                               parentTransform);
    
    // Append the offset frame to the Joint's internal list of frames
    int pix = append_frames(pInPo);

    // CHILD TRANSFORM
    Rotation childRotation(BodyRotationSequence,
        orientationInChild[0], XAxis,
        orientationInChild[1], YAxis,
        orientationInChild[2], ZAxis);
    SimTK::Transform childTransform(childRotation, locationInChild);

    PhysicalOffsetFrame cInCo( child.getName() + "_offset",
                               child,
                               childTransform);

    // Append the child offset frame to the Joint's internal list of frames
    int cix = append_frames(cInCo);

    // finalize recognizes the offset frames as the Joint's subcomponents
    finalizeFromProperties();

    // When the PhysicalOffsetFrames are constructed they are unaware that this
    // Joint contains them as subcomponents and the path name associated with 
    // them will not be valid. This a temporary fix to set the name once the
    // added frames have been included as subcomponents which occurs during
    // finalizeFromProperties() above.
    static_cast<PhysicalOffsetFrame&>(upd_frames(pix)).setParentFrame(parent);
    static_cast<PhysicalOffsetFrame&>(upd_frames(cix)).setParentFrame(child);

    connectSocket_parent_frame(upd_frames(pix));
    connectSocket_child_frame(upd_frames(cix));
}

//=============================================================================
// CONSTRUCTION Utility
//=============================================================================
Joint::CoordinateIndex Joint::constructCoordinate(Coordinate::MotionType mt,
                                                  unsigned idx)
{
    Coordinate* coord = new Coordinate();
    coord->setName(getName() + "_coord_" + std::to_string( numCoordinates() ));
    // Joint takes ownership
    coord->setJoint(*this);
    updProperty_coordinates().adoptAndAppendValue(coord);
    auto cix = CoordinateIndex(getProperty_coordinates().
                               findIndexForName( coord->getName() ));
    _motionTypes.push_back(mt);
    SimTK_ASSERT_ALWAYS(static_cast<unsigned>(numCoordinates()) == 
                        _motionTypes.size(), 
                        "Joint::constructCoordinate() MotionTypes do not "
                        "correspond to coordinates");
    SimTK_ASSERT_ALWAYS(static_cast<unsigned>(cix) == idx,
                        "Joint::constructCoordinate() must be passed "
                        "enumerations in the same order as the enumerations "
                        "have been defined");
    return cix;
}

//_____________________________________________________________________________
/**
 * Set the data members of this Joint to their null values.
 */
void Joint::setNull()
{
    setAuthors("Ajay Seth");
    isReversed = false;
}

//_____________________________________________________________________________
/**
 * Construct properties and initialize their default values.
 */
void Joint::constructProperties()
{
    // Generalized coordinates
    constructProperty_coordinates();

    //Default frames list is empty
    constructProperty_frames();
}

void Joint::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    // add all coordinates listed under this joint
    for (int i = 0; i < numCoordinates(); ++i)
        upd_coordinates(i).setJoint(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// CHILD Frame
//-----------------------------------------------------------------------------
const PhysicalFrame& Joint::getChildFrame() const
{
    return getSocket<PhysicalFrame>("child_frame").getConnectee();
}

//-----------------------------------------------------------------------------
// PARENT BODY
//-----------------------------------------------------------------------------
const OpenSim::PhysicalFrame& Joint::getParentFrame() const
{
    return getSocket<PhysicalFrame>("parent_frame").getConnectee();
}

const Coordinate& Joint::getCoordinate() const {
    OPENSIM_THROW_IF(numCoordinates() == 0,
                     JointHasNoCoordinates);
    OPENSIM_THROW_IF(numCoordinates() > 1,
                     InvalidCall,
                     "Joint has more than one coordinate. Use get_coordinates() "
                     "or the getCoordinate() method defined in the concrete "
                     "class instead.");

    return get_coordinates(0);
}

Coordinate& Joint::updCoordinate() {
    OPENSIM_THROW_IF(numCoordinates() == 0,
                     JointHasNoCoordinates);
    OPENSIM_THROW_IF(numCoordinates() > 1,
                     InvalidCall,
                     "Joint has more than one coordinate. Use upd_coordinates() "
                     "or the updCoordinate() method defined in the concrete "
                     "class instead.");

    return upd_coordinates(0);
}

Coordinate::MotionType Joint::getMotionType(CoordinateIndex cix) const
{
    OPENSIM_THROW_IF(cix >= _motionTypes.size(), Exception,
        "Joint::getMotionType() given an invalid CoordinateIndex");
    return _motionTypes[cix];
}

void Joint::setMotionType(CoordinateIndex cix, Coordinate::MotionType mt)
{
    int nc = numCoordinates();

    // Ensure that coordinate index is less than the number of coordinates
    // this Joint has in its CoordinateSet.
    OPENSIM_THROW_IF(cix >= nc, Exception,
        "Joint::setMotionType() for an invalid CoordinateIndex");
    // Grow the size of _motionTypes (array) if it is less than the number of
    // coordinates. Joint's _motionTypes must correspond to its CoordinateSet.
    if (_motionTypes.size() < static_cast<unsigned>(nc))
        _motionTypes.resize(nc);

    _motionTypes[cix] = mt;
}


//_____________________________________________________________________________
/**
 * Check if a coordinate is used by the Joint.
 *
 * @param aCoordinate Coordinate to look for in joint.
 * @return True if the coordinate is used.
 */
bool Joint::isCoordinateUsed(const Coordinate& aCoordinate) const
{
    for(int i = 0; i < numCoordinates(); ++i) {
        if(&get_coordinates(i) == &aCoordinate) return true;
    }

    return false;
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________

void Joint::scale(const ScaleSet& scaleSet)
{
    SimTK::Vec3 parentFactors(1.0);
    SimTK::Vec3 childFactors(1.0);

    // Find the factors associated with the PhysicalFrames this Joint connects
    const string& parentName = getParentFrame().findBaseFrame().getName();
    const string& childName = getChildFrame().findBaseFrame().getName();
    // Get scale factors
    bool found_p = false;
    bool found_b = false;
    for (int i = 0; i < scaleSet.getSize(); i++) {
        Scale& scale = scaleSet.get(i);
        if (!found_p && (scale.getSegmentName() == parentName)) {
            scale.getScaleFactors(parentFactors);
            found_p = true;
        }
        if (!found_b && (scale.getSegmentName() == childName)) {
            scale.getScaleFactors(childFactors);
            found_b = true;
        }
        if (found_p && found_b)
            break;
    }

    // if the frame is owned by this Joint scale it,
    // otherwise let the owner of the frame decide.
    int found = getProperty_frames().findIndex(getParentFrame());
    if (found >= 0) {
        PhysicalOffsetFrame& offset
            = SimTK_DYNAMIC_CAST_DEBUG<PhysicalOffsetFrame&>(upd_frames(found));
        offset.scale(parentFactors);
    }
    found = getProperty_frames().findIndex(getChildFrame());
    if (found >= 0) {
        PhysicalOffsetFrame& offset
            = SimTK_DYNAMIC_CAST_DEBUG<PhysicalOffsetFrame&>(upd_frames(found));
        offset.scale(childFactors);
    }
}

const SimTK::MobilizedBodyIndex Joint::
    getMobilizedBodyIndex(const OpenSim::Body& body) const
{
        return body.getMobilizedBodyIndex();
} 

void Joint::setChildMobilizedBodyIndex(const SimTK::MobilizedBodyIndex index) const
{ 
    getChildFrame().setMobilizedBodyIndex(index);
}


void Joint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // The parent node in the multibody tree must part of the system
    if(isReversed)
        // this will be the child if the joint definition is reversed
        getSocket<PhysicalFrame>("child_frame").getConnectee().addToSystem(system);
    else // otherwise it is the parent frame
        getSocket<PhysicalFrame>("parent_frame").getConnectee().addToSystem(system);
}

void Joint::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    for (int i = 0; i < numCoordinates(); ++i)
        get_coordinates(i).extendInitStateFromProperties(s);
}

void Joint::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);

    for (int i = 0; i < numCoordinates(); ++i)
        upd_coordinates(i).extendSetPropertiesFromState(state);
}


//=============================================================================
// Computation
//=============================================================================
/* Calculate the equivalent spatial force, FB_G, acting on the body connected by
   this joint at its location B, expressed in ground.  */
SimTK::SpatialVec Joint::calcEquivalentSpatialForce(const SimTK::State &s, 
    const SimTK::Vector &mobilityForces) const
{
    // The number of mobilities for the entire system.
    int nm = _model->getMatterSubsystem().getNumMobilities();

    if(nm != mobilityForces.size()){
        throw Exception("Joint::calcEquivalentSpatialForce(): input mobilityForces does not match model's mobilities");
    }

    const SimTK::MobilizedBodyIndex &mbx = getChildFrame().getMobilizedBodyIndex();
    // build a unique list of underlying MobilizedBodies that are involved
    // with this Joint in addition to and not including that of the child body

    std::set<SimTK::MobilizedBodyIndex> mbds;

    for(int i = 0; i < numCoordinates(); ++i) {
        const MobilizedBodyIndex& coordsMbx = get_coordinates(i).getBodyIndex();
        if (coordsMbx != mbx){
            mbds.insert(coordsMbx);
        }
    }
    
    SimTK::SpatialVec FB_G = calcEquivalentSpatialForceForMobilizedBody(s, mbx, mobilityForces);
    SimTK::SpatialVec FBx_G;

    std::set<SimTK::MobilizedBodyIndex>::const_iterator it = mbds.begin();

    //const SimTK::MobilizedBody &G = getModel().getMatterSubsystem().getGround();
    //const SimTK::MobilizedBody &B = getModel().getMatterSubsystem().getMobilizedBody(mbx);
    //SimTK::Vec3 r_BG =
    //    B.expressVectorInAnotherBodyFrame(s, B.getOutboardFrame(s).p(), G);

    while(it != mbds.end()){
        FBx_G = calcEquivalentSpatialForceForMobilizedBody(s, *it, mobilityForces);

        //const SimTK::MobilizedBody &b = 
        //   getModel().getMatterSubsystem().getMobilizedBody(*it);

        
        //SimTK::Vec3 r_bG = 
        //    b.expressVectorInAnotherBodyFrame(s, b.getOutboardFrame(s).p(), G);

        // Torques add and include term due to offset in forces
        FB_G += FBx_G; // shiftForceFromTo(FBx_G, r_bG, r_BG);
        ++it;
    }
    
    return FB_G;
}

/** Joints only produce power when internal constraint forces have components along
    the mobilities of the joint (for example to satisfy prescribed motion). In 
    which case the joint power is the constraint forces projected onto the mobilities
    multiplied by the mobilities (internal coordinate velocities). Only constraints
    internal to the joint are accounted for, not external constraints that affect
    joint motion. */
double Joint::calcPower(const SimTK::State &s) const
{
    double power = 0;
    for(int i = 0; i < numCoordinates(); ++i) {
        if (get_coordinates(i).isPrescribed(s)) {
            // get the reaction force for this coordinate prescribed motion constraint
            const SimTK::Constraint &pc =
                _model->updMultibodySystem().updMatterSubsystem()
                    .getConstraint(get_coordinates(i)._prescribedConstraintIndex);
            power += pc.calcPower(s);
        }
    }

    return power;
}

//=============================================================================
// Helper
//=============================================================================
/* Calculate the equivalent spatial force, FB_G, acting on a mobilized body specified 
   by index acting at its mobilizer frame B, expressed in ground.  */
SimTK::SpatialVec Joint::calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s, 
    const SimTK::MobilizedBodyIndex mbx, const SimTK::Vector &mobilityForces) const
{
    // Get the mobilized body
    const SimTK::MobilizedBody mbd    = getModel().getMatterSubsystem().getMobilizedBody(mbx);
    const SimTK::UIndex        ustart = mbd.getFirstUIndex(s);
    const int                  nu     = mbd.getNumU(s);

    const SimTK::MobilizedBody ground = getModel().getMatterSubsystem().getGround();

    if (nu == 0) // No mobility forces (weld joint?).
        return SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0));

    // Construct the H (joint Jacobian, joint transition) matrix for this mobilizer
    SimTK::Matrix transposeH_PB_w(nu, 3);
    SimTK::Matrix transposeH_PB_v(nu, 3);
    // from individual columns
    SimTK::SpatialVec Hcol;
    
    // To obtain the joint Jacobian, H_PB (H_FM in Simbody) need to be realized to at least position
    _model->getMultibodySystem().realize(s, SimTK::Stage::Position);

    SimTK::Vector f(nu, 0.0);
    for(int i =0; i<nu; ++i){
        f[i] = mobilityForces[ustart + i];
        // Get the H matrix for this Joint by constructing it from the operator H*u
        Hcol = mbd.getH_FMCol(s, SimTK::MobilizerUIndex(i));
        const SimTK::Vector hcolw(Hcol[0]);
        const SimTK::Vector hcolv(Hcol[1]);

        transposeH_PB_w[i] = ~hcolw;
        transposeH_PB_v[i] = ~hcolv;
    }

    // Spatial force and torque vectors
    SimTK::Vector Fv(3, 0.0), Fw(3, 0.0);

    // Solve the pseudoinverse problem of Fv = pinv(~H_PB_G_v)*f;
    SimTK::FactorQTZ pinvForce(transposeH_PB_v);

    //if rank = 0, body force cannot contribute to the mobility force
    if(pinvForce.getRank() > 0)
        pinvForce.solve(f, Fv);
    
    // Now solve the pseudoinverse for torque for any unaccounted f: Fw = pinv(~H_PB_G_w)*(f - ~H_PB_G_v*Fv);
    SimTK::FactorQTZ pinvTorq(transposeH_PB_w);

    //if rank = 0, body torque cannot contribute to the mobility force
    if(pinvTorq.getRank() > 0)
        pinvTorq.solve(f, Fw);
    
    // Now we have two solution with either the body force Fv or body torque accounting for some or all of f
    SimTK::Vector fv =  transposeH_PB_v*Fv;
    SimTK::Vector fw =  transposeH_PB_w*Fw; 

    // which to choose? Choose the more effective as fx.norm/Fx.norm
    if(fv.norm() > SimTK::SignificantReal){ // if body force can contributes at all
        // if body torque can contribute too and it is more effective
        if(fw.norm() > SimTK::SignificantReal){
            if (fw.norm()/Fw.norm() > fv.norm()/Fv.norm() ){ 
                // account for f using torque, Fw, so compute Fv with remainder
                pinvForce.solve(f-fw, Fv);      
            }else{
                // account for f using force, Fv, first and Fw from remainder
                pinvTorq.solve(f-fv, Fw);
            }
        }
        // else no torque contribution and Fw should be zero
    }
    // no force contribution but have a torque
    else if(fw.norm() > SimTK::SignificantReal){
        // just Fw
    }
    else{
        // should be the case where gen force is zero.
        assert(f.norm() < SimTK::SignificantReal);
    }

    // The spatial forces above are expressed in the joint frame of the parent
    // Transform from parent joint frame, P into the parent body frame, Po
    const SimTK::Rotation R_PPo = (mbd.getInboardFrame(s).R());

    // Re-express forces in ground, first by describing force in the parent, Po, 
    // frame instead of joint frame
    SimTK::Vec3 vecFw = R_PPo*SimTK::Vec3::getAs(&Fw[0]);
    SimTK::Vec3 vecFv = R_PPo*SimTK::Vec3::getAs(&Fv[0]);

    //Force Acting on joint frame, B,  in child body expressed in Parent body, Po
    SimTK::SpatialVec FB_Po(vecFw, vecFv);

    const MobilizedBody parent = mbd.getParentMobilizedBody();
    // to apply spatial forces on bodies they must be expressed in ground
    vecFw = parent.expressVectorInAnotherBodyFrame(s, FB_Po[0], ground);
    vecFv = parent.expressVectorInAnotherBodyFrame(s, FB_Po[1], ground);

    // Package resulting torque and force as a spatial vec
    SimTK::SpatialVec FB_G(vecFw, vecFv);

    return FB_G;
}

void Joint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    //bool converting = false;
    if (documentVersion < XMLDocument::getLatestVersion()){
        if (documentVersion<30500){
            XMLDocument::renameChildNode(aNode, "location", "location_in_child"); 
            XMLDocument::renameChildNode(aNode, "orientation", "orientation_in_child");
        }
        // Version 30501 converted Connector_Body_ to Connector_PhysicalFrame_
        if (documentVersion < 30501) {
            // Handle any models that have the Joint connecting to Bodies instead
            // of PhyscialFrames
            XMLDocument::renameChildNode(aNode, "Connector_Body_",
                                                "Connector_PhysicalFrame_");
        }
        // Version 30505 changed "parent_body" connector name to "parent_frame"
        // Convert location and orientation into PhysicalOffsetFrames owned by the Joint
        if (documentVersion < 30505) {
            // Elements for the parent and child names the joint connects
            SimTK::Xml::element_iterator parentNameElt;
            SimTK::Xml::element_iterator childNameElt;
            // The names of the two PhysicalFrames this joint connects
            std::string parentFrameName("");
            std::string childFrameName("");

            SimTK::Xml::element_iterator connectors_node = aNode.element_begin("connectors");

            SimTK::Xml::element_iterator connectorElement =
                connectors_node->element_begin("Connector_PhysicalFrame_");
            while (connectorElement != aNode.element_end()) {
                // If connector name is "parent_body" rename it to "parent_frame"
                if (connectorElement->getRequiredAttributeValue("name") == "parent_body") {
                    connectorElement->setAttributeValue("name", "parent_frame");
                }
                // If connector name is "parent_frame" get the name of the connectee
                if (connectorElement->getRequiredAttributeValue("name") == "parent_frame"){
                    parentNameElt = connectorElement->element_begin("connectee_name");
                    parentNameElt->getValueAs<std::string>(parentFrameName);
                }
                if (connectorElement->getRequiredAttributeValue("name") == "child_body") {
                    connectorElement->setAttributeValue("name", "child_frame");
                }
                if (connectorElement->getRequiredAttributeValue("name") == "child_frame") {
                    childNameElt =  connectorElement->element_begin("connectee_name");
                    childNameElt->getValueAs<std::string>(childFrameName);
                }
                ++connectorElement;
            }

            SimTK::Xml::element_iterator locParentElt =
                aNode.element_begin("location_in_parent");
            SimTK::Xml::element_iterator orientParentElt =
                aNode.element_begin("orientation_in_parent");
            SimTK::Xml::element_iterator locChildElt =
                aNode.element_begin("location_in_child");
            SimTK::Xml::element_iterator orientChildElt =
                aNode.element_begin("orientation_in_child");

            Vec3 location_in_parent(0);
            Vec3 orientation_in_parent(0);
            Vec3 location_in_child(0);
            Vec3 orientation_in_child(0);

            if (locParentElt != aNode.element_end()){
                locParentElt->getValueAs<Vec3>(location_in_parent);
            }
            if (orientParentElt != aNode.element_end()){
                orientParentElt->getValueAs<Vec3>(orientation_in_parent);
            }
            if (locChildElt != aNode.element_end()){
                locChildElt->getValueAs<Vec3>(location_in_child);
            }
            if (orientChildElt != aNode.element_end()){
                orientChildElt->getValueAs<Vec3>(orientation_in_child);
            }

            // now append updated frames to the property list if they are not
            // identity transforms.
            if ((location_in_parent.norm() > 0.0) ||
                (orientation_in_parent.norm() > 0.0)) {
                XMLDocument::addPhysicalOffsetFrame(aNode, parentFrameName+"_offset",
                    parentFrameName, location_in_parent, orientation_in_parent);
                parentNameElt->setValue(parentFrameName + "_offset");
            }

            // again for the offset frame on the child
            if ((location_in_child.norm() > 0.0) ||
                (orientation_in_child.norm() > 0.0)) {
                XMLDocument::addPhysicalOffsetFrame(aNode, childFrameName + "_offset",
                    childFrameName, location_in_child, orientation_in_child);
                childNameElt->setValue(childFrameName + "_offset");
            }
        }

        // Version 30507 replaced Joint's CoordinateSet with a "coordinates"
        // list property.
        if (documentVersion < 30507) {
            if (aNode.hasElement("CoordinateSet")) {
                auto coordSetIter = aNode.element_begin("CoordinateSet");
                if (coordSetIter->hasElement("objects")) {
                    auto coordIter = coordSetIter->getRequiredElement("objects")
                                                   .element_begin("Coordinate");
                    if (coordIter != aNode.element_end()) {
                        // A "CoordinateSet" element exists, it contains an
                        // "objects" element, and the "objects" element contains
                        // at least one "Coordinate" element.

                        // Create an element for the new layout.
                        Xml::Element coordinatesElement("coordinates");
                        // Copy all "Coordinate" elements from the old layout.
                        while (coordIter != aNode.element_end()) {
                            coordinatesElement.appendNode(coordIter->clone());
                            ++coordIter;
                        }
                        // Insert new "coordinates" element.
                        aNode.insertNodeAfter(coordSetIter, coordinatesElement);
                    }
                }

                // Remove old "CoordinateSet" element.
                aNode.eraseNode(coordSetIter);
            }
        }

        // Version 30514 removed the user-facing "reverse" property from Joint.
        // The parent and child frames are swapped if a "reverse" element is
        // found and its value is "true".
        if (documentVersion < 30514) {
            auto reverseElt = aNode.element_begin("reverse");

            if (reverseElt != aNode.element_end()) {
                bool swapFrames = false;
                reverseElt->getValue().tryConvertToBool(swapFrames);

                if (swapFrames) {
                    std::string oldParentFrameName = "";
                    std::string oldChildFrameName  = "";

                    // Find names of parent and child frames. If more than one
                    // "parent_frame" or "child_frame" element exists, keep the
                    // first one. The "parent_frame" and "child_frame" elements
                    // may be listed in either order.
                    SimTK::Xml::element_iterator connectorsNode =
                        aNode.element_begin("connectors");
                    SimTK::Xml::element_iterator connectorElt = connectorsNode->
                        element_begin("Connector_PhysicalFrame_");
                    SimTK::Xml::element_iterator connecteeNameElt;

                    while (connectorElt != connectorsNode->element_end())
                    {
                        if (connectorElt->getRequiredAttributeValue("name") ==
                            "parent_frame" && oldParentFrameName.empty())
                        {
                            connecteeNameElt = connectorElt->
                                               element_begin("connectee_name");
                            connecteeNameElt->getValueAs<std::string>(
                                oldParentFrameName);
                        }
                        else if (connectorElt->getRequiredAttributeValue("name")
                                 == "child_frame" && oldChildFrameName.empty())
                        {
                            connecteeNameElt = connectorElt->
                                               element_begin("connectee_name");
                            connecteeNameElt->getValueAs<std::string>(
                                oldChildFrameName);
                        }
                        ++connectorElt;
                    }

                    // Swap parent and child frame names. If more than one
                    // "parent_frame" or "child_frame" element exists, assign
                    // the same value to all such elements.
                    connectorsNode = aNode.element_begin("connectors");
                    connectorElt = connectorsNode->element_begin(
                                   "Connector_PhysicalFrame_");

                    while (connectorElt != connectorsNode->element_end())
                    {
                        if (connectorElt->getRequiredAttributeValue("name") ==
                            "parent_frame")
                        {
                            connecteeNameElt = connectorElt->
                                               element_begin("connectee_name");
                            connecteeNameElt->setValue(oldChildFrameName);
                        }
                        else if (connectorElt->getRequiredAttributeValue("name")
                                 == "child_frame")
                        {
                            connecteeNameElt = connectorElt->
                                               element_begin("connectee_name");
                            connecteeNameElt->setValue(oldParentFrameName);
                        }
                        ++connectorElt;
                    }
                }

                // Remove "reverse" element regardless of its value (it is no
                // longer a property of Joint).
                aNode.eraseNode(reverseElt);
            }
        }

    }

    Super::updateFromXMLNode(aNode, versionNumber);
}

int Joint::assignSystemIndicesToBodyAndCoordinates(
    const SimTK::MobilizedBody& mobod,
    const OpenSim::PhysicalFrame* mobilized,
    const int& numMobilities,
    const int& startingCoordinateIndex) const
{
    // If not OpenSim body provided as the one being mobilized assume it is 
    // an intermediate body and ignore.
    if (mobilized){
        // Index can only be assigned to a parent or child body connected by this
        // Joint

        SimTK_ASSERT3( ( (mobilized == &getParentFrame()) || 
                         (mobilized == &getChildFrame()) ||
                         (mobilized == _slaveBodyForParent.get()) ||
                         (mobilized == _slaveBodyForChild.get()) ),
            "%s::'%s' - Cannot assign underlying system index to a PhysicalFrame '%s', "
            "which is not a parent or child Frame of this Joint.",
                      getConcreteClassName().c_str(),
                      getName().c_str(), mobilized->getName().c_str());

        // ONLY the base Joint can do this assignment
        mobilized->setMobilizedBodyIndex(mobod.getMobilizedBodyIndex());

        // Note that setting the mobilized body index of a PhysicalOffsetFrame
        // does not set it on the parent PhysicalFrame. 
        // Do the check and set it here as well since only the Joint can set the index.
        const PhysicalOffsetFrame* physOff =
            dynamic_cast<const PhysicalOffsetFrame*>(mobilized);
        if (physOff) {
            physOff->getParentFrame().setMobilizedBodyIndex(mobod.getMobilizedBodyIndex());
        }
    }
    const int nc = numCoordinates();
    SimTK_ASSERT3(numMobilities <= (nc - startingCoordinateIndex),
        "%s attempted to create an underlying SimTK::MobilizedBody that "
        "supplies %d mobilities but only %d required.",
                  getConcreteClassName().c_str(),
                  numMobilities, nc - startingCoordinateIndex);

    // Need a writable reference to this Joint so indices can be set on its
    // Coordinates.
    Self& mutableSelf = const_cast<Self&>(*this);

    int j = startingCoordinateIndex;
    for (int iq = 0; iq < numMobilities; ++iq){
        if (j < nc){ // assign
            mutableSelf.upd_coordinates(j)._mobilizerQIndex =
                SimTK::MobilizerQIndex(iq);
            mutableSelf.upd_coordinates(j)._bodyIndex =
                mobod.getMobilizedBodyIndex();
            j++;
        }
        else{
            std::string msg = getConcreteClassName() +
                " creating MobilizedBody with more mobilities than declared Coordinates.";
            throw Exception(msg);
        }
    }
    return j;
}

/* Return the equivalent (internal) SimTK::Rigid::Body for a given parent OR
child OpenSim::Body. Not guaranteed to be valid until after addToSystem on
Body has be called  */
const SimTK::Body& Joint::getParentInternalRigidBody() const
{
    if (_slaveBodyForParent){
        return _slaveBodyForParent->extractInternalRigidBody();
    }

    return static_cast<const PhysicalFrame&>(getParentFrame()
        .findBaseFrame()).extractInternalRigidBody();
}
const SimTK::Body& Joint::getChildInternalRigidBody() const
{
    if (_slaveBodyForChild){
        return _slaveBodyForChild->extractInternalRigidBody();
    }

    return static_cast<const PhysicalFrame&>(getChildFrame()
        .findBaseFrame()).extractInternalRigidBody();;
}
