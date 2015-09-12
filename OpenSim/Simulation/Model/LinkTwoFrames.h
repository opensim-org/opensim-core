#ifndef OPENSIM_LINK_TWO_FRAMES_H_
#define OPENSIM_LINK_TWO_FRAMES_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  LinkTwoFrames.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

// INCLUDE
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Common/ScaleSet.h>

namespace OpenSim {

class PhysicalOffsetFrame;

//=============================================================================
//=============================================================================
/**
 * LinkTwoFrames is a utility class to extend a Component such that it connects
 * two Frames. For example, a WeldConstraint and BushingForces operate between
 * two Frames to restrict their motion. A LinkTwoFrames<Force, PhysicalFrame>,
 * for example, is a Force that operates between two PhyscialFrames and it is 
 * the base class for BushingForce.
 * (A class whose super class is a template parameter is called a mixin class.)
 *
 * @code class BushingForce : public LinkTwoFrames<Force, PhysicalFrame> @endcode
 *
 * @author Ajay Seth
 */
template <class C = Component, class F = Frame>
class LinkTwoFrames : public C {
    OpenSim_DECLARE_ABSTRACT_OBJECT_T(LinkTwoFrames, C, C);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /// Frames added to satisfy the connectors of this LinkTwoFrames Component
    OpenSim_DECLARE_LIST_PROPERTY(frames, F,
        "Frames created/added to satisfy this component's connections.");

//=============================================================================
// PUBLIC METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** By default, the LinkTwoFrames is not connected to any parent frame, and its
    transform is an identity transform.
    */
    LinkTwoFrames();

    /** Convenience Constructor.
    Create a WeldConstraint bewteen two PhysicalFrames, frame1 and frame2.
    @param[in] name         the name of this LinkTwoFrames component
    @param[in] frame1Name   the name of the first PhysicalFrame being linked
    @param[in] frame2Name   the name of the second PhysicalFrame being linked
    */
    LinkTwoFrames(const std::string &name,
        const std::string& frame1Name,
        const std::string& frame2Name);

    /** Convenience Constructor
    Construct a WeldConstraint where the weld frames are specified in terms of their
    transforms in their respective PhysicalFrames.

    @param[in] name         the name of this LinkTwoFrames component
    @param[in] frame1       the first Frame that the component links
    @param[in] transformInFrame1    offset Transform in the first frame
    @param[in] frame2       the second Frame that the component links
    @param[in] transformInFrame2    offset Transform in the second frame
    */
    LinkTwoFrames(const std::string &name,
        const F& frame1, const SimTK::Transform& transformInFrame1,
        const F& frame2, const SimTK::Transform& transformInFrame2);

    /** Backwards compatible Convenience Constructor
    LinkTwoFrames with offsets specified in terms of the location and orientation 
    in respective PhysicalFrames.

    @param[in] name     the name of this LinkTwoFrames component
    @param[in] frame1   the first Frame that the component links
    @param[in] locationInFrame1    Vec3 of the location offset in the first frame
    @param[in] orientationInFrame1 Vec3 of the XYZ body-fixed Euler angles of the
                                   orientation offset in frame 1.
    @param[in] frame2    the second Frame that the component links
    @param[in] locationInFrame2    Vec3 of the location of the weld in the second frame
    @param[in] orientationInFrame1 Vec3 of the XYZ body-fixed Euler angles
    of the weld frame orientation in frame2.
    */
    LinkTwoFrames(const std::string &name,
        const F& frame1,
        const SimTK::Vec3& locationInFrame1, const SimTK::Vec3& orientationInFrame1,
        const F& frame2,
        const SimTK::Vec3& locationInFrame2, const SimTK::Vec3& orientationInFrame2);

    // use compiler generated destructor, copy constructor and assignment operator

    /** Access the first frame the LinkTwoFrames component connects */
    const F& getFrame1() const;
    /** Access the second frame the LinkTwoFrames component connects */
    const F& getFrame2() const;

    /**
    * Scale a joint based on XYZ scale factors for PhysicalFrames.
    * Generic behavior is to scale the locations of parent and child offsets
    * according to scale factors of the physical frame upon which they are located.
    *
    * Joint subclasses should invoke this method before scaling joint specific
    * properties
    *
    * @param aScaleSet Set of XYZ scale factors for PhysicalFrames.
    */
    virtual void scale(const ScaleSet& aScaleSet);

protected:
    /** @name Component Interface
    These methods adhere to the Component Interface**/
    /**@{**/
    void constructConnectors() override;
    void extendFinalizeFromProperties() override;
    /**@}**/

private:

    void constructProperties() override;

//=============================================================================
}; // END of class OffsetFrame
//=============================================================================

//=============================================================================
// Implementation of OffsetFrame<C> template methods
//=============================================================================
// Default constructor
template <class C, class F>
LinkTwoFrames<C, F>::LinkTwoFrames() : C()
{
    this->setAuthors("Ajay Seth");
    this->constructInfrastructure();
}

// Convenience constructors
template <class C, class F>
LinkTwoFrames<C, F>::LinkTwoFrames(const std::string &name,
    const std::string& frame1Name,
    const std::string& frame2Name) : LinkTwoFrames<C, F>()
{
    setName(name);
    updConnector<F>("frame1").set_connectee_name(frame1Name);
    updConnector<F>("frame2").set_connectee_name(frame2Name);
}

template <class C, class F>
LinkTwoFrames<C, F>::LinkTwoFrames(const std::string &name,
    const F& frame1, const SimTK::Transform& transformInFrame1,
    const F& frame2, const SimTK::Transform& transformInFrame2)
    : LinkTwoFrames()
{
    setName(name);

    PhysicalOffsetFrame frame1Offset(frame1.getName() + "_offset",
        frame1, transformInFrame1);

    PhysicalOffsetFrame frame2Offset(frame2.getName() + "_offset",
        frame2, transformInFrame2);

    // Append the offset frames to the Joints internal list of frames
    append_frames(frame1Offset);
    append_frames(frame2Offset);

    updConnector<PhysicalFrame>("frame1").set_connectee_name(
        getName() + "/" + frame1Offset.getName());
    updConnector<PhysicalFrame>("frame2").set_connectee_name(
        getName() + "/" + frame2Offset.getName());
}


template <class C, class F>
LinkTwoFrames<C, F>::LinkTwoFrames(const std::string &name,
    const F& frame1,
    const SimTK::Vec3& locationInFrame1, const SimTK::Vec3& orientationInFrame1,
    const F& frame2,
    const SimTK::Vec3& locationInFrame2, const SimTK::Vec3& orientationInFrame2)
    : LinkTwoFrames(name,
        frame1, SimTK::Transform(SimTK::Rotation(SimTK::BodyRotationSequence,
            orientationInFrame1[0], SimTK::XAxis,
            orientationInFrame1[1], SimTK::YAxis,
            orientationInFrame1[2], SimTK::ZAxis), locationInFrame1),
        frame2, SimTK::Transform(SimTK::Rotation(SimTK::BodyRotationSequence,
            orientationInFrame2[0], SimTK::XAxis,
            orientationInFrame2[1], SimTK::YAxis,
            orientationInFrame2[2], SimTK::ZAxis), locationInFrame2))
{}


template <class C, class F>
void LinkTwoFrames<C, F>::constructProperties()
{
    //Default frames list is empty
    this->constructProperty_frames();
}

template <class C, class F>
void LinkTwoFrames<C,F>::constructConnectors()
{
    this->template constructConnector<F>("frame1");
    this->template constructConnector<F>("frame2");
}

template <class C, class F>
const F& LinkTwoFrames<C, F>::getFrame1() const
{
    return this->getConnector<F>("frame1").getConnectee();
}

template <class C, class F>
const F& LinkTwoFrames<C, F>::getFrame2() const
{
    return this->getConnector<F>("frame2").getConnectee();
}

template<class C, class F>
void LinkTwoFrames<C, F>::scale(const ScaleSet& scaleSet)
{
    SimTK::Vec3 frame1Factors(1.0);
    SimTK::Vec3 frame2Factors(1.0);

    // Find the factors associated with the PhysicalFrames this Joint connects
    const string& base1Name = this->getFrame1().findBaseFrame().getName();
    const string& base2Name = this->getFrame2().findBaseFrame().getName();
    // Get scale factors
    bool found_b1 = false;
    bool found_b2 = false;
    for (int i = 0; i < scaleSet.getSize(); i++) {
        Scale& scale = scaleSet.get(i);
        if (!found_b1 && (scale.getSegmentName() == base1Name)) {
            scale.getScaleFactors(frame1Factors);
            found_b1 = true;
        }
        if (!found_b2 && (scale.getSegmentName() == base2Name)) {
            scale.getScaleFactors(frame2Factors);
            found_b2 = true;
        }
        if (found_b1 && found_b2)
            break;
    }

    // if the frame is owned by this Joint scale it,
    // otherwise let the owner of the frame decide.
    int found = getProperty_frames().findIndex(getFrame1());
    if (found >= 0) {
        PhysicalOffsetFrame* offset
            = dynamic_cast<PhysicalOffsetFrame*>(&upd_frames(found));
        if (offset)
            offset->scale(frame1Factors);
    }
    found = getProperty_frames().findIndex(getFrame2());
    if (found >= 0) {
        PhysicalOffsetFrame* offset
            = dynamic_cast<PhysicalOffsetFrame*>(&upd_frames(found));
        if (offset)
            offset->scale(frame2Factors);
    }
}

template<class C, class F>
void LinkTwoFrames<C, F>::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
    for (int i = 0; i < this->getProperty_frames().size(); ++i) {
        this->addComponent(&upd_frames(i));
    }
}

} // end of namespace OpenSim

#endif // OPENSIM_LINK_TWO_FRAMES_H_


