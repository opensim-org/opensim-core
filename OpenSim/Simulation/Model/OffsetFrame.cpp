/* -------------------------------------------------------------------------- *
 *                          OpenSim:  OffsetFrame.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matt DeMers, Ajay Seth                                          *
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
#include "OffsetFrame.h"
#include "PhysicalFrame.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
template <class C>
OffsetFrame<C>::OffsetFrame() : C()
{
    setNull();
    this->constructInfrastructure();
}

//_____________________________________________________________________________
/**
 * Constructors.
 */
template <class C>
OffsetFrame<C>::OffsetFrame(const C& parent,
    const SimTK::Transform& offset) : C()
{
    setNull();
    this->constructInfrastructure();
    setParentFrame(parent);
    setOffsetTransform(offset);
}
//_____________________________________________________________________________
/**
* Set a null frame as Identity rotation, 0 translation
*/
template <class C>
void OffsetFrame<C>::setNull()
{
    _offsetTransform.setToNaN();
    this->setAuthors("Matt DeMers, Ajay Seth");
}
//_____________________________________________________________________________
/**
* construct properties
*/
template <class C>
void OffsetFrame<C>::constructProperties()
{
    SimTK::Vec3 zero(0.0, 0.0, 0.0);
    constructProperty_translation(zero);
    constructProperty_orientation(zero);
    // transform at default
}

template <class C>
void OffsetFrame<C>::constructConnectors()
{
    this->template constructConnector<C>("parent");
}

//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/*
 * Implementation of Frame interface by OffsetFrame.
 *
 */
template <class C>
const SimTK::Transform& OffsetFrame<C>::
    calcGroundTransform(const SimTK::State& s) const
{
    if (!this->isCacheVariableValid(s, "ground_transform")){
        this->template setCacheVariableValue<SimTK::Transform>(s,
                "ground_transform",
                getParentFrame().getGroundTransform(s)*getOffsetTransform());
    }
    // return X_GF = X_GB * X_BF where F is the offset frame;
    return this->template getCacheVariableValue<SimTK::Transform>(s,
            "ground_transform");
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
template <class C>
void OffsetFrame<C>::setParentFrame(const C& parent)
{ 
    this->template updConnector<C>("parent").connect(parent);
}

template <class C>
const C& OffsetFrame<C>::getParentFrame() const
{
    return this->template getConnector<C>("parent").getConnectee();
}

template <class C>
const SimTK::Transform& OffsetFrame<C>::getOffsetTransform() const
{
    return _offsetTransform;
}

template <class C>
void OffsetFrame<C>::setOffsetTransform(const SimTK::Transform& xform)
{
    _offsetTransform = xform;
    // Make sure properties are updated in case we either call gettters or
    // serialize after this call
    set_translation(xform.p());
    set_orientation(xform.R().convertRotationToBodyFixedXYZ());
}

template<class C>
const Frame& OffsetFrame<C>::extendFindBaseFrame() const
{
    // check if parent is also an offset 
    const OffsetFrame<C>* parentOffset =
        dynamic_cast<const OffsetFrame<C>*>(&getParentFrame());
    if (parentOffset) {
        return parentOffset->findBaseFrame();
    }
    else {
        return getParentFrame();
    }
}

template<class C>
SimTK::Transform OffsetFrame<C>::extendFindTransformInBaseFrame() const
{
    // check if parent is also an offset 
    const OffsetFrame<C>* parentOffset
        = dynamic_cast<const OffsetFrame<C>*>(&getParentFrame());
    if (parentOffset) {
        return parentOffset->findTransformInBaseFrame()
                * getOffsetTransform();
    }
    else {
        return getOffsetTransform();
    }
}

template<class C>
void OffsetFrame<C>::extendFinalizeFromProperties()
{
    _offsetTransform.updP() = get_translation();
    _offsetTransform.updR().setRotationToBodyFixedXYZ(get_orientation());
}


//Specialization for Offset on a Frame of type PhysicalFrame
template<>
OffsetFrame<PhysicalFrame>::OffsetFrame(const PhysicalFrame& parent,
    const SimTK::Transform& offset) : PhysicalFrame() {
    setNull();
    constructInfrastructure();
    setParentFrame(parent);
    setOffsetTransform(offset);
}

template <>
void OffsetFrame<PhysicalFrame>::
extendAddToSystem(SimTK::MultibodySystem& system) const
{
    SimTK::Transform x;
    // If the properties, topology or coordinate values, change, 
    // Stage::Position will be invalid.
    addCacheVariable("ground_transform", x, SimTK::Stage::Position);
    setMobilizedBodyIndex(getParentFrame().getMobilizedBodyIndex());
}


// Explicit template instantiation
namespace OpenSim {
template class OffsetFrame<PhysicalFrame>;
}
