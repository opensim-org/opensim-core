#ifndef OPENSIM_WRAP_SPHERE_H_
#define OPENSIM_WRAP_SPHERE_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  WrapSphere.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
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


// INCLUDE
#include "WrapObject.h"

namespace OpenSim {

class PathWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a sphere for muscle wrapping.
 *
 * @author Peter Loan
 * updated for OpenSim 4.0 by Benjamin Michaud, 2019.
 */
class OSIMSIMULATION_API WrapSphere : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapSphere, WrapObject);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(radius, double, "The radius of the sphere.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    WrapSphere();
    virtual ~WrapSphere();

    const char* getWrapTypeName() const override;
    std::string getDimensionsString() const override;
    double getRadius() const;

    /** Scale the sphere by the average of the scale factors in each direction.
        The base class (WrapObject) scales the origin of the sphere in the
        body's reference frame. */
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;

protected:
    int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
        const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const override;

    /// Implement generateDecorations to draw geometry in visualizer
    void generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const override;

    void extendFinalizeFromProperties() override;
private:
    void constructProperties();

//=============================================================================
};  // END of class WrapSphere
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WRAP_SPHERE_H_


