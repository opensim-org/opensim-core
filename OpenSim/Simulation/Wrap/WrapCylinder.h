#ifndef OPENSIM_WRAP_CYLINDER_H_
#define OPENSIM_WRAP_CYLINDER_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapCylinder.h                          *
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
 * A class implementing a cylinder for muscle wrapping.
 *
 * @author Peter Loan  
 * updated for OpenSim 4.0 by Benjamin Michaud, 2019.
 */
class OSIMSIMULATION_API WrapCylinder : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapCylinder, WrapObject);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(radius, double, "The radius of the cylinder.");
    OpenSim_DECLARE_PROPERTY(length, double, "The length of the cylinder.");

//=============================================================================
// METHODS
//=============================================================================
public:
    WrapCylinder();
    virtual ~WrapCylinder();

    const char* getWrapTypeName() const override;
    std::string getDimensionsString() const override;

    /** Scale the cylinder's dimensions. The base class (WrapObject) scales the
        origin of the cylinder in the body's reference frame. */
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;

protected:
    int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
        const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const override;
    // WrapTorus uses WrapCylinder::wrapLine.
    friend class WrapTorus;

    /// Implement generateDecorations to draw geometry in visualizer
    void generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const override;

    void extendFinalizeFromProperties() override;

private:
    void constructProperties();

    void _make_spiral_path(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                                                 bool far_side_wrap,WrapResult& aWrapResult) const;
    void _calc_spiral_wrap_point(const SimTK::Vec3& r1a,
                                 const SimTK::Vec3& axial_vec,
                                 const SimTK::Rotation& rotation,
                                 const SimTK::Vec3& axis,
                                 double sense,
                                 double t,
                                 double theta,
                                 SimTK::Vec3& wrap_pt) const;


    bool _adjust_tangent_point(SimTK::Vec3& pt1,
                                                      SimTK::Vec3& dn,
                                                      SimTK::Vec3& r1,
                                                      SimTK::Vec3& w1) const;

//=============================================================================
};  // END of class WrapCylinder
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WRAP_CYLINDER_H_


