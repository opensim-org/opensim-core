#ifndef __WrapTorus_h__
#define __WrapTorus_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  WrapTorus.h                            *
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


// INCLUDE
#include "WrapObject.h"
#include <OpenSim/Common/PropertyDbl.h>

namespace OpenSim {

class Model;
class PathWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a torus for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapTorus : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapTorus, WrapObject);

private:
    struct CircleCallback {
        double p1[3], p2[3], r;
    };

//=============================================================================
// DATA
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(innerRadius, double, "The inner radius of the torus.");
    OpenSim_DECLARE_PROPERTY(outerRadius, double, "The inner radius of the torus.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    WrapTorus();
    WrapTorus(const WrapObject* aWrapTorus);
    WrapTorus(const WrapTorus& aWrapTorus);
    virtual ~WrapTorus();

#ifndef SWIG
    WrapTorus& operator=(const WrapTorus& aWrapTorus);
#endif
    void copyData(const WrapTorus& aWrapTorus);
    const char* getWrapTypeName() const override;
    std::string getDimensionsString() const override;
    SimTK::Real getInnerRadius() const;
    SimTK::Real getOuterRadius() const;

    /** Scale the torus's dimensions. The base class (WrapObject) scales the
        origin of the torus in the body's reference frame. */
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;

    void connectToModelAndBody(Model& aModel, PhysicalFrame& aBody) override;
protected:
    int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
        const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const override;
    /// Implement generateDecorations to draw geometry in visualizer
    void generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const override;
    void setupProperties();

private:
    void setNull();
    int findClosestPoint(double radius, double p1[], double p2[],
        double* xc, double* yc, double* zc,
        int wrap_sign, int wrap_axis) const;
    static void calcCircleResids(int numResid, int numQs, double q[],
        double resid[], int *flag2, void *ptr);

//=============================================================================
};  // END of class WrapTorus
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapTorus_h__


