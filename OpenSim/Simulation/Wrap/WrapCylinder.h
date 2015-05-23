#ifndef __WrapCylinder_h__
#define __WrapCylinder_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapCylinder.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "WrapObject.h"

namespace OpenSim {

class VisibleObject;
class Body;
class Model;
class PathPoint;
class PathWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a cylinder for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapCylinder : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapCylinder, WrapObject);

//=============================================================================
// DATA
//=============================================================================

    PropertyDbl _radiusProp;
    double& _radius;

    PropertyDbl _lengthProp;
    double& _length;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    WrapCylinder();
    WrapCylinder(const WrapCylinder& aWrapCylinder);
    virtual ~WrapCylinder();

#ifndef SWIG
    WrapCylinder& operator=(const WrapCylinder& aWrapCylinder);
#endif
   void copyData(const WrapCylinder& aWrapCylinder);

    double getRadius() const { return _radius; }
    void setRadius(double aRadius) { _radius = aRadius; }
    double getLength() const { return _length; }
    void setLength(double aLength) { _length = aLength; }

    virtual const char* getWrapTypeName() const;
    virtual std::string getDimensionsString() const;
    virtual void scale(const SimTK::Vec3& aScaleFactors);

    void connectToModelAndBody(Model& aModel, OpenSim::PhysicalFrame& aBody) override;
#ifndef SWIG
    virtual int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
        const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const;
#endif
protected:
    void setupProperties();

private:
    void setNull();
    void _make_spiral_path(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
                                                 bool far_side_wrap,WrapResult& aWrapResult) const;
    void _calc_spiral_wrap_point(const SimTK::Vec3& r1a,
                                                         const SimTK::Vec3& axial_vec,
                                                         double m[4][4],
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

#endif // __WrapCylinder_h__


