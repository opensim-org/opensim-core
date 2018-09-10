#ifndef __WrapSphereObst_h__
#define __WrapSphereObst_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  WrapSphereObst.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Brian Garner, Peter Loan                                        *
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
#include "WrapObject.h"
#include <OpenSim/Common/PropertyDbl.h>

namespace OpenSim {

class Model;
class PathWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a sphere obstacle for muscle wrapping, based on the
 * algorithm presented in Garner & Pandy (2000).
 *
 * @author Brian Garner, derived from Peter Loan
 * @version 0.1
 */
class OSIMSIMULATION_API WrapSphereObst : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapSphereObst, WrapObject);

//=============================================================================
// DATA
//=============================================================================
private:
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
    WrapSphereObst();
    WrapSphereObst(const WrapSphereObst& aWrapSphereObst);
    virtual ~WrapSphereObst();

#ifndef SWIG
    WrapSphereObst& operator=(const WrapSphereObst& aWrapSphereObst);
#endif
   void copyData(const WrapSphereObst& aWrapSphereObst);

    double getRadius() const { return _radius; }
    void setRadius(double aRadius) { _radius = aRadius; }
    double getLength() const { return _length; }
    void setLength(double aLength) { _length = aLength; }

    const char* getWrapTypeName() const override;
    std::string getDimensionsString() const override;
    void connectToModelAndBody(Model& aModel, PhysicalFrame& aBody) override;
protected:
    int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
        const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const override;
    void setupProperties();

private:
    void setNull();

//=============================================================================
};  // END of class WrapCylinder
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapCylinder_h__


