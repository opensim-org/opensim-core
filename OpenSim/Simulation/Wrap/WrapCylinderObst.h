#ifndef OPENSIM_WRAP_CYLINDER_OBST_H_
#define OPENSIM_WRAP_CYLINDER_OBST_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  WrapCylinderObst.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
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


// INCLUDE
#include "WrapObject.h"

namespace OpenSim {

class PathWrap;
class WrapResult;

/** @cond **/ // hide from Doxygen

//=============================================================================
//=============================================================================
/**
 * A class implementing a cylinder obstacle for muscle wrapping, based on
 * algorithm presented in Garner & Pandy (2000).
 *
 * @author Brian Garner, derived from Peter Loan
 * updated for OpenSim 4.0 by Benjamin Michaud, 2019.
 */
class OSIMSIMULATION_API WrapCylinderObst : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapCylinderObst, WrapObject);
private:
//==============================================================================
// PROPERTIES
//==============================================================================
    enum WrapDirectionEnum  // The prescribed direction of wrapping about the cylinders' z-axis
    {
        righthand,
        lefthand
    };

public:
    OpenSim_DECLARE_PROPERTY(radius, double, "The radius of the cylinder.");
    OpenSim_DECLARE_PROPERTY(length, double, "The length of the cylinder.");
    OpenSim_DECLARE_PROPERTY(wrapDirection, std::string, "Describe if the cylinder is right or left handed.");

private:
    // Facilitate prescription of wrapping direction around obstacle: "righthand" or "lefthand".
    // In traversing from the 1st point (P) to the 2nd (S), the path will wrap either
    //    right-handed or left-handed about the obstacle's z-axis.
    WrapDirectionEnum m_wrapDirection;


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    WrapCylinderObst();
    virtual ~WrapCylinderObst();


    const char* getWrapTypeName() const override;
    std::string getDimensionsString() const override;

    double getRadius() const;
    void setRadius(double aRadius);
    double getLength() const;
    void setLength(double aLength);
    int getWrapDirection() const;


protected:
    int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
        const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const override;

    void extendFinalizeFromProperties() override;

private:
    void constructProperties();

    void initCircleWrapPts();

//=============================================================================
};  // END of class WrapCylinder
//=============================================================================
//=============================================================================

/** @endcond **/

} // end of namespace OpenSim

#endif // OPENSIM_WRAP_CYLINDER_OBST_H_


