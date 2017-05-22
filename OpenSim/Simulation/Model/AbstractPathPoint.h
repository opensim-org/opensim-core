#ifndef OPENSIM_ABSTRACT_PATH_POINT_H_
#define OPENSIM_ABSTRACT_PATH_POINT_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  AbstractPathPoint.h                         *
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
#include "OpenSim/Simulation/Model/Point.h"
#include "OpenSim/Simulation/Model/PhysicalFrame.h"

namespace OpenSim {

class WrapObject;

//=============================================================================
//=============================================================================
/**
 * An abstract class implementing a path point interface.
 */
class OSIMSIMULATION_API AbstractPathPoint : public Point {
    OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractPathPoint, Point);
public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(parent_frame, PhysicalFrame,
        "The frame in which this path point is defined.");
public:
//=============================================================================
// METHODS
//=============================================================================
    AbstractPathPoint() = default;
    virtual ~AbstractPathPoint() {}

    /** get the relative location of the path point with respect to the body
    it is connected to. */
    virtual SimTK::Vec3 getLocation(const SimTK::State& s) const = 0;

    /** get the parent PhysicalFrame in which the PathPoint is defined */
    const PhysicalFrame& getParentFrame() const;
    /** set the parent PhysicalFrame in which the PathPoint is defined */
    void setParentFrame(const OpenSim::PhysicalFrame& aFrame);

    /** <b>(Deprecated)</b> Old PathPoint interface.
        Use getParentFrame() instead to get the PhysicalFrame in which
        this PathPoint is defined.*/
    DEPRECATED_14("Use getParentFrame() instead.")
    const  PhysicalFrame& getBody() const;
    /** <b>(Deprecated)</b> Old PathPoint interface.
        Use setParentFrame() instead.*/
    DEPRECATED_14("Use setParentFrame() instead.")
    void setBody(const PhysicalFrame& body);
    /** <b>(Deprecated)</b> Old PathPoint interface.
        Use getParentFrame().getName() instead.*/
    DEPRECATED_14("Use getParentFrame().getName() instead.")
    const std::string& getBodyName() const;

    virtual void scale(const SimTK::Vec3& scaleFactors) = 0;

    virtual const WrapObject* getWrapObject() const { return nullptr; }

    virtual bool isActive(const SimTK::State& s) const { return true; }

    /** get the partial derivative of the point location in the parent frame
        w.r.t. to the coordinates(Q) and expressed in the parent frame. */
    virtual SimTK::Vec3 getdPointdQ(const SimTK::State& s) const = 0;

    static void deletePathPoint(AbstractPathPoint* aPoint) {
        if (aPoint) delete aPoint;
    }

    /** Update the use of *body* property in previous revisions to the 
        parent_frame (Socket) for the path point's dependency on a 
        PhysicalFrame. 
        @note If overriding updateFromXMLNode of derived classes, do not
        forget to invoke Super::updateFromXMLNode to include this update.*/
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;

//=============================================================================
};  // END of class AbstractPathPoint
//=============================================================================

//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ABSTRACT_PATH_POINT_H_
