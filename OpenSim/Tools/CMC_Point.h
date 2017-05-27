#ifndef CMCPoint_h__
#define CMCPoint_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  CMC_Point.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//============================================================================
// INCLUDE
//============================================================================
#include "CMC_Task.h"

namespace OpenSim {

class Body;

//=============================================================================
//=============================================================================
/**
 * A class for specifying and computing parameters for tracking a point.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMC_Point : public CMC_Task {
OpenSim_DECLARE_CONCRETE_OBJECT(CMC_Point, CMC_Task);

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Location of the tracked point on the body expressed in the body-local
    coordinate frame. */
    PropertyDblVec3 _propPoint;

    // REFERENCES
    SimTK::Vec3 &_point;

    // Work Variables
    SimTK::Vec3 _p,_v,_inertialPTrk,_inertialVTrk;
    Body *_wrtBody,*_expressBody;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    CMC_Point(const SimTK::Vec3 &aPoint = SimTK::Vec3(0));
    CMC_Point(const CMC_Point &aTask);
    virtual ~CMC_Point();

private:
    void setNull();
    void setupProperties();
    void copyData(const CMC_Point &aTask);
    void updateWorkVariables(const SimTK::State& s);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    CMC_Point& operator=(const CMC_Point &aTask);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setModel(Model& aModel) override;
    void setPoint(const SimTK::Vec3 &aPoint);
    SimTK::Vec3 getPoint() const;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    void computeErrors(const SimTK::State& s, double aT) override;
    void computeDesiredAccelerations(const SimTK::State& s, double aT) override;
    void computeDesiredAccelerations(const SimTK::State& s, double aTI,double aTF) override;
    void computeAccelerations(const SimTK::State& s ) override;

    //--------------------------------------------------------------------------
    // XML
    //--------------------------------------------------------------------------
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;

//=============================================================================
};  // END of class CMC_Point
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CMCPoint_h__


