#ifndef __SimbodySimmModel_h__
#define __SimbodySimmModel_h__
/* -------------------------------------------------------------------------- *
 *                            SimbodySimmModel.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
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

// INCLUDES
#include <iostream>
#include <string>
#include "osimSimmFileWriterDLL.h"
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Function.h>
#include "SimbodySimmBody.h"
#include "SimbodySimmJoint.h"
#include "SimbodySimmGencoord.h"
#include "SimbodySimmFunction.h"
#include <SimTKsimbody.h>

#ifdef SWIG
    #ifdef OSIMSIMMFILEWRITER_API
        #undef OSIMSIMMFILEWRITER_API
        #define OSIMSIMMFILEWRITER_API
    #endif
#endif

namespace OpenSim {

class SimmFileWriter;
class Model;
class Muscle;
class ForceSet;
class Joint;
class Body;
class Coordinate;
class Joint;
class MarkerSet;
class Function;

//=============================================================================
//=============================================================================
/**
 * A class to hold a SIMM model representing a Simbody model.
 *
 * @authors Peter Loan
 * @version 1.0
 */
class OSIMSIMMFILEWRITER_API SimbodySimmModel : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(SimbodySimmModel, Object);

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Pointer to the OpenSim model that this object was created from. */
    const Model* _model;

    /** bodies */
    Array<SimbodySimmBody*> _simmBody;

    /** joints */
    Array<SimbodySimmJoint*> _simmJoint;

    /** gencoords */
    Array<SimbodySimmGencoord*> _simmGencoord;

    /** functions for joint file */
    Array<SimbodySimmFunction*> _simmJointFunction;

    /** functions for muscle file */
    Array<SimbodySimmFunction*> _simmMuscleFunction;

    int _maxFunctionUserNumber;
    int _uniqueJointNumber;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~SimbodySimmModel();
    SimbodySimmModel();
    SimbodySimmModel(const Model* aModel);
    SimbodySimmModel(const SimbodySimmModel& aModel);

#ifndef SWIG
    SimbodySimmModel& operator=(const SimbodySimmModel &aModel);
#endif

private:
    void setNull();
    void copyData(const SimbodySimmModel &aEngine);
    void connectSimbodySimmModel(const Model* aModel);
    bool jointArrayContains(const std::string& aName);

public:
   bool writeJointFile(const std::string& aFileName) const;
   const std::string& getGravityLabel(const SimTK::Vec3& aGravity) const;
   const Function* isDependent(const Coordinate* aCoordinate, const Coordinate** rIndependentCoordinate) const;
   void convertBody(const OpenSim::Body& aBody);
   void convertJoint(const Joint& joint);
   bool isChildJointNeeded(const OpenSim::Joint& aJoint);
    bool isParentJointNeeded(const OpenSim::Joint& aJoint);
    void addSimmJoint(SimbodySimmJoint* joint);
   void makeSimmJoint(const std::string& aName, const std::string& aParentName, const std::string& aChildName,
                      SimTK::Vec3& aLocation, SimTK::Vec3& aOrientation);
   bool addExtraJoints(const OpenSim::Joint& aJoint, std::string& rParentName, std::string& rChildName);
   void addBody(const OpenSim::Body& aBody);
   void addGencoord(const Coordinate* aCoordinate);
   int addJointFunction(const Function* aFunction, Coordinate::MotionType aXType, Coordinate::MotionType aYType);
   int addMuscleFunction(const Function* aFunction, Coordinate::MotionType aXType, Coordinate::MotionType aYType);
   void writeWrapObjects(OpenSim::Body& aBody, std::ofstream& aStream) const;
    int getUniqueFunctionUserNumber(const OpenSim::Function* aFunction);
    bool writeMuscleFile(const std::string& aFileName);
    bool writeMuscle(Muscle& aMuscle, const ForceSet& aActuatorSet, std::ofstream& aStream);

//=============================================================================
};  // END of class SimbodySimmModel
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodySimmModel_h__


