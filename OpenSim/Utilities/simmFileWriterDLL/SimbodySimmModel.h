#ifndef __SimbodySimmModel_h__
#define __SimbodySimmModel_h__
// SimbodySimmModel.h
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
*   1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
*   2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
*   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
*   4. Credits to developers may not be removed from executables
*     created from modifications of the source.
*   5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
    
    static const double defaultAxes[][3];

//=============================================================================
};  // END of class SimbodySimmModel
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodySimmModel_h__


