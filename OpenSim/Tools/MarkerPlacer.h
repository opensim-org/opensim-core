#ifndef __MarkerPlacer_h__
#define __MarkerPlacer_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  MarkerPlacer.h                          *
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
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>
#include "osimToolsDLL.h"
#include <SimTKcommon/internal/ResetOnCopy.h>

namespace SimTK {
class State;
}

namespace OpenSim {

class Model;
class MarkerData;
class IKTaskSet;
class IKTrial;
class Storage;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to place markers
 * on a model (presumably after it has been scaled to fit a subject).
 *
 * MarkerPlacer is bundled with ModelScaler and GenericModelMaker to 
 * form the ScaleTool
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API MarkerPlacer : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MarkerPlacer, Object);

//=============================================================================
// DATA
//=============================================================================
private:

protected:
    // whether or not to apply marker placer
    PropertyBool _applyProp;
    bool &_apply;

    // name of marker file that contains marker locations in the static pose
    PropertyStr _markerFileNameProp;
    std::string &_markerFileName;

    // range of frames to average in static pose marker file, specified by time
    PropertyDblArray _timeRangeProp;
    Array<double> &_timeRange;

    // tasks used to specify IK weights
    PropertyObj _ikTaskSetProp;
    IKTaskSet &_ikTaskSet;

    // name of SIMM motion file that contains [optional] coordinates for the static pose
    PropertyStr _coordinateFileNameProp;
    std::string &_coordinateFileName;

    // name of XML model file to write when done placing markers
    PropertyStr _outputModelFileNameProp;
    std::string &_outputModelFileName;

    // name of marker file to write when done placing markers
    PropertyStr _outputMarkerFileNameProp;
    std::string &_outputMarkerFileName;

    // name of motion file (containing solved static pose) when done placing markers
    PropertyStr _outputMotionFileNameProp;
    std::string &_outputMotionFileName;

    // amount of allowable motion for each marker when averaging frames of the static trial
    PropertyDbl _maxMarkerMovementProp;
    double &_maxMarkerMovement;

    // Whether or not to write to the designated output files (GUI will set this to false)
    bool _printResultFiles;
    // Whether to move the model markers (set to false if you just want to preview the static pose)
    bool _moveModelMarkers;

    // This is cached during processModel() so the GUI can access it.
    mutable SimTK::ResetOnCopy<std::unique_ptr<Storage>> _outputStorage;
//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    MarkerPlacer();
    MarkerPlacer(const MarkerPlacer &aMarkerPlacementParams);
    virtual ~MarkerPlacer();

    void copyData(const MarkerPlacer &aMarkerPlacementParams);

#ifndef SWIG
    MarkerPlacer& operator=(const MarkerPlacer &aMarkerPlacementParams);
#endif
    bool processModel(Model* aModel,
            const std::string& aPathToSubject="") const;

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    bool getApply() const { return _apply; }
    void setApply(bool aApply) 
    { 
        _apply = aApply; 
        _applyProp.setValueIsDefault(false); 
    }

    const std::string &getStaticPoseFileName() const { return _markerFileName; }
    void setStaticPoseFileName(const std::string &aFileName) 
    {
        _markerFileName = aFileName;
        _markerFileNameProp.setValueIsDefault(false);
    }

   const Array<double> &getTimeRange() const { return _timeRange; }
   void setTimeRange(const Array<double> &timeRange) 
    {
        _timeRange = timeRange; 
        _timeRangeProp.setValueIsDefault(false); 
    }

    IKTaskSet &getIKTaskSet() { return _ikTaskSet; }

    const std::string &getCoordinateFileName() const { return _coordinateFileName; }
    void setCoordinateFileName(const std::string& aCoordinateFileName)
    {
        _coordinateFileName = aCoordinateFileName;
        _coordinateFileNameProp.setValueIsDefault(false);
    }

    const std::string& getMarkerFileName() const {return _markerFileName; }
    void setMarkerFileName( const std::string& aMarkerFileName)
    {
        _markerFileName=aMarkerFileName;
        _markerFileNameProp.setValueIsDefault(false);
    }

    double getMaxMarkerMovement() const { return _maxMarkerMovement; }
    void setMaxMarkerMovement(double aMaxMarkerMovement)
    {
        _maxMarkerMovement=aMaxMarkerMovement;
        _maxMarkerMovementProp.setValueIsDefault(false);
    }

    const std::string& getOutputModelFileName() const { return _outputModelFileName; }
    void setOutputModelFileName(const std::string& aOutputModelFileName)
    {
        _outputModelFileName = aOutputModelFileName;
        _outputModelFileNameProp.setValueIsDefault(false);
    }

    const std::string& getOutputMarkerFileName() const { return _outputMarkerFileName; }
    void setOutputMarkerFileName(const std::string& outputMarkerFileName)
    {
        _outputMarkerFileName = outputMarkerFileName;
        _outputMarkerFileNameProp.setValueIsDefault(false);
    }

    const std::string& getOutputMotionFileName() const { return _outputMotionFileName; }
    void setOutputMotionFileName(const std::string& outputMotionFileName)
    {
        _outputMotionFileName = outputMotionFileName;
        _outputMotionFileNameProp.setValueIsDefault(false);
    }

    void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

    bool getMoveModelMarkers() { return _moveModelMarkers; }
    void setMoveModelMarkers(bool aMove) { _moveModelMarkers = aMove; }

    Storage *getOutputStorage();


private:
    void setNull();
    void setupProperties();
    void moveModelMarkersToPose(SimTK::State& s, Model& aModel,
            MarkerData& aPose) const;
//=============================================================================
};  // END of class MarkerPlacer
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __MarkerPlacer_h__


