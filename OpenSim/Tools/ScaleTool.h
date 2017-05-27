#ifndef OPENSIM_SCALE_TOOL_H_ 
#define OPENSIM_SCALE_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  ScaleTool.h                            *
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
#include "osimToolsDLL.h"
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "ModelScaler.h"
#include "MarkerPlacer.h"

namespace OpenSim {

class GenericModelMaker;
class Model;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how to scale a model
 * to fit a subject, place markers on it, and do IK on one or more motion
 * trials.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API ScaleTool : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(ScaleTool, Object);

//=============================================================================
// DATA
//=============================================================================
protected:
    PropertyDbl _massProp;
    double &_mass;

    PropertyDbl _heightProp;
    double &_height;

    PropertyDbl _ageProp;
    double &_age;

    PropertyStr _notesProp;
    std::string &_notes;

    PropertyObj _genericModelMakerProp;
    GenericModelMaker &_genericModelMaker;

    PropertyObj _modelScalerProp;
    ModelScaler &_modelScaler;

    PropertyObj _markerPlacerProp;
    MarkerPlacer &_markerPlacer;

    /** All files in workflow are specified relative to
     * where the subject file is. Need to keep track of that in case absolute
     * path is needed later.
     */
    std::string  _pathToSubject;    

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ScaleTool();
    ScaleTool(const std::string &aFileName) SWIG_DECLARE_EXCEPTION;
    ScaleTool(const ScaleTool &aSubject);
    virtual ~ScaleTool();

#ifndef SWIG
    ScaleTool& operator=(const ScaleTool &aSubject);
#endif
    void copyData(const ScaleTool &aSubject);

    Model* createModel() const;
    /* Query the subject for different parameters */
    const GenericModelMaker& getGenericModelMaker() const
    { return _genericModelMaker; }

    const ModelScaler& getModelScaler() const
    { return _modelScaler; }

    const MarkerPlacer& getMarkerPlacer() const
    { return _markerPlacer; }

    /** Run the scale tool. This first runs the ModelScaler, then runs the
     * MarkerPlacer. This is the method called by the command line `scale`
     * executable. 
     * @returns whether or not the scale procedure was successful. */
    bool run() const;

    bool isDefaultGenericModelMaker() const
    { return _genericModelMakerProp.getValueIsDefault(); }
    bool isDefaultModelScaler() const
    { return _modelScalerProp.getValueIsDefault(); }
    bool isDefaultMarkerPlacer() const
    { return _markerPlacerProp.getValueIsDefault(); }

    /* Register types to be used when reading a ScaleTool object from xml file. */
    static void registerTypes();

    /** Accessor methods to obtain model attributes */
    double getSubjectMass() const { return _mass; }
    double getSubjectAge() const { return _age; }
    double getSubjectHeight() const { return _height; }
    void setSubjectMass(double mass) { _mass = mass; }
    void setSubjectAge(double age) { _age = age; }
    void setSubjectHeight(double height) { _height = height; }
    /**
     * Accessor methods to set and get path to Subject. This is needed
     * since all file names referred to in the subject file are relative
     * to the subject file.
     */
    const std::string& getPathToSubject() const
    {
        return _pathToSubject;
    }
    void setPathToSubject(const std::string& aPath)
    {
        _pathToSubject=aPath;
    }
    //std::string getParentDirectory(const std::string& fileName);
    
    void setPrintResultFiles(bool aToWrite) { 
        _modelScaler.setPrintResultFiles(aToWrite);
        _markerPlacer.setPrintResultFiles(aToWrite);
    }

protected:

private:
    void setNull();
    void setupProperties();
//=============================================================================
};  // END of class ScaleTool
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SCALE_TOOL_H_
