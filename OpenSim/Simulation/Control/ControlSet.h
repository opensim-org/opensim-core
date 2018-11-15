#ifndef OPENSIM_CONTROL_SET_H_
#define OPENSIM_CONTROL_SET_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  ControlSet.h                           *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "Control.h"
#include <OpenSim/Common/Set.h>


//=============================================================================
//=============================================================================
namespace OpenSim { 

class ControlLinear;
class Storage;

/**
 * A class for holding and managing a set of controls for a dynamic
 * simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API ControlSet : public Set<Control> {
OpenSim_DECLARE_CONCRETE_OBJECT(ControlSet, Set<Control>);

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Map from parameters to controls. */
    Array<int> _ptcMap;
    /** Map from set parameters to control parameters. */
    Array<int> _ptpMap;


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ControlSet();
    ControlSet(const std::string &aFileName);
    ControlSet(const ControlSet &aSet);
    virtual ~ControlSet();

    // Constructor from a storage, useful for connecting result files to 
    // analyses that expect ControlSets. Default arguments 
    ControlSet(const Storage& aStorage, int nControlsToConvert=0, int aStartIndex=0);
private:
    void setNull();
    void setupProperties();
    OpenSim::ControlLinear* ExtractControl(const Storage& storage,int index);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    ControlSet& operator=(const ControlSet &aSet);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // SIZE
    int getSize(bool aForModelControls=true) const;
    // CONTROL LIST
    //int getControlIndex(const char *aName) const;
    void getControlList(const char *aType,Array<int> &rList,
        bool aForModelControls=true) const;
    // CONTROL VALUES
#ifndef SWIG
    void getControlValues(double aT,double rX[],
            bool aForModelControls=true) const;
#endif
    void getControlValues(double aT,Array<double> &rX,
            bool aForModelControls=true) const;
#ifndef SWIG
    void setControlValues(double aT,const double aX[],
            bool aForModelControls=true);
#endif
    void setControlValues(double aT,const Array<double> &aX,
            bool aForModelControls=true);
    // PARAMETERS
    int getNumParameters(bool aForModelControls=true) const;
    void getParameterList(Array<int> &rList,
            bool aForModelControls=true) const;
    void getParameterList(double aT,Array<int> &rList,
            bool aForModelControls=true) const;
    void getParameterList(double aTLower,double aTUpper,Array<int> &rList,
            bool aForModelControls=true) const;
    void getParameterMins(Array<double> &rMins,
            const Array<int> *aList=NULL) const;
    void getParameterMaxs(Array<double> &rMaxs,
            const Array<int> *aList=NULL) const;
#ifndef SWIG
    void getParameterValues(double rP[],
            const Array<int> *aList=NULL) const;
#endif
    void getParameterValues(Array<double> &rP,
            const Array<int> *aList=NULL) const;
#ifndef SWIG
    void setParameterValues(const double aP[],
            const Array<int> *aList=NULL);
#endif
    void setParameterValues(const Array<double> &aP,
            const Array<int> *aList=NULL);

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    void simplify(const PropertySet &aProperties);
    void filter(double aT);
    Storage*
        constructStorage(int aN,double aT1,double aT2,bool aForModelControls);
    int mapParameterToControl(int aIndex) const;
    int mapParameterToParameter(int aIndex) const;
    void generateParameterMaps();

//=============================================================================
};  // END of class ControlSet

}; //namespace
//=============================================================================
//=============================================================================


#endif // OPENSIM_CONTROL_SET_H_


