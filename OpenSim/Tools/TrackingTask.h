#ifndef TrackingTask_h__
#define TrackingTask_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  TrackingTask.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
 * Contributor(s): Ayman Habib, Ajay Seth                                     *
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
#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDblArray.h>

namespace OpenSim {

class Function;
class Model;

//=============================================================================
//=============================================================================
/**
 * An abstract base class for specifying a target for a tracking problem.
 *
 * @author Ayman Habib & Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API TrackingTask : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(TrackingTask, Object);

//=============================================================================
// DATA
//=============================================================================
protected:
    // PROPERTIES
    /** Property to indicate on or off state. */
    PropertyBool _propOn;
    /** Weights of the task goals. */
    PropertyDblArray _propW;

    /** Reference to the value of the on property. */
    bool &_on;
    /** Reference to the value of the Weight property. */
    Array<double> &_w;

    /** Model. */
    Model *_model;

    /** Number of functions for this target. */
    int _nTrk;

    /** Position task functions.  Different types of tasks can 
    require different numbers of task functions.  For example, to track
    a joint angle, only one task function is needed.  However, to track
    a position, up to three task functions may be needed. */
    Function *_pTrk[3];
    /** Velocity task functions.  If velocity task functions are
    not specified, derivatives of the position task function are used. */
    Function *_vTrk[3];
    /** Acceleration task functions.  If acceleration task functions are
    not specified, derivatives of the position task function are used. */
    Function *_aTrk[3];

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    TrackingTask();
    TrackingTask(const TrackingTask &aTaskObject);
    virtual ~TrackingTask();

private:
    void setNull();
    void setupProperties();
    void copyData(const TrackingTask &aTaskObject);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:

#ifndef SWIG
    TrackingTask& operator=(const TrackingTask &aTaskObject);
#endif

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // MODEL
    virtual void setModel(OpenSim::Model& aModel);
    Model* getModel() const;

    // ON,OFF
    void setOn(bool aTrueFalse);
    bool getOn() const;
    // WEIGHTS
    void setWeight(double aW0,double aW1=0.0,double aW2=0.0);
    void setWeights(const Array<double>& aWeights) {_w = aWeights; };
    double getWeight(int aWhich) const;
    const Array<double>& getWeights() const { return _w; };
    // TASK FUNCTIONS
    int getNumTaskFunctions() const;
    virtual void setTaskFunctions(Function *aF0,
        Function *aF1=NULL,Function *aF2=NULL);
//=============================================================================
};  // END of class TrackingTask
//=============================================================================
//=============================================================================

}; // end namespace

#endif // __TrackingTask_h__


