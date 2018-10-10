#ifndef _InverseDynamics_h_
#define _InverseDynamics_h_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  InverseDynamics.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Eran Guendelman                                                 *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimAnalysesDLL.h"
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Common/GCVSplineSet.h>


//=============================================================================
//=============================================================================
/**
 */
namespace OpenSim { 

class Model;
class ForceSet;

/** @cond **/ // hide from Doxygen

/**
 * A class for performing and recording Inverse Dynamics forces/moments
 * on a motion trajectory.
 *
 * @author Eran
 */
class OSIMANALYSES_API InverseDynamics : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(InverseDynamics, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:
    int _numCoordinateActuators;
protected:
    /** Use force set from model. */
    PropertyBool _useModelForceSetProp;
    bool &_useModelForceSet;

    Storage *_storage;
    GCVSplineSet _statesSplineSet;

    Array<double> _dydt;
    Array<int> _accelerationIndices;

    bool _ownsForceSet;
    ForceSet *_forceSet;

    SimTK::Matrix _performanceMatrix;
    SimTK::Vector _performanceVector;
    SimTK::Matrix _constraintMatrix;
    SimTK::Vector _constraintVector;
    SimTK::Vector _lapackWork;

    Model *_modelWorkingCopy;

//=============================================================================
// METHODS
//=============================================================================
public:
    InverseDynamics(Model *aModel=0);
    InverseDynamics(const InverseDynamics &aObject);
    virtual ~InverseDynamics();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    InverseDynamics& operator=(const InverseDynamics &aInverseDynamics);
#endif
private:
    void setNull();
    void setupProperties();
    void constructDescription();
    void constructColumnLabels();
    void allocateStorage();
    void deleteStorage();
    void computeAcceleration(SimTK::State& s, double *aF,double *rAccel) const;

public:
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setStorageCapacityIncrements(int aIncrement);
    Storage* getStorage();

    bool getUseModelForceSet() { return _useModelForceSet; }
    void setUseModelForceSet(bool aUseModelForceSet) { _useModelForceSet = aUseModelForceSet; }

    void setModel(Model& aModel) override;
    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------
#ifndef SWIG
    int
        begin(const SimTK::State& s ) override;
    int
        step(const SimTK::State& s, int setNumber ) override;
    int
        end(const SimTK::State& s ) override;
protected:
    virtual int
        record(const SimTK::State& s );
#endif  
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

//=============================================================================
};  // END of class InverseDynamics

/** @endcond **/

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __InverseDynamics_h__
