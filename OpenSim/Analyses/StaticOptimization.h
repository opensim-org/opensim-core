#ifndef _StaticOptimization_h_
#define _StaticOptimization_h_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StaticOptimization.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt                                             *
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
#include <memory>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include "ForceReporter.h"

//=============================================================================
//=============================================================================
/**
 */
namespace OpenSim { 

class Model;
class ForceSet;

/**
 * This class implements static optimization to compute Muscle Forces and 
 * activations. 
 *
 * @author Jeff Reinbolt
 */
class OSIMANALYSES_API StaticOptimization : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(StaticOptimization, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:
    int _numCoordinateActuators;

    std::unique_ptr<ForceReporter> _forceReporter;

protected:
    /** Use force set from model. */
    PropertyBool _useModelForceSetProp;
    bool &_useModelForceSet;

    PropertyDbl _activationExponentProp;
    double &_activationExponent;

    PropertyBool _useMusclePhysiologyProp;
    bool    &_useMusclePhysiology;

    PropertyDbl _convergenceCriterionProp;
    double &_convergenceCriterion;

    PropertyInt _maximumIterationsProp;
    int &_maximumIterations;

    Storage *_activationStorage;
    Storage *_forceStorage;
    GCVSplineSet _statesSplineSet;

    Array<int> _accelerationIndices;

    SimTK::Vector _parameters;

    bool _ownsForceSet;
    ForceSet* _forceSet;

    double _numericalDerivativeStepSize;
    std::string _optimizerAlgorithm;
    int _printLevel;

    Model *_modelWorkingCopy;

//=============================================================================
// METHODS
//=============================================================================
public:
    StaticOptimization(Model *aModel=0);
    StaticOptimization(const StaticOptimization &aObject);
    virtual ~StaticOptimization();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    StaticOptimization& operator=(const StaticOptimization &aStaticOptimization);
#endif
private:
    void setNull();
    void setupProperties();
    void constructDescription();
    void constructColumnLabels();
    void allocateStorage();
    void deleteStorage();

public:
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setStorageCapacityIncrements(int aIncrement);
    Storage* getActivationStorage();
    Storage* getForceStorage();

    bool getUseModelForceSet() { return _useModelForceSet; }
    void setUseModelForceSet(bool aUseModelActuatorSet) { _useModelForceSet = aUseModelActuatorSet; }

    void setModel(Model& aModel) override;
    void setActivationExponent(const double aExponent) { _activationExponent=aExponent; }
    double getActivationExponent() const { return _activationExponent; }
    void setUseMusclePhysiology(const bool useIt) { _useMusclePhysiology=useIt; }
    bool getUseMusclePhysiology() const { return _useMusclePhysiology; }
    void setConvergenceCriterion(const double tolerance) { _convergenceCriterion = tolerance; }
    double getConvergenceCriterion() { return _convergenceCriterion; }
    void setMaxIterations( const int maxIt) { _maximumIterations = maxIt; }
    int getMaxIterations() {return _maximumIterations; }
    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------
    int
        begin(const SimTK::State& s ) override;
    int
        step(const SimTK::State& s, int setNumber ) override;
    int
        end(const SimTK::State& s ) override;
protected:
    virtual int
        record(const SimTK::State& s );
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

//=============================================================================
};  // END of class StaticOptimization

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __StaticOptimization_h__
