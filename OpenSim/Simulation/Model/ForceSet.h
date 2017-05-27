#ifndef OPENSIM_FORCE_SET_H_
#define OPENSIM_FORCE_SET_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  ForceSet.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Jack Middleton                                       *
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
#include "ModelComponentSet.h"
#include "Muscle.h"

namespace OpenSim {

class Actuator;
class Force;
class Model;

//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of forces for a model.
 * This class is based on ModelComponentSet
 *
 * @authors Ajay Seth, Jack Middleton 
 * @version 1.0
 */

//=============================================================================
class OSIMSIMULATION_API ForceSet : public ModelComponentSet<Force> {
OpenSim_DECLARE_CONCRETE_OBJECT(ForceSet, ModelComponentSet<Force>);

//=============================================================================
// DATA
//=============================================================================
protected:

   /** The subset of Forces that are Actuators. */
    Set<Actuator> _actuators;

    /** The subset of Forces that are Muscles. */
    Set<Muscle> _muscles;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ForceSet();
    ForceSet(Model& model);
    ForceSet(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode = true);
    ForceSet(const ForceSet &aForceSet);
    virtual ~ForceSet();

private:
    void setNull();
    void setupSerializedMembers();
    void updateActuators();
    void updateMuscles();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    ForceSet& operator=(const ForceSet &aSet);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    // Override ModelComponentSet method.
    void invokeConnectToModel(Model& aModel) override;

    // FORCE
    bool remove(int aIndex) override;
    bool append(Force *aForce);
#ifndef SWIG
    bool append(Force &aForce);
#endif
    bool append(ForceSet &aForceSet, bool aAllowDuplicateNames=false);
    bool set(int aIndex, Force *aForce);
    bool insert(int aIndex, Force *aObject) override;

    // subsets 
    const Set<Actuator>& getActuators() const;
    Set<Actuator>& updActuators();
    const Set<Muscle>& getMuscles() const;
    Set<Muscle>& updMuscles();

    // STATES
    void getStateVariableNames(Array<std::string> &rNames) const;


    //--------------------------------------------------------------------------
    // CHECK
    //--------------------------------------------------------------------------
    bool check() const;

//=============================================================================
};  // END of class ForceSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // OPENSIM_FORCE_SET_H_


