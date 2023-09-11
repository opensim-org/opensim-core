#ifndef OPENSIM_FUNCTIONBASEDPATH_H
#define OPENSIM_FUNCTIONBASEDPATH_H
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  FunctionBasedPath.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include "OpenSim/Simulation/Model/AbstractPath.h"
#include "OpenSim/Common/Function.h"

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

//=============================================================================
//                           FUNCTION-BASED PATH
//=============================================================================
/**
 * A concrete class representing a path for muscles, ligaments, etc., based on 
 * OpenSim::Function objects.
 *
 * TODOs
 * - coordinates must be in the same order as the function arguments
 * - coordinates must be in the same order as the moment arm functions
 * - does *not* support joints with u =/= qdot (e.g., BallJoint)
 * - only applied mobility forces
 */
class OSIMSIMULATION_API FunctionBasedPath : public AbstractPath {
OpenSim_DECLARE_CONCRETE_OBJECT(FunctionBasedPath, AbstractPath);
    
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(coordinates, std::string, "");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(length_function, Function, "");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(speed_function, Function, "");
    OpenSim_DECLARE_LIST_PROPERTY(moment_arm_functions, Function, "");
    
//=============================================================================
// METHODS
//=============================================================================
    
    // CONSTRUCTION AND DESTRUCTION
    FunctionBasedPath();
    ~FunctionBasedPath() override;
    
    // GET AND SET
    void appendCoordinate(const std::string& coordinateName);
    void setCoordinates(const std::vector<std::string>& coordinateNames);
    // warning: freshly constructed from property list
    std::vector<std::string> getCoordinates() const;
    
    void setLengthFunction(const Function& lengthFunction);
    const Function& getLengthFunction() const;
    
    void setSpeedFunction(const Function& speedFunction);
    const Function& getSpeedFunction() const;
    
    void appendMomentArmFunction(const Function& momentArmFunction);
    void setMomentArmFunctions(const std::vector<Function>& momentArmFunctions);
    const Function& getMomentArmFunction(
            const std::string& coordinateName) const;
    
    // INTERFACE METHODS
    double getLength(const SimTK::State& s) const override;
    double getLengtheningSpeed(const SimTK::State& s) const override;
    double computeMomentArm(const SimTK::State& s, 
            const Coordinate& coord) const override;
    void addInEquivalentForces(const SimTK::State& state,
            const double& tension,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const override;
    bool isVisualPath() const override { return false; }
    
private:
    // ModelComponent interface.
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    
    // CONVENIENCE METHODS
    void constructProperties();
    void computeLength(const SimTK::State& s) const;
    void computeMomentArms(const SimTK::State& s) const;
    void computeLengtheningSpeed(const SimTK::State& s) const;
    
    // MEMBER VARIABLES
    std::vector<SimTK::ReferencePtr<const Coordinate>> _coordinates;
    std::map<std::string, int> _coordinateIndices;
    bool _computeSpeeds = false;
    bool _computeMomentArms = false;
    
    static const std::string LENGTH_NAME;
    static const std::string LENGTHENING_SPEED_NAME;
    static const std::string MOMENT_ARMS_NAME;
};

} // namespace OpenSim

#endif // OPENSIM_FUNCTIONBASEDPATH_H
