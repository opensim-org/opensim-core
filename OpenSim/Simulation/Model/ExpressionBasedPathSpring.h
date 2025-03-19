#ifndef OPENSIM_EXPRESSION_BASED_PATH_SPRING_H_
#define OPENSIM_EXPRESSION_BASED_PATH_SPRING_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ExpressionBasedPathSpring.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
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

#include <OpenSim/Simulation/Model/AbstractGeometryPath.h>
#include <OpenSim/Simulation/Model/ForceProducer.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>

namespace OpenSim {

class ScaleSet;

/**
 * A massless, spring-based path Force whose force magnitude is determined by a 
 * user-defined expression, with the path stretch (`s = l-l0`) and the path 
 * speed (`ldot`) as variables, where `l` is the path length and `l0` is the 
 * rest length. The path of the ExpressionBasedPathSpring is determined by an 
 * object derived from AbstractGeometryPath. 
 *
 * "s" and "ldot" are the variables names expected by the expression parser.
 * Common C math library functions such as: exp(), pow(), sqrt(), sin(), ...
 * are permitted. See Lepton/Operation.h for a complete list.
 *
 * For example: string expression = "-1.5*exp(10*(d-0.25)^2)*(1 + 2.0*ddot)"
 *              provides a model of a path spring with non-linear stiffness 
 *              and damping.
 */
class OSIMSIMULATION_API ExpressionBasedPathSpring : public ForceProducer {
OpenSim_DECLARE_CONCRETE_OBJECT(ExpressionBasedPathSpring, ForceProducer);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(resting_length, double,
        "The resting length (m) of the ExpressionBasedPathSpring. Default: 0.");
    OpenSim_DECLARE_PROPERTY(expression, std::string,
        "Expression of the path spring magnitude as a function of "
        "the path stretch (s) and the path speed (ldot). "
        "Note, expression cannot have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(path, AbstractGeometryPath,
        "The path defines the length and lengthening speed of the "
        "ExpressionBasedPathSpring");

//=============================================================================
// OUTPUTS
//=============================================================================
    OpenSim_DECLARE_OUTPUT(length, double, getLength,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(stretch, double, getStretch,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(lengthening_speed, double, getLengtheningSpeed,
        SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(tension, double, getTension,
        SimTK::Stage::Dynamics);

//==============================================================================
// METHODS
//==============================================================================
    
    /** Default constructor. */
    ExpressionBasedPathSpring();

    /** Convenience constructor.
     * @param name          name of this %ExpressionBasedPathSpring Object
     * @param restLength    the spring's resting length
     * @param expression    the expression used to compute spring tension */
     ExpressionBasedPathSpring(const std::string& name, 
        double restLength, double stiffness, double dissipation);


    /** 
     * The resting length of the path spring. The default value is 0. 
     */
    double getRestingLength() const {   
        return get_resting_length();
    }
    /// @copydoc getRestingLength()
    void setRestingLength(double restingLength) {
        set_resting_length(restingLength);
    }

    /** 
     * The path object. 
     */
    AbstractGeometryPath& updPath() { return upd_path(); }
    const AbstractGeometryPath& getPath() const { return get_path(); }

    template <typename PathType>
    PathType& updPath() {
        return dynamic_cast<PathType&>(upd_path());
    }
    template <typename PathType>
    const PathType& getPath() const {
        return dynamic_cast<const PathType&>(get_path());
    }

    template <typename PathType>
    PathType* tryUpdPath() {
        return dynamic_cast<PathType*>(&upd_path());
    }
    template <typename PathType>
    const PathType* tryGetPath() const {
        return dynamic_cast<const PathType*>(&get_path());
    }

    GeometryPath& updGeometryPath() {
        return updPath<GeometryPath>();
    }
    const GeometryPath& getGeometryPath() const {
        return getPath<GeometryPath>();
    }
    
    bool hasVisualPath() const override { return getPath().isVisualPath(); };

    /** 
     * Get the length of the ExpressionBasedPathSpring. 
     * 
     * @note Accessible at SimTK::Stage::Position. 
     */
    double getLength(const SimTK::State& s) const;

    /** 
     * Get the stretch in the ExpressionBasedPathSpring. 
     * 
     * @note Accessible at SimTK::Stage::Position. 
     */
    double getStretch(const SimTK::State& s) const;

    /** 
     * Get the lengthening speed of the ExpressionBasedPathSpring. 
     * 
     * @note Accessible at SimTK::Stage::Velocity. 
     */
    double getLengtheningSpeed(const SimTK::State& s) const;

    /** 
     * Get the tension generated by the ExpressionBasedPathSpring. 
     * 
     * @note The value of the tension can only be obtained after the system has
     * been realized to SimTK::Stage::Dynamics. 
     */
    double getTension(const SimTK::State& s) const;

    // COMPUTATIONS
    /** 
     * Compute the moment arm of the path with respect to a given coordinate.
     */
    double computeMomentArm(const SimTK::State& s, 
            const Coordinate& aCoord) const;

=    // SCALE
    /** Adjust the resting length of the path spring after the model has been
     * scaled. The `resting_length` property is multiplied by the quotient of
     * the current path length and the path length before scaling. 
     */
    void extendPostScale(const SimTK::State& s,
                         const ScaleSet& scaleSet) override;

protected:
    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;

    // FORCE INTERFACE
    OpenSim::Array<std::string> getRecordLabels() const override;
    OpenSim::Array<double> getRecordValues(
            const SimTK::State& state) const override;

private:
    // FORCE PRODUCER INTERFACE
    void implProduceForces(const SimTK::State&, ForceConsumer&) const override;

    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_EXPRESSION_BASED_PATH_SPRING_H_
