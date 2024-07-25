/* -------------------------------------------------------------------------- *
* OpenSim: MocoOutputBoundConstraint.h                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Allison John                                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MocoOutputBoundConstraint.h"
#include "MocoProblemInfo.h"
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using namespace OpenSim;

void MocoOutputBoundConstraint::constructProperties() {
    constructProperty_output_path("");
    constructProperty_second_output_path("");
    constructProperty_operation("");
    constructProperty_exponent(1);
    constructProperty_output_index(-1);
    constructProperty_lower_bound();
    constructProperty_upper_bound();
    constructProperty_equality_with_lower(false);
}

void MocoOutputBoundConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo& problemInfo) const {
    OPENSIM_THROW_IF_FRMOBJ(get_output_path().empty(), Exception,
            "No output_path provided.");
    std::string componentPath, outputName, channelName, alias;
    AbstractInput::parseConnecteePath(
            get_output_path(), componentPath, outputName, channelName, alias);
    const auto& component = getModel().getComponent(componentPath);
    const auto* abstractOutput = &component.getOutput(outputName);

    OPENSIM_THROW_IF_FRMOBJ(get_output_index() < -1, Exception,
            "Invalid Output index provided.");
    m_boundVectorNorm = (get_output_index() == -1);

    if (dynamic_cast<const Output<double>*>(abstractOutput)) {
        m_data_type = Type_double;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() != -1, Exception,
                "An Output index was provided, but the Output is of type 'double'.")

    } else if (dynamic_cast<const Output<SimTK::Vec3>*>(abstractOutput)) {
        m_data_type = Type_Vec3;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() > 2, Exception,
                "The Output is of type 'SimTK::Vec3', but an Output index greater "
                "than 2 was provided.");
        m_index1 = get_output_index();

    } else if (dynamic_cast<const Output<SimTK::SpatialVec>*>(abstractOutput)) {
        m_data_type = Type_SpatialVec;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() > 5, Exception,
                "The Output is of type 'SimTK::SpatialVec', but an Output index "
                "greater than 5 was provided.");
        if (get_output_index() < 3) {
            m_index1 = 0;
            m_index2 = get_output_index();
        } else {
            m_index1 = 1;
            m_index2 = get_output_index() - 3;
        }

    } else {
        OPENSIM_THROW_FRMOBJ(Exception,
                "Data type of specified model output not supported.");
    }
    m_output.reset(abstractOutput);

    OPENSIM_THROW_IF_FRMOBJ(get_exponent() < 1, Exception,
            "Exponent must be 1 or greater.");
    int exponent = get_exponent();

    // The pow() function gives slightly different results than x * x. On Mac,
    // using x * x requires fewer solver iterations.
    if (exponent == 1) {
        m_power_function = [](const double& x) { return x; };
    } else if (exponent == 2) {
        m_power_function = [](const double& x) { return x * x; };
    } else {
        m_power_function = [exponent](const double& x) {
            return pow(std::abs(x), exponent);
        };
    }

    // Set the "depends-on stage", the SimTK::Stage we must realize to
    // in order to calculate values from this output.
    m_dependsOnStage = m_output->getDependsOnStage();

    // initialize bounds
    m_hasLower = !getProperty_lower_bound().empty();
    m_hasUpper = !getProperty_upper_bound().empty();
    if (!m_hasLower && !m_hasUpper) {
        log_warn("In MocoOutputBoundConstraint '{}', output path(s) are "
                 "specified but no bounds are provided.", getName());
    }
    OPENSIM_THROW_IF_FRMOBJ(get_equality_with_lower() && m_hasUpper, Exception,
        "If equality_with_lower==true, upper bound function must not be "
        "set.");
    OPENSIM_THROW_IF_FRMOBJ(get_equality_with_lower() && !m_hasLower, Exception,
            "If equality_with_lower==true, lower bound function must be set.");

    auto checkTimeRange = [&](const Function& f) {
        if (auto* spline = dynamic_cast<const GCVSpline*>(&f)) {
            OPENSIM_THROW_IF_FRMOBJ(
                    spline->getMinX() > problemInfo.minInitialTime, Exception,
                    "The function's minimum domain value ({}) must "
                    "be less than or equal to the minimum possible "
                    "initial time ({}).",
                    spline->getMinX(), problemInfo.minInitialTime);
            OPENSIM_THROW_IF_FRMOBJ(
                    spline->getMaxX() < problemInfo.maxFinalTime, Exception,
                    "The function's maximum domain value ({}) must "
                    "be greater than or equal to the maximum possible "
                    "final time ({}).",
                    spline->getMaxX(), problemInfo.maxFinalTime);
        }
    };
    if (m_hasLower) checkTimeRange(get_lower_bound());
    if (m_hasUpper) checkTimeRange(get_upper_bound());

    if (get_equality_with_lower()) {
        setNumEquations(1);
    } else {
        setNumEquations((int)m_hasLower + (int)m_hasUpper);
    }

    // TODO: setConstraintInfo() is not really intended for use here.
    MocoConstraintInfo info;
    std::vector<MocoBounds> bounds;
    if (get_equality_with_lower()) {
        bounds.emplace_back(0, 0);
    } else {
        // The lower and upper bounds on the path constraint must be
        // constants, so we cannot use the lower and upper bound functions
        // directly. Therefore, we use a pair of constraints where the
        // lower/upper bound functions are part of the path constraint
        // functions and the lower/upper bounds for the path constraints are
        // -inf, 0, and/or inf.
        // If a lower bound function is provided, we enforce
        //      lower_bound_function <= output
        // by creating the constraint
        //      0 <= output - lower_bound_function <= inf
        if (m_hasLower) { bounds.emplace_back(0, SimTK::Infinity); }
        // If an upper bound function is provided, we enforce
        //      output <= upper_bound_function
        // by creating the constraint
        //      -inf <= output - upper_bound_function <= 0
        if (m_hasUpper) { bounds.emplace_back(-SimTK::Infinity, 0); }
    }
    info.setBounds(bounds);
    const_cast<MocoOutputBoundConstraint*>(this)->setConstraintInfo(info);

    m_useCompositeOutputValue = false;
    // if there's a second output, initialize it
    if (get_second_output_path() != "") {
        m_useCompositeOutputValue = true;
        initializeComposite();
    } else if (get_operation() != "") {
        OPENSIM_THROW_FRMOBJ(Exception, fmt::format("An operation was provided "
                "but a second Output path was not provided. Either provide no "
                "operation with a single Output, or provide a value to both "
                "setOperation() and setSecondOutputPath()."));
    }
}

void MocoOutputBoundConstraint::initializeComposite() const {
    if (get_operation() == "addition") {
        m_operation = Addition;
    } else if (get_operation() == "subtraction") {
        m_operation = Subtraction;
    } else if (get_operation() == "multiplication") {
        m_operation = Multiplication;
    } else if (get_operation() == "division") {
        m_operation = Division;
    } else if (get_operation() == "") {
        OPENSIM_THROW_FRMOBJ(Exception, fmt::format("A second Output path was "
                "provided, but no operation was provided. Use setOperation() to"
                "provide an operation"));
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, fmt::format("Invalid operation: '{}', must "
                "be 'addition', 'subtraction', 'multiplication', or 'division'.",
                get_operation()));
    }

    std::string componentPath, outputName, channelName, alias;
    AbstractInput::parseConnecteePath(get_second_output_path(), componentPath,
                                      outputName, channelName, alias);
    const auto& component = getModel().getComponent(componentPath);
    const auto* abstractOutput = &component.getOutput(outputName);

    if (dynamic_cast<const Output<double>*>(abstractOutput)) {
        OPENSIM_THROW_IF_FRMOBJ(getOutputIndex() != -1, Exception,
                "An Output index was provided, but the second Output is of type"
                " 'double'.")
        OPENSIM_THROW_IF_FRMOBJ(m_data_type != Type_double, Exception,
                "Output types do not match. The second Output is of type double"
                " but the first is of type {}.", getDataTypeString(m_data_type));
    } else if (dynamic_cast<const Output<SimTK::Vec3>*>(abstractOutput)) {
        OPENSIM_THROW_IF_FRMOBJ(m_data_type != Type_Vec3, Exception,
                "Output types do not match. The second Output is of type "
                "SimTK::Vec3 but the first is of type {}.",
                getDataTypeString(m_data_type));
    } else if (dynamic_cast<const Output<SimTK::SpatialVec>*>(abstractOutput)) {
        OPENSIM_THROW_IF_FRMOBJ(m_data_type != Type_SpatialVec, Exception,
                "Output types do not match. The second Output is of type "
                "SimTK::SpatialVec but the first is of type {}.",
                getDataTypeString(m_data_type));
        OPENSIM_THROW_IF_FRMOBJ(m_boundVectorNorm &&
                (m_operation == Multiplication || m_operation == Division),
                Exception, "Multiplication and division operations are not "
                "supported with Output type SimTK::SpatialVec without an index.")
    } else {
        OPENSIM_THROW_FRMOBJ(Exception,
                "Data type of specified second Output not supported.");
    }
    m_second_output.reset(abstractOutput);

    if (getDependsOnStage() < m_second_output->getDependsOnStage()) {
        m_dependsOnStage = m_second_output->getDependsOnStage();
    }
}

void MocoOutputBoundConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {
    int iconstr = 0;
    SimTK::Vector time(1);
    time[0] = state.getTime();
    double output_value = setValueToExponent(calcOutputValue(state));
    if (m_hasLower) {
        errors[iconstr++] = output_value - get_lower_bound().calcValue(time);
    }
    if (m_hasUpper) {
        errors[iconstr++] = output_value - get_upper_bound().calcValue(time);
    }
}

double MocoOutputBoundConstraint::calcOutputValue(const SimTK::State& state) const {
    if (m_useCompositeOutputValue) {
        return calcCompositeOutputValue(state);
    }
    return calcSingleOutputValue(state);
}

double MocoOutputBoundConstraint::calcSingleOutputValue(const SimTK::State& state) const {
    getModel().getSystem().realize(state, m_output->getDependsOnStage());

    double value = 0;
    if (m_data_type == Type_double) {
        value = static_cast<const Output<double>*>(m_output.get())
                        ->getValue(state);
    } else if (m_data_type == Type_Vec3) {
        if (m_boundVectorNorm) {
            value = static_cast<const Output<SimTK::Vec3>*>(m_output.get())
                        ->getValue(state).norm();
        } else {
            value = static_cast<const Output<SimTK::Vec3>*>(m_output.get())
                        ->getValue(state)[m_index1];
        }
    } else if (m_data_type == Type_SpatialVec) {
        if (m_boundVectorNorm) {
            value = static_cast<const Output<SimTK::SpatialVec>*>(m_output.get())
                        ->getValue(state).norm();
        } else {
            value = static_cast<const Output<SimTK::SpatialVec>*>(m_output.get())
                        ->getValue(state)[m_index1][m_index2];
        }
    }

    return value;
}

double MocoOutputBoundConstraint::calcCompositeOutputValue(const SimTK::State& state) const {
    getModel().getSystem().realize(state, getDependsOnStage());

    double value = 0;
    if (m_data_type == Type_double) {
        double value1 = getOutput<double>().getValue(state);
        double value2 = getSecondOutput<double>().getValue(state);
        value = applyOperation(value1, value2);
    } else if (m_data_type == Type_Vec3) {
        if (m_boundVectorNorm) {
            const SimTK::Vec3& value1 = getOutput<SimTK::Vec3>().getValue(state);
            const SimTK::Vec3& value2 = getSecondOutput<SimTK::Vec3>().getValue(state);
            value = applyOperation(value1, value2);
        } else {
            double value1 = getOutput<SimTK::Vec3>().getValue(state)[m_index1];
            double value2 = getSecondOutput<SimTK::Vec3>().getValue(state)[m_index1];
            value = applyOperation(value1, value2);
        }
    } else if (m_data_type == Type_SpatialVec) {
        if (m_boundVectorNorm) {
            const SimTK::SpatialVec& value1 = getOutput<SimTK::SpatialVec>()
                                              .getValue(state);
            const SimTK::SpatialVec& value2 = getSecondOutput<SimTK::SpatialVec>()
                                              .getValue(state);
            value = applyOperation(value1, value2);
        } else {
            double value1 = getOutput<SimTK::SpatialVec>().getValue(state)
                            [m_index1][m_index2];
            double value2 = getSecondOutput<SimTK::SpatialVec>().getValue(state)
                            [m_index1][m_index2];
            value = applyOperation(value1, value2);
        }
    }

    return value;
}
