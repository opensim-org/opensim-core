/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoOutputGoal.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2022 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include "MocoOutputGoal.h"
#include "OpenSim/Common/Logger.h"

using namespace OpenSim;

// ============================================================================
// MocoOutputBase
// ============================================================================

void MocoOutputBase::constructProperties() {
    constructProperty_output_path("");
    constructProperty_second_output_path("");
    constructProperty_operation("");
    constructProperty_exponent(1);
    constructProperty_output_index(-1);
}

void MocoOutputBase::initializeOnModelBase() const {
    OPENSIM_THROW_IF_FRMOBJ(get_output_path().empty(), Exception,
            "No output_path provided.");
    std::string componentPath, outputName, channelName, alias;
    AbstractInput::parseConnecteePath(
            get_output_path(), componentPath, outputName, channelName, alias);
    const auto& component = getModel().getComponent(componentPath);
    const auto* abstractOutput = &component.getOutput(outputName);

    OPENSIM_THROW_IF_FRMOBJ(get_output_index() < -1, Exception,
            "Invalid Output index provided.");
    m_minimizeVectorNorm = (get_output_index() == -1);

    if (dynamic_cast<const Output<double>*>(abstractOutput)) {
        m_data_type = Type_double;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() != -1, Exception,
                "An Output index was provided, but the Output is of type "
                "'double'.")

    } else if (dynamic_cast<const Output<SimTK::Vec3>*>(abstractOutput)) {
        m_data_type = Type_Vec3;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() > 2, Exception,
                "The Output is of type 'SimTK::Vec3', but an Output index "
                "greater than 2 was provided.");
        m_index1 = get_output_index();

    } else if (dynamic_cast<const Output<SimTK::SpatialVec>*>(abstractOutput)) {
        m_data_type = Type_SpatialVec;
        OPENSIM_THROW_IF_FRMOBJ(get_output_index() > 5, Exception,
                "The Output is of type 'SimTK::SpatialVec', but an Output index"
                " greater than 5 was provided.");
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

void MocoOutputBase::initializeComposite() const {
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
        OPENSIM_THROW_IF_FRMOBJ(m_minimizeVectorNorm &&
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

double MocoOutputBase::calcOutputValue(const SimTK::State& state) const {
    if (m_useCompositeOutputValue) {
        return calcCompositeOutputValue(state);
    }
    return calcSingleOutputValue(state);
}

double MocoOutputBase::calcSingleOutputValue(const SimTK::State& state) const {
    getModel().getSystem().realize(state, getDependsOnStage());

    double value = 0;
    if (m_data_type == Type_double) {
        value = getOutput<double>().getValue(state);
    } else if (m_data_type == Type_Vec3) {
        if (m_minimizeVectorNorm) {
            value = getOutput<SimTK::Vec3>().getValue(state).norm();
        } else {
            value = getOutput<SimTK::Vec3>().getValue(state)[m_index1];
        }
    } else if (m_data_type == Type_SpatialVec) {
        if (m_minimizeVectorNorm) {
            value = getOutput<SimTK::SpatialVec>().getValue(state).norm();
        } else {
            value = getOutput<SimTK::SpatialVec>().getValue(state)[m_index1][m_index2];
        }
    }

    return value;
}

double MocoOutputBase::calcCompositeOutputValue(const SimTK::State& state) const {
    getModel().getSystem().realize(state, getDependsOnStage());

    double value = 0;
    if (m_data_type == Type_double) {
        double value1 = getOutput<double>().getValue(state);
        double value2 = getSecondOutput<double>().getValue(state);
        value = applyOperation(value1, value2);
    } else if (m_data_type == Type_Vec3) {
        if (m_minimizeVectorNorm) {
            const SimTK::Vec3& value1 = getOutput<SimTK::Vec3>().getValue(state);
            const SimTK::Vec3& value2 = getSecondOutput<SimTK::Vec3>().getValue(state);
            value = applyOperation(value1, value2);
        } else {
            double value1 = getOutput<SimTK::Vec3>().getValue(state)[m_index1];
            double value2 = getSecondOutput<SimTK::Vec3>().getValue(state)[m_index1];
            value = applyOperation(value1, value2);
        }
    } else if (m_data_type == Type_SpatialVec) {
        if (m_minimizeVectorNorm) {
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

void MocoOutputBase::printDescriptionImpl() const {
    // Output path.
    std::string str = fmt::format("        output: {}", getOutputPath());

    if (m_useCompositeOutputValue) {
        str += fmt::format("\n        second output: {}", getSecondOutputPath());
        // Operation.
        str += fmt::format("\n        operation: {}", get_operation());
    }

    // Output type.
    str += fmt::format(", type: {}", getDataTypeString(m_data_type));

    // Output index (if relevant).
    if (getOutputIndex() != -1) {
        str += fmt::format(", index: {}", getOutputIndex());
    }

    // Exponent.
    str += fmt::format(", exponent: {}", getExponent());

    // Bounds (if endpoint constraint).
    if (getModeIsEndpointConstraint()) {
        str += fmt::format(", bounds: {}", getConstraintInfo().getBounds()[0]);
    }

    log_info(str);
}

// ============================================================================
// MocoOutputGoal
// ============================================================================

void MocoOutputGoal::initializeOnModelImpl(const Model& output) const {
    initializeOnModelBase();
    setRequirements(1, 1, getDependsOnStage());
}

void MocoOutputGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    integrand = setValueToExponent(calcOutputValue(input.state));
}

void MocoOutputGoal::calcGoalImpl(
        const MocoGoal::GoalInput& input, SimTK::Vector& values) const {
    values[0] = input.integral;
}

// ============================================================================
// MocoOutputExtremumGoal
// ============================================================================

void MocoOutputExtremumGoal::constructProperties() {
    constructProperty_extremum_type("minimum");
    constructProperty_smoothing_factor(1);
}

void MocoOutputExtremumGoal::initializeOnModelImpl(const Model& output) const {
    initializeOnModelBase();

    OPENSIM_THROW_IF_FRMOBJ(
            get_extremum_type() != "minimum" 
                && get_extremum_type() != "maximum",
            Exception, "The extremum type must be either 'minimum' " 
            " or 'maximum'.");

    OPENSIM_THROW_IF_FRMOBJ(
            get_smoothing_factor() < 0.2 || get_smoothing_factor() > 1.0,
            Exception,
            fmt::format("Expected the smoothing factor must be in the range "
                    "[0.2, 1.0], but received {}.", get_smoothing_factor()));

    OPENSIM_THROW_IF_FRMOBJ(m_minimizeVectorNorm == 1 &&
                                        m_data_type == Type_Vec3,
            Exception, "The MocoOutputExtremumGoal cannot be used when "
            "taking the norm of an output of SimTK::Vec3 type. Use the "
            "MocoOutputGoal instead.");

    OPENSIM_THROW_IF_FRMOBJ(
             m_minimizeVectorNorm == 1 && m_data_type == Type_SpatialVec,
             Exception,
             "The MocoOutputExtremumGoal cannot be used when "
             "taking the norm of an output of SimTK::SpatialVec type. Use the "
             "MocoOutputGoal instead.");
    
    if (get_extremum_type() == "minimum") { 
        m_beta = -1;
    } else if (get_extremum_type() == "maximum") {
        m_beta = 1;
    };

    setRequirements(1, 1, getDependsOnStage());
}

void MocoOutputExtremumGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    double integrand_temp =
            (1 / get_smoothing_factor()) *
            (std::log(1 + exp(get_smoothing_factor() * m_beta *
                                  calcOutputValue(input.state))));
    integrand = m_beta*setValueToExponent(integrand_temp);
}

void MocoOutputExtremumGoal::calcGoalImpl(
        const MocoGoal::GoalInput& input, SimTK::Vector& values) const {
    values[0] = input.integral;
}

// ============================================================================
// MocoOutputTrackingGoal
// ============================================================================

void MocoOutputTrackingGoal::constructProperties() {
    constructProperty_tracking_function();
}

void MocoOutputTrackingGoal::initializeOnModelImpl(const Model& output) const {
    initializeOnModelBase();
    setRequirements(1, 1, getDependsOnStage());
}

void MocoOutputTrackingGoal::calcIntegrandImpl(const IntegrandInput& input,
        double& integrand) const {
    SimTK::Vector time(1, input.state.getTime());
    integrand = setValueToExponent(calcOutputValue(input.state) -
                                   get_tracking_function().calcValue(time));
}

void MocoOutputTrackingGoal::calcGoalImpl(const GoalInput &input,
        SimTK::Vector &values) const {
    values[0] = input.integral;
}
