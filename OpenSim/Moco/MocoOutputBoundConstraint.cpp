//
// Created by Allison John on 7/24/24.
//

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
