//
// Created by Allison John on 8/1/24.
//

#include "MocoParameterExpressionGoal.h"
#include <lepton/Parser.h>
#include <lepton/ParsedExpression.h>

using namespace OpenSim;

void MocoParameterExpressionGoal::constructProperties() {
    constructProperty_expression("");
    constructProperty_parameters();
}

void MocoParameterExpressionGoal::initializeOnModelImpl(const Model&) const {
    _parameterProg = Lepton::Parser::parse(get_expression()).optimize().createProgram();
}

void MocoParameterExpressionGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    integrand = calcOutputValue(input.state);
}

double MocoParameterExpressionGoal::calcOutputValue(const SimTK::State& state) const {

    std::map<std::string, double> parameterVars;
    for (int i=0; i<getProperty_variable_names().size(); ++i) {
        std::string variableName = getProperty_variable_names()[i];
        double parameterValue = getProperty_parameters()[i].getValue();
        parameterVars[variableName] = parameterValue;
    }
    return _parameterProg.evaluate(parameterVars);
}

void MocoParameterExpressionGoal::calcGoalImpl(
        const MocoGoal::GoalInput& input, SimTK::Vector& values) const {
    values[0] = input.integral;
}

