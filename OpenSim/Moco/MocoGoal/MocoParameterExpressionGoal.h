//
// Created by Allison John on 8/1/24.
//

#ifndef OPENSIM_MOCOPARAMETEREXPRESSIONGOAL_H
#define OPENSIM_MOCOPARAMETEREXPRESSIONGOAL_H

#include "MocoGoal.h"
#include <lepton/ExpressionProgram.h>

namespace OpenSim {

class OSIMMOCO_API MocoParameterExpressionGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoParameterExpressionGoal, MocoGoal);

public:
    MocoParameterExpressionGoal() { constructProperties(); }
    MocoParameterExpressionGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoParameterExpressionGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    void setExpression(std::string& expression) {
        set_expression(expression);
    }

    void addParameter(MocoParameter& parameter, std::string variableName) {
        updProperty_parameters().appendValue(parameter);
        updProperty_variable_names().appendValue(variableName);
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    //void printDescriptionImpl() const override;

private:
    void constructProperties();
    /** Calculate the Output value for the provided SimTK::State. Do not
    call this function until 'initializeOnModelBase()' has been called. */
    double calcOutputValue(const SimTK::State&) const;
    OpenSim_DECLARE_PROPERTY(expression, std::string,
            "The expression string with variables q0-q9.");
    // consider a mapping from variable names to parameters instead
    OpenSim_DECLARE_LIST_PROPERTY(parameters, MocoParameter,
            "MocoParameters to use in the expression.");
    OpenSim_DECLARE_LIST_PROPERTY(variable_names, std::string,
            "Variable names of the MocoParameters to use in the expression.");

    mutable Lepton::ExpressionProgram _parameterProg;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOPARAMETEREXPRESSIONGOAL_H
