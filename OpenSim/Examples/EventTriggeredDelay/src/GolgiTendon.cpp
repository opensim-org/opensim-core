#include "GolgiTendon.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

const string GolgiTendon::STATE_NAME_ONE = "gto_state_one";
const string GolgiTendon::STATE_NAME_TWO = "gto_state_two";
const string GolgiTendon::STATE_NAME_THREE = "gto_state_three";


GolgiTendon::GolgiTendon() 
	: BlockComponent()
{
	setNull();
	constructProperties();
	setName("default_GolgiTendon");
}

GolgiTendon::GolgiTendon(const string& name) 
	: BlockComponent(name)
{
	setNull();
	constructProperties();
}

GolgiTendon::~GolgiTendon()
{

}

/************************************************************************/
/* Getters/setters                                                      */
/************************************************************************/

double GolgiTendon::getThreshold() const
{
	return get_threshold();
}

void GolgiTendon::setThreshold(const double t)
{
	set_threshold(t);
}

/************************************************************************/
/* Overload                                                             */
/************************************************************************/

void GolgiTendon::computeStateVariableDerivatives(const State& s) const
{
	double u = getInputValue<double>(s, BlockComponent::INPUT);

	Vector xdot(getNumStateVariables());
	Vector x = getTransferFunctionState(s);

	xdot[0] = -14.8 * x[2] + u;
	xdot[1] = x[0] - 81.8 * x[2];
	xdot[2] = x[1] - 39.2 * x[2];

	setStateVariableDerivativeValue(s, STATE_NAME_ONE, xdot[0]);
	setStateVariableDerivativeValue(s, STATE_NAME_TWO, xdot[1]);
	setStateVariableDerivativeValue(s, STATE_NAME_THREE, xdot[2]);
}

void GolgiTendon::extendAddToSystem(MultibodySystem& system) const
{
	Super::extendAddToSystem(system);

	addStateVariable(STATE_NAME_ONE, SimTK::Stage::Dynamics);
	addStateVariable(STATE_NAME_TWO, SimTK::Stage::Dynamics);
	addStateVariable(STATE_NAME_THREE, SimTK::Stage::Dynamics);
}

void GolgiTendon::calculateOutput(const State& s, double& data) const
{
	const double u = getInputValue<double>(s, BlockComponent::INPUT);

	Vector x = getTransferFunctionState(s);

	double y =
		get_gain() * -21.55 * x[0] +
		get_gain() * 789.6 * x[1] +
		get_gain() * -2.92e+04 * x[2] +
		get_gain() * u;

	//threshold function
	if (y <= get_threshold())
	{
		y = 0;
	}

	data = y;
}

void GolgiTendon::setNull()
{
	setAuthors("Dimitar Stanev");
}

void GolgiTendon::constructProperties()
{
	set_gain(1);//25
	set_delay(40E-3);
	set_type("GTO");

	constructProperty_threshold(0);
}

Vector GolgiTendon::getTransferFunctionState(const State& s) const
{
	Vector derivs(getNumStateVariables(), .0);

	derivs[0] = getStateVariableValue(s, STATE_NAME_ONE);
	derivs[1] = getStateVariableValue(s, STATE_NAME_TWO);
	derivs[2] = getStateVariableValue(s, STATE_NAME_THREE);

	return derivs;
}
