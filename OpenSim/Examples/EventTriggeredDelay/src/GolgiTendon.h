#ifndef GOLGI_TENDON_H
#define GOLGI_TENDON_H

#include "BlockComponent.h"

namespace OpenSim {

/** 
* Implementation of Hook's Golgi tendon transfer function as suggested from the 
* bibliography
*
* @author Dimitar Stanev
*/
class GolgiTendon :public BlockComponent {
	OpenSim_DECLARE_CONCRETE_OBJECT(GolgiTendon, BlockComponent);

public:

	OpenSim_DECLARE_PROPERTY(threshold, double,
		"Cut off threshold value");

	GolgiTendon();

	GolgiTendon(const std::string& name);

	~GolgiTendon();

	double getThreshold() const;
	void setThreshold(const double t);

protected:

	virtual void computeStateVariableDerivatives(const SimTK::State& s)
		const override;
	
	virtual void extendAddToSystem(SimTK::MultibodySystem& system) 
		const override;

	virtual void calculateOutput(const SimTK::State& s, double& data)
		const override;

	virtual void setNull() override;

	virtual void constructProperties() override;

private:
	
	/**
	* State variable names
	*/
	static const std::string STATE_NAME_ONE;
	static const std::string STATE_NAME_TWO;
	static const std::string STATE_NAME_THREE;

	SimTK::Vector getTransferFunctionState(const SimTK::State& s) const;


}; // end of class 
}  // end of namespace OpenSim

#endif 
