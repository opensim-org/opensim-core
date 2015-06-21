#ifndef DELAY_SIGNAL_H
#define DELAY_SIGNAL_H

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

class DelayEvenTrigger;

/**
* Implements a delay component which can track sudden changes to its input and
* informs the integrator.
*
* @author Dimitar Stanev
*/
class DelaySignal : public OpenSim::ModelComponent
{
	OpenSim_DECLARE_CONCRETE_OBJECT(DelaySignal, OpenSim::ModelComponent);

public:
	DelaySignal(double delay, int dependsOn);

	~DelaySignal();

	OpenSim_DECLARE_PROPERTY(delay, double,
		"Component delay");
	OpenSim_DECLARE_PROPERTY(stage_dependence, int,
		"Component input output stage dependence");

	static const std::string OUTPUT;
	static const std::string INPUT;


protected:

	double getValue(const SimTK::State& s) const;

	/**
	* It must be implemented in order to have access to subsystem and must be
	* called to initialize the state variables.
	*/
	virtual void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:

	SimTK::Measure_<double>::Delay m_delay;

	SimTK::ReferencePtr<DelayEvenTrigger> m_delay_trigger;

	/**
	* Initializes property values, do use the auto generated set functions.
	*/
	virtual void constructProperties();

	/**
	* Constructs output of the component
	*/
	virtual void constructInputs() const;

	/**
	* Constructs output of the component
	*/
	virtual void constructOutputs() const;

};

class DelayEvenTrigger : public SimTK::TriggeredEventHandler {

public:
	DelayEvenTrigger(OpenSim::DelaySignal* delay, SimTK::Stage dependsOn)
		: TriggeredEventHandler(dependsOn), m_delay_ref(delay)
	{
		getTriggerInfo().setTriggerOnRisingSignTransition(true);
		getTriggerInfo().setTriggerOnFallingSignTransition(false);
	}

	~DelayEvenTrigger(){};

protected:

	/**
	*	Checks if the input has changed and triggers an event.
	*/
	virtual SimTK::Real getValue(const SimTK::State& s) const override
	{
		if (m_delay_ref->getOutputValue<double>(s, "output") == 
			m_delay_ref->getInputValue<double>(s, "input"))
		{
			return -1;
		}
		else
		{
			return 1;
		}
	}

	/**
	*	Handles the event of changing in the input with respect to the output.
	*/
	virtual void handleEvent(
		SimTK::State& s, SimTK::Real accuracy, bool& shouldTerminate)
		const override
	{

	}

private:
	SimTK::ReferencePtr<OpenSim::DelaySignal> m_delay_ref;

}; // end of class 

/** A SimTK::Measure_ whose value is the value of an OpenSim::Input, and whose
* dependsOn SimTK::Stage is the connectAt stage of the OpenSim::Input. This
* Measure is useful for building OpenSim Components that use
* a SimTK::Measure_ internally.
*/
template <class T>
class InputMeasure : public SimTK::Measure_<T> {
public:
	SimTK_MEASURE_HANDLE_PREAMBLE(InputMeasure, SimTK::Measure_<T>);

	InputMeasure(SimTK::Subsystem& sub, const OpenSim::Input<T>& input)
		: SimTK::Measure_<T>(sub, new Implementation(input),
		SimTK::AbstractMeasure::SetHandle()) {}

	SimTK_MEASURE_HANDLE_POSTSCRIPT(InputMeasure, SimTK::Measure_<T>);
};

template <class T>
class InputMeasure<T>::Implementation :
public SimTK::Measure_<T>::Implementation{
public:
	Implementation(const OpenSim::Input<T>& input)
		: SimTK::Measure_<T>::Implementation(), m_input(input) {}

	Implementation* cloneVirtual() const override
	{
		return new Implementation(*this);
	}
	int getNumTimeDerivativesVirtual() const override { return 0; }
	SimTK::Stage getDependsOnStageVirtual(int order) const override
	{
		return m_input.getConnectAtStage();
	}

	// The meat of this class: return the Input's value.
	void calcCachedValueVirtual(const SimTK::State& s,
		int derivOrder, T& value) const override
	{
		SimTK_ASSERT1_ALWAYS(derivOrder == 0,
			"InputMeasure::Implementation::calcCachedValueVirtual(): "
			"derivOrder %d seen but only 0 allowed.", derivOrder);

		value = m_input.getValue(s);
	}

private:
	const OpenSim::Input<T>& m_input;
}; // END class InputMeasure<T>.


}; // end of namespace OpenSim

#endif