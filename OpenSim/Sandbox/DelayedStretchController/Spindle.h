#ifndef _Spindle_h_
#define _Spindle_h_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Spindle.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Ian Stavness, Mohammad Shabani, Chris Dembia,                   *
 *			  Shrinidhi K. Lakshmikanth, Ajay Seth, Thomas Uchida             *                                      *
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


#include <OpenSim/OpenSim.h>

namespace OpenSim {

class Spindle : public ModelComponent {
	OpenSim_DECLARE_CONCRETE_OBJECT(Spindle, ModelComponent);

public:
	// Property of the spindle for the input signal delay duration
	OpenSim_DECLARE_PROPERTY(delay_time, double,
		"Duration of delay (seconds).");

	// Input the muscle fiber length 
	OpenSim_DECLARE_INPUT(fiberLength, double, SimTK::Stage::Model,
		"The input fiber length measured from a muscle.");

	// Output the (delayed) length measured by the spindle.
	OpenSim_DECLARE_OUTPUT(length, double, getLength, SimTK::Stage::Position);

	Spindle() {
		constructProperties();
	}

	// Member function for getting the output (delayed) length
	double getLength(const SimTK::State& s) const {
		return getMemberSubcomponent<Delay>(_delayIdx).getValue(s);
	}
	

private:
	
	void constructProperties() {
		constructProperty_delay_time(0.0);
	}

	void extendFinalizeFromProperties() override {
		Super::extendFinalizeFromProperties();
		updMemberSubcomponent<Delay>(_delayIdx).set_delay(get_delay_time());
	}

	void extendConnectToModel(Model& model) override {
		Super::extendConnectToModel(model);

		// Wire the spindle input to the delay subcomponent input
		auto& input_channel = getInput<double>("fiberLength").getChannel();
		updMemberSubcomponent<Delay>(_delayIdx)
			.updInput("input").connect(input_channel, "length");
	}

	MemberSubcomponentIndex _delayIdx
	{ constructSubcomponent<Delay>("length_delay") };

}; // end of Spindle

} // end of namespace OpenSim

#endif // _Spindle_h_
