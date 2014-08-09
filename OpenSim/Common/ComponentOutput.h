#ifndef OPENSIM_COMPONENT_OUTPUT_H_
#define OPENSIM_COMPONENT_OUTPUT_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ComponentOutput.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

/** @file
 * This file defines the Output class, which formalizes an output (signal)
 * that a Component produces. This can be the tension in a force element, the
 * location of a body, metabolic energy consumption of a model, etc...
 * It is the obligation of the Component to define its outputs.
 */

// INCLUDES
#include "OpenSim/Common/Component.h"
#include <functional>

namespace OpenSim {

//=============================================================================
//                           OPENSIM COMPONENT OUTPUT
//=============================================================================
/**
 * Output formalizes the accesss to a value of interest computed by the
 * owning Component. The value is then exposed and easily accessible for use by
 * other components (e.g. to satisfy an Input, @see Input).
 * The purpose of an Output is to bind a value of interest to a component's
 * member function (generator), and provide a generic interface to the value,
 * its type and label so it can be easily identified. It also specifies the
 * realization (computational) stage at which the value is valid, so that a
 * caller can provide adequate error handling.
 *
 * For example, a Body can have its position transormation with respect to
 * ground as an Output, which is only accessible when the model has been
 * realized to the Position stage or beyond, in which case it depends on the
 * Position stage. The validity of data flow can be checked prior to
 * initiating a simulation.
 *
 * An Output is intended to lightweight and adds no computational overhead
 * if the output goes unused. When an Output's value is called upon,
 * the overhead is a single redirect to the corresponding member function
 * for the value.
 *
 * @author  Ajay Seth
 */

/**
* Abstract base class representing an arbitrary Output as self-describing type.
*/
class OSIMCOMMON_API AbstractOutput {
public:
    AbstractOutput() : numSigFigs(8), dependsOnStage(SimTK::Stage::Infinity) {}
    AbstractOutput(const std::string& name, SimTK::Stage dependsOnStage) :
        name(name), dependsOnStage(dependsOnStage), numSigFigs(8) {}
    virtual ~AbstractOutput() { }

    /** Output's name */
    const std::string& getName() const {
        return name;
    }
    /** Output's dependence on System being realized to at least this System::Stage */
    const SimTK::Stage& getDependsOnStage() const {
        return dependsOnStage;
    }

    /** Output Interface */
    virtual std::string     getTypeName() const = 0;
    virtual std::string		getValueAsString(const SimTK::State& state) const = 0;
    virtual bool        isCompatible(const AbstractOutput&) const = 0;
    virtual void compatibleAssign(const AbstractOutput&) = 0;

    AbstractOutput& operator=(const AbstractOutput& o)
    {
        compatibleAssign(o);
        return *this;
    }

    virtual AbstractOutput* clone() const = 0;

    /** Specification for number of significant figures in string value. */
    unsigned int getNumberOfSignificantDigits() const {
        return numSigFigs;
    }
    void		 setNumberOfSignificantDigits(unsigned int numSigFigs)
    {
        numSigFigs = numSigFigs;
    }

private:
    unsigned int numSigFigs;
    SimTK::Stage dependsOnStage;
    std::string name;
//=============================================================================
};	// END class AbstractOutput

template<class T>
class  Output : public AbstractOutput {
public:
    //default construct output function pointer and result container
    Output() : AbstractOutput(), _outputFcn(nullptr) {}
    /** Convenience constructor
    Create a Component::Output bound to a specific method of the Component and
    valid at a given realization Stage.
    @param name             The name of the output.
    @param outputFunction	The output function to be invoked (returns Output T)
    @param dependsOnStage	Stage at which Output can be evaluated. */
    explicit Output(const std::string& name,
                    const std::function<T(const SimTK::State&)> outputFunction,
                    const SimTK::Stage&		dependsOnStage) :
        AbstractOutput(name, dependsOnStage), _outputFcn(outputFunction)
    {}

    virtual ~Output() {}

    bool isCompatible(const AbstractOutput& o) const override {
        return isA(o);
    }
    void compatibleAssign(const AbstractOutput& o) override {
        if (!isA(o))
            SimTK_THROW2(SimTK::Exception::IncompatibleValues, o.getTypeName(), getTypeName());
        *this = downcast(o);
    }


    //--------------------------------------------------------------------------
    // OUTPUT VALUE
    //--------------------------------------------------------------------------
    /** return the Value of this ouput if the state is appropriately realized
        to a stage at our beyond the dependsOnStage, otherwise expect an
    	Exception. */
    const T& getValue(const SimTK::State& state) const {
        _result = SimTK::NaN;
        if (state.getSystemStage() < getDependsOnStage())
        {
            throw SimTK::Exception::StageTooLow(__FILE__, __LINE__,
                                                state.getSystemStage(), getDependsOnStage(),
                                                "Output::getValue(state)");
        }
        _result = _outputFcn(state);
        return _result;
    }

    /** determine the value type for this Output*/
    std::string getTypeName() const override
    {
        return SimTK::NiceTypeName<T>::name();
    }

    std::string	getValueAsString(const SimTK::State& state) const override
    {
        unsigned int ns = getNumberOfSignificantDigits();
        std::stringstream s;
        s << std::setprecision(ns) << getValue(state);
        return s.str();
    }

    AbstractOutput* clone() const override {
        return new Output(*this);
    }
    SimTK_DOWNCAST(Output, AbstractOutput);

private:
    mutable T _result;
    std::function<T(const SimTK::State&)> _outputFcn;

//=============================================================================
};	// END class Output

//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // OPENSIM_COMPONENT_OUTPUT_H_
