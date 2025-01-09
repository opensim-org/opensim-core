#ifndef OPENSIM_REFERENCE_H_
#define OPENSIM_REFERENCE_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Reference.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Common/Object.h"
#include "SimTKcommon/SmallMatrix.h"
#include "SimTKcommon/internal/Array.h"
#include "SimTKcommon/internal/ResetOnCopy.h"

namespace SimTK {
class State;
}

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This base (abstract) class defines the interface for a Reference signals to
 * be achieved/tracked via optimization and/or tracking controller. Combines
 * weightings that identifies the relative importance of achieving one
 * Reference value relative to the others. The specific value type is defined
 * by the concrete References. For example, a MarkerRefrence is of type Vec3,
 * for the 3D location coordinates of a marker. Correspondence with model
 * values are established via the Reference names.
 *
 * @author Ajay Seth
 */
template <class T> class Reference_ : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT_T(Reference_, T, Object);
    //=============================================================================
    // METHODS
    //=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    virtual ~Reference_() {}

    Reference_() : Object() {}
    Reference_(std::string name) : Reference_() { setName(name); }

    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
    /** get the number of referettes (individual signals) in this Reference. All
        return arrays are guaranteed to be this length */
    virtual int getNumRefs() const = 0;
    /** get the time range for which the Reference is valid, which can and will
        be finite if the reference encapsulates experimental data. By default
        they are infinite */
    virtual SimTK::Vec2 getValidTimeRange() const {
        return SimTK::Vec2(-SimTK::Infinity, SimTK::Infinity);
    }
    /** get the name(s) of the reference or its referettes */
    virtual const SimTK::Array_<std::string>& getNames() const = 0;

    /** get the weighting (importance) of meeting this Reference */
    virtual void getWeights(
            const SimTK::State& s, SimTK::Array_<double>& weights) const = 0;
    /** Indicate whether this Reference can provide discretized data or not */
    virtual bool hasNext() const { return false; };
    //--------------------------------------------------------------------------
    // Convenience Interface
    //--------------------------------------------------------------------------
    /* getWeights as above, but a copy is returned, which may be costly */
    virtual SimTK::Array_<double> getWeights(const SimTK::State& s) const {
        SimTK::Array_<double> weights(getNumRefs());
        getWeights(s, weights);
        return weights;
    }
    /** get the values of the Reference signals as a function
    of the passed in time */
    virtual void getValuesAtTime(
            double time, SimTK::Array_<T>& values) const = 0;
    /* getValues but a copy is returned, which may be costly */
    virtual SimTK::Array_<T> getValues(double time) const {
        SimTK::Array_<T> values(this->getNumRefs());
        getValuesAtTime(time, values);
        return values;
    }
};

// Subclass for Streamable Reference signals
// This can support adding data on the fly
// The concept of getting "Next" set of values and corresponding
// time makes sense for these References.
template <class T> class StreamableReference_ : public Reference_<T> {
    OpenSim_DECLARE_ABSTRACT_OBJECT_T(StreamableReference_, T, Reference_<T>);

    using Reference_<T>::Reference_;
public:
    // Optionally support streaming mode where data can be added and used
    virtual double getNextValuesAndTime(SimTK::Array_<T>& values) = 0;
    // indicate whether to stop or wait for more data
    virtual bool hasNext() const override = 0;
};
  // END of class templatized Reference_<T>
//=============================================================================
}

#endif // OPENSIM_REFERENCE_H_
