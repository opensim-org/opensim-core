/* CoordinateReference.cpp
* Author: Ajay Seth
* Copyright (c)  2010 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CoordinateReference.h"
#include "Model/Model.h"

using namespace std;
using namespace SimTK;

namespace OpenSim {

CoordinateReference::CoordinateReference() : Reference_<double>(),
	_coordinateValueFunction(_coordinateValueFunctionProp.getValueObjPtrRef()),
	_defaultWeight(_defaultWeightProp.getValueDbl())
{
	_names.resize(getNumRefs());
	_names[0] = getName();
}

//______________________________________________________________________________
/**
 * An implementation of the CoordinateReference
 *
 * @param name of the reference to be found in the model and
 * @param referenceFunction is its its function returning its value
 */
	CoordinateReference::CoordinateReference(const std::string name, Function &referenceFunction) : Reference_<double>(name),
	_coordinateValueFunction(_coordinateValueFunctionProp.getValueObjPtrRef()),
	_defaultWeight(_defaultWeightProp.getValueDbl())
{
	setValueFunction(referenceFunction);
	_names.resize(getNumRefs());
	_names[0] = getName();
}

// Copy constructor
CoordinateReference::CoordinateReference(const CoordinateReference& source)
:   Super(source), 
    _coordinateValueFunction(_coordinateValueFunctionProp.getValueObjPtrRef()),
	_defaultWeight(_defaultWeightProp.getValueDbl()) 
{
    copyData(source);
}


CoordinateReference& CoordinateReference::operator=(const CoordinateReference& source)
{
    if (&source != this) {
	    Super::operator=(source);
        copyData(source);
    }
	return *this;
};

void CoordinateReference::copyData(const CoordinateReference& source)
{
	_coordinateValueFunction = source._coordinateValueFunction->clone();
	_defaultWeight = source._defaultWeight;
};


/** get the names of the referettes */
const SimTK::Array_<std::string>& CoordinateReference::getNames() const
{
	return _names;
}


/** get the values of the CoordinateReference */
void CoordinateReference::getValues(const SimTK::State &s, SimTK::Array_<double> &values) const
{
	SimTK::Vector t(1, s.getTime());
	values.resize(getNumRefs());
	values[0] = _coordinateValueFunction->calcValue(t);
}


/** get the weighting (importance) of meeting this Reference */
void CoordinateReference::getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const
{
	weights.resize(getNumRefs());
	weights[0] = _defaultWeight;
}


/** get the value of the CoordinateReference */
double CoordinateReference::getValue(const SimTK::State &s) const
{
	SimTK::Vector t(1, s.getTime());
	return _coordinateValueFunction->calcValue(t);
}

/** get the speed value of the CoordinateReference */
double CoordinateReference::getSpeedValue(const SimTK::State &s) const
{
	SimTK::Vector t(1, s.getTime());
	std::vector<int> order(1, 0);
	return _coordinateValueFunction->calcDerivative(order, t);
}

/** get the acceleration value of the CoordinateReference */
double CoordinateReference::getAccelerationValue(const SimTK::State &s) const
{
	SimTK::Vector t(1, s.getTime());
	std::vector<int> order(2, 0);
	return _coordinateValueFunction->calcDerivative(order, t);
}

/** get the weight of the CoordinateReference */
double CoordinateReference::getWeight(const SimTK::State &s) const
{
	return Reference_<double>::getWeights(s)[0];
}

/** set the weight of the CoordinateReference */
void CoordinateReference::setWeight(double weight)
{
	_defaultWeight = weight;
}

} // end of namespace OpenSim
