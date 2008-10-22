#ifndef __CoordinateCouplerConstraint_h__
#define __CoordinateCouplerConstraint_h__

// CoordinateCouplerConstraint.h
// Author: Ajay Seth
/*
 * Copyright (c) 2008, Stanford University. All rights reserved. 
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


// INCLUDE
#include <string>
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Common/Function.h>
#include "Constraint.h"
#include "Body.h"
#include "SimbodyEngine.h"
#include "simbody/internal/common.h"


// Helper class to construct functions when user's specify a dependency as qd = f(qi)
// this function casts as C(q) = 0 = f(qi) - qd;
class CompoundFunction : public SimTK::Function<1> {
// returns f1(x[0]) - x[1];
private:
	const SimTK::Function<1> *f1;

public:
	
	CompoundFunction(const Function<1> *cf) : f1(cf){
	}

    SimTK::Vec<1> calcValue(const SimTK::Vector& x) const {
		double ka = x[0];
		double txy = x[1];
		SimTK::Vector xf(1);
		xf[0] = x[0];
		SimTK::Vec1 val = f1->calcValue(xf)-x[1];
		return val;
    }
    SimTK::Vec<1> calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const {
		SimTK::Vec1 df1;
		if (derivComponents.size() == 1){
			if (derivComponents[0]==0){
				SimTK::Vector x1(1);
				x1[0] = x[0];
				df1 = f1->calcDerivative(derivComponents, x1);
				return df1;
			}
			else if (derivComponents[0]==1)
				return SimTK::Vec1(-1);
		}
		else if(derivComponents.size() == 2){
			if (derivComponents[0]==0 && derivComponents[1] == 0){
				SimTK::Vector x1(1);
				x1[0] = x[0];
				df1 = f1->calcDerivative(derivComponents, x1);
				return df1;
			}
		}
        return SimTK::Vec1(0);
    }

    int getArgumentSize() const {
        return 2;
    }
    int getMaxDerivativeOrder() const {
        return 2;
    }

	void setFunction(const Function<1> *cf) {
		f1 = cf;
	}
};


namespace OpenSim {

//=============================================================================
//=============================================================================
 /** @file
 * A class implementing a CoordinateCoupler Constraint.  The underlying Constraint 
 * is a Constraint::CoordinateCoupler in Simbody, which relates coordinates
 * of the same or different body(ies) to one another at the position level (holonomic).
 * Relationship between coordinates is a specified by a function that equates 
 * to zero only when the coordinates satisfy the function (constraint).
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API CoordinateCouplerConstraint : public Constraint  
{

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Constraint function of generalized coordinates (to be specified) used to
	evaluate the constraint errors and their derivatives, and must valid to at
	least 2nd order. Constraint function must evaluate to zero when coordinates
	satisfy constraint */
	PropertyObjPtr<Function> _functionProp;
	Function *&_function;

	/** List of names of the independent coordinates (restricted to 1 for now). */
	PropertyStrArray _independentCoordNamesProp;
	Array<std::string>& _independentCoordNames;

	/** Name of the dependent coordinate. */
	PropertyStr _dependentCoordNameProp;
	std::string& _dependentCoordName;

	/** List of mobilized body indices established when constraint is setup */
	std::vector<SimTK::MobilizedBodyIndex> _bodies;
	/** List of coordinate (Q) indices corresponding to the respective body */
	std::vector<SimTK::MobilizerQIndex> _coordinates;

	/** Pointer to the underlying SimTK function that the Simbody::CoordinateCoupler evaluates */
	CompoundFunction* _simtkCouplerFunction;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	CoordinateCouplerConstraint();
	CoordinateCouplerConstraint(const CoordinateCouplerConstraint &aConstraint);
	virtual ~CoordinateCouplerConstraint();
	virtual Object* copy() const;
	CoordinateCouplerConstraint& operator=(const CoordinateCouplerConstraint &aConstraint);
	void copyData(const CoordinateCouplerConstraint &aConstraint);
	void setup(AbstractDynamicsEngine* aEngine);

	// GET AND SET
	void setIndependentCoordinateNames(const Array<std::string> &aCoordNames) { _independentCoordNames = aCoordNames; }
	const Array<std::string>& getIndependentCoordinateNames() const { return _independentCoordNames; }
	void setDependentCoordinateName(const std::string &aCoordName) { _dependentCoordName = aCoordName; }
	const std::string& getDependentCoordinateName() const { return _dependentCoordName; }
	Function* getFunction() const {return _function; }

	// SCALE
	virtual void scale(const ScaleSet& aScaleSet);

	OPENSIM_DECLARE_DERIVED(CoordinateCouplerConstraint, AbstractConstraint);

private:
	void setNull();
	void setupProperties();
	void updateSimbody();
	friend class SimbodyEngine;

//=============================================================================
};	// END of class CoordinateCouplerConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __CoordinateCouplerConstraint_h__


