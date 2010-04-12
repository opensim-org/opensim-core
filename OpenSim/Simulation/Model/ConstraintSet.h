#ifndef __ConstraintSet_h__
#define __ConstraintSet_h__

// ConstraintSet.h
// Author: Ajay Seth
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/SimbodyEngine/Constraint.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {


class Model;

//class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of constraints.
 *
 * @authors Ajay Seth
 * @version 1.0
 */

class OSIMSIMULATION_API ConstraintSet : public ModelComponentSet<Constraint>
{
private:
	void setNull();
public:
	ConstraintSet();
	ConstraintSet(Model& model);
	ConstraintSet(const ConstraintSet& aAbsConstraintSet);
	~ConstraintSet(void);
	void setup(Model& aModel);
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	ConstraintSet& operator=(const ConstraintSet &aAbsConstraintSet);
#endif
	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------

	// SCALE
	void scale(const ScaleSet& aScaleSet);

//=============================================================================
};	// END of class ConstraintSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ConstraintSet_h__
