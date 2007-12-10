#ifndef __BodyScale_h__
#define __BodyScale_h__

// BodyScale.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include <iostream>
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/PropertyStrArray.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing how
 * to scale a body segment.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API BodyScale : public Object  
{
	OPENSIM_DECLARE_DERIVED(BodyScale,Object);

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyStrArray _axisNamesProp;
	Array<std::string>& _axisNames;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	BodyScale();
	BodyScale(const BodyScale &aBodyScale);
	virtual ~BodyScale();
	virtual Object* copy() const;

#ifndef SWIG
	BodyScale& operator=(const BodyScale &aBodyScale);
#endif
	void copyData(const BodyScale &aBodyScale);

#ifndef SWIG
	const Array<std::string>& getAxisNames() const { return _axisNames; }
#endif
	Array<std::string>& getAxisNames() { return _axisNames; }

	void setAxisNames(const Array<std::string> &aAxisNames) { 
		_axisNames = aAxisNames;
		_axisNamesProp.setUseDefault(false);
	}

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class BodyScale
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __BodyScale_h__


