#ifndef __IKTool_h__
#define __IKTool_h__
// IKTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include "SimTKsimbody.h"

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {

class IKTaskSet;
class IKTrialSet;


//=============================================================================
//=============================================================================
/**
 * An investigation class for the IK solver.
 *
 * @author Eran Guendelman
 * @version 1.0
 */
class OSIMTOOLS_API IKTool: public AbstractTool
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	// tasks used to specify IK weights
	PropertyObj _ikTaskSetProp;
	IKTaskSet &_ikTaskSet;

	// the set of IK trials to perform
	PropertyObj _IKTrialSetProp;
	IKTrialSet &_IKTrialSet;

	/** Preferred optimizer algorithm. */
	PropertyStr _optimizerAlgorithmProp;
	std::string &_optimizerAlgorithm;

	// Whether or not to write write to the designated output files (GUI will set this to false)
	bool _printResultFiles;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~IKTool();
	IKTool();
	IKTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
	IKTool(const IKTool &aObject);
	virtual OpenSim::Object* copy() const;

	/* Register types to be used when reading an IKTool object from xml file. */
	static void registerTypes();
private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	IKTool&
		operator=(const IKTool &aIKTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	IKTrialSet& getIKTrialSet() { return _IKTrialSet; }

	IKTaskSet& getIKTaskSet() { return _ikTaskSet; }

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	bool initializeTrial(const SimTK::State& s, int i);
	bool solveTrial( SimTK::State& s, int i);
	virtual bool run() SWIG_DECLARE_EXCEPTION;
	bool run(SimTK::State& state);

	void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

//=============================================================================
};	// END of class IKTool
//=============================================================================
} // namespace

#endif // __IKTool_h__
