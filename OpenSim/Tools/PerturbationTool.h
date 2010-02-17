#ifndef __PerturbationTool_h__
#define __PerturbationTool_h__
// PerturbationTool.h
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include "osimToolsDLL.h"
#include "ForwardTool.h"
#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API PerturbationTool: public ForwardTool
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	// PERTURBATION PARAMETERS
	/** Perturbation time window. */
	PropertyDbl _pertWindowProp;
	double &_pertWindow;
	/** Time increment between perturbation windows. */
	PropertyDbl _pertIncrementProp;
	double &_pertIncrement;
	/** Magnitude of perturbation. */
	PropertyDbl _pertDFProp;
	double &_pertDF;
	/** Actuators to perturb. */
	PropertyStrArray _actuatorsToPerturbProp;
	Array<std::string> &_actuatorsToPerturb;
	/** Whether or not to perturb gravity. */
	PropertyBool _perturbGravityProp;
	bool &_perturbGravity;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PerturbationTool();
	PerturbationTool(const std::string &aFileName,bool aUpdateFromXMLNode=true,bool aLoadModel=true);
	virtual ~PerturbationTool();
	// Copy constrctor and virtual copy 
	PerturbationTool(const PerturbationTool &aObject);
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	PerturbationTool&
		operator=(const PerturbationTool &aPerturbationTool);
#endif

	//--------------------------------------------------------------------------
	// XML NEW
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode() {AbstractTool::updateFromXMLNode();};
	
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION;
	virtual void printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class PerturbationTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PerturbationTool_h__


