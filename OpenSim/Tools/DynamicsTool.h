#ifndef __DynamicsTool_h__
#define __DynamicsTool_h__
// DynamicsTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
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
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include  <OpenSim/Simulation/Model/ForceSet.h>
#include  <OpenSim/Simulation/Model/ExternalLoads.h>
#include "Tool.h"

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * An abstract Tool for defining tools for perfroming a dynamics analysis 
 * with a given model. For example, InverseDynamics and ForwardDynamics Tools
 * derive from DynamicsTool, which provides convenient method for performing
 * and dynamics analysis over or to produce a trajectory in time.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API DynamicsTool: public Tool
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	
	/** Pointer to the model being investigated. */
	Model *_model;

	/** Name of the xml file used to deserialize or construct a model. */
	PropertyStr _modelFileNameProp;
	std::string &_modelFileName;

	/** The range of time over which to perform the dynamics analysis */
	PropertyDblVec2 _timeRangeProp;
	SimTK::Vec2 &_timeRange;

	/** Idenitify the list of forces to be ignored for computing dynamics */
	PropertyStrArray _excludedForcesProp;
	Array<std::string> &_excludedForces;

	/** Name of the file containing the external loads applied to the model. */
	OpenSim::PropertyStr _externalLoadsFileNameProp;
	std::string &_externalLoadsFileName;
	/** External loads object that manages loading and applying external forces
	    to the model, including transformations required by the Tool */
	ExternalLoads	_externalLoads;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~DynamicsTool();
	DynamicsTool();
	DynamicsTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
	DynamicsTool(const DynamicsTool &aTool);
	virtual OpenSim::Object* copy() const = 0;

	/** Modify model to exclude specified forces by disabling those identified by name or group */
	void disableModelForces(Model &model, SimTK::State &s, const Array<std::string> &forcesByNameOrGroup);
	
	const ExternalLoads& getExternalLoads() const { return _externalLoads; }
	ExternalLoads& updExternalLoads() { return _externalLoads; }

	// External loads get/set
	const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
	void setExternalLoadsFileName(const std::string &aFileName) { _externalLoadsFileName = aFileName; }

private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	DynamicsTool& operator=(const DynamicsTool &aDynamicsTool);
#endif


	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	void setStartTime(double d) { _timeRange[0] = d; };
	double getStartTime() const {return  _timeRange[0]; };

	void setEndTime(double d) { _timeRange[1] = d; };
	double getEndTime() const {return  _timeRange[1]; };
	void setModel(Model& aModel) { _model = &aModel; };

	void setExcludedForces(const Array<std::string> &aExcluded) {
		_excludedForces = aExcluded;
	}
    bool createExternalLoads( const std::string &aExternalLoadsFileName,
                                     Model& aModel, const Storage *loadKinematics=NULL);

	virtual bool run() SWIG_DECLARE_EXCEPTION=0;


//=============================================================================
};	// END of class DynamicsTool
//=============================================================================
} // namespace

#endif // __DynamicsTool_h__
