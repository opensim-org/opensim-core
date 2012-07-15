#ifndef OPENSIM_ANALYSIS_PLUGIN_TEMPLATE_H_
#define OPENSIM_ANALYSIS_PLUGIN_TEMPLATE_H_
// AnalysisPlugin_Template.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Ajay Seth, Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2012, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
// Headers define the various property types that OpenSim objects can read 
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimPluginDLL.h"      // Header to define plugin (DLL) interface


//=============================================================================
//=============================================================================
/*
 * A class template for writing a custom Analysis 
 * Currenlty reports the position and orientation of bodies listed in its
 * properties.
 *
 * @author Frank C. Anderson, Ajay Seth, Tim Dorn
 * @version 3.0
 */
namespace OpenSim { 

class OSIMPLUGIN_API AnalysisPlugin_Template : public Analysis 
{
OpenSim_DECLARE_CONCRETE_OBJECT(AnalysisPlugin_Template, Analysis);
public:
//=======================================================================
// PROPERTIES
//=======================================================================
/** @name Property declarations
	These are the serializable properties associated with this class. **/
/**@{**/

    /** String list property containing the name of the body names*/
    OpenSim_DECLARE_LIST_PROPERTY(body_names, std::string, 
    "Names of the bodies on which to perform the analysis."
    "The key word 'All' indicates that the analysis should be performed for all bodies.");

    // Here are some examples of other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //// My string property
    //OpenSim_DECLARE_PROPERTY(string_property, std::string, 
    //"My string property."); 

    //// My int property
    //OpenSim_DECLARE_PROPERTY(int_property, int, 
    //"My int property."); 

    //// My bool property
    //OpenSim_DECLARE_PROPERTY(bool_property, bool, 
    //"My bool property."); 

    //// My double property
    //OpenSim_DECLARE_PROPERTY(double_property, double, 
    //"My double property."); 

/**@}**/


//=======================================================================
// INTERNAL MEMBER VARIABLES
//=======================================================================

	// In addition to properties, add any additional member variables
	// you need for your analysis.  These variables are not read from
	// or written to file.  They are just variables you use to execute
	// your analysis.  For example, you will almost certainly need a
	// storage object for storing the results of your analysis.

	// Storage object for storing and writing out results.  In general,
	// each storage file that you create will contain only one kind of data.
	// Create a different storage object for each kind of data.  For example,
	// create a _storePos for positions, _storeVel for velocities,
	// _storeAcc for accelerations, etc. */

	// Indices of bodies for kinematics to be reported
	Array<int> _bodyIndices;

	/** Storage for recording body positions. */
	Storage _storePos;

	/** Internal work arrays to hold body positions at each time step. */
	Array<double> _bodypos;

//=============================================================================
// METHODS
//=============================================================================
private:
    /** Construct default values for internal member variables, */
	/** i.e., zero data and set pointers to Null */
	void setNull();

	/** Construct default values for properties */
	void constructProperties();

public:
    /** Default constructor */
    AnalysisPlugin_Template();

    /** setModel */
	virtual void setModel(Model& aModel);

	//-------------------------------------------------------------------------
	// METHODS THAT MUST BE OVERRIDDEN
	//-------------------------------------------------------------------------
	virtual int
		begin(SimTK::State& s);
	virtual int
		step(const SimTK::State& s, int stepNumber);
	virtual int
		end(SimTK::State& s);

	//-------------------------------------------------------------------------
	// IO
	//-------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");


protected:
	//========================== Internal Methods =============================
	int record(const SimTK::State& s);
	void constructDescription();
	void constructColumnLabels();
	void setupStorage();

//=============================================================================
}; // END of class AnalysisPlugin_Template
}; //namespace
//=============================================================================
//=============================================================================

#endif // #ifndef OPENSIM_ANALYSIS_PLUGIN_TEMPLATE_H_
