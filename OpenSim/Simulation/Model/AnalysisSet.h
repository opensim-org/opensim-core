#ifndef _AnalysisSet_h_
#define _AnalysisSet_h_
// AnalysisSet.h
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <string>
#include <OpenSim/Common/Set.h>
#include "Analysis.h"


//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of integration callbacks for
 * a model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class Model;

class OSIMSIMULATION_API AnalysisSet : public Set<Analysis>
{
//=============================================================================
// DATA
//=============================================================================
public:
   AnalysisSet&
        operator=(const AnalysisSet &aAnalysisSet);
protected:
	/** Model on which the callbacks have been set. */
	Model *_model;

    // testing for memory free error
    OpenSim::PropertyBool _enableProp;
    bool &_enable;
//
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AnalysisSet();
	AnalysisSet(Model *aModel);
	AnalysisSet(const std::string &aFileName);
	AnalysisSet(const AnalysisSet &aSet);
	virtual ~AnalysisSet();
	virtual Object* copy() const;
private:
	void setNull();
    void setupProperties();
public:

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setModel(Model& aModel);
	Model& getModel();
	void setOn(bool aTrueFalse);
	void setOn(const Array<bool> &aOn);
	Array<bool> getOn() const;

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		begin(const SimTK::State& s );
	virtual void
		step(const SimTK::State& s, int stepNumber );
	virtual void
		end(const SimTK::State& s );

	//--------------------------------------------------------------------------
	// RESULTS
	//--------------------------------------------------------------------------
	virtual void
		printResults(const std::string &aBaseName,const std::string &aPath="",
		double aDT=-1.0,const std::string &aExtension=".sto");

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	static void getAvailableAnalyses(AnalysisSet& as);

//=============================================================================
};	// END of class AnalysisSet

}; //namespace
//=============================================================================
//=============================================================================

#endif // __AnalysisSet_h__


