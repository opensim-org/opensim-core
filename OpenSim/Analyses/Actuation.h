#ifndef _Actuation_h_
#define _Actuation_h_
// Actuation.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"
#include "SimTKsimbody.h"


#ifdef SWIG
	#ifdef OSIMANALYSES_API
		#undef OSIMANALYSES_API
		#define OSIMANALYSES_API
	#endif
#endif
//=============================================================================
//=============================================================================
namespace OpenSim { 
/**
 * A class for recording the basic actuator information for a model
 * during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMANALYSES_API Actuation : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	/** Number of actuators. */
	int _na;
	/** Work array for storing forces, speeds, or powers. */
	double *_fsp;
	/** Force storage. */
	Storage *_forceStore;
	/** Speed storage. */
	Storage *_speedStore;
	/** Power storage. */
	Storage *_powerStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	Actuation(Model *aModel=0);
	Actuation(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	Actuation(const Actuation &aObject);
	virtual Object* copy() const;
	virtual ~Actuation();
private:
	void setNull();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	Actuation& operator=(const Actuation &aActuation);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getForceStorage() const;
	Storage* getSpeedStorage() const;
	Storage* getPowerStorage() const;
	// MODEL
	virtual void setModel(Model& aModel);

	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
#ifndef SWIG
	virtual int
        begin(const SimTK::State& s );
    virtual int
        step(const SimTK::State& s, int setNumber );
    virtual int
        end(const SimTK::State& s );
protected:
    virtual int
        record(const SimTK::State& s );
#endif
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class Actuation

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __Actuation_h__
