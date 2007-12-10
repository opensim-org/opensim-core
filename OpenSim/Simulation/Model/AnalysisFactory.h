#ifndef _AnalysisFactory_h_
#define _AnalysisFactory_h_
// AnalysisFactory
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

/*  
 * Author:  Ayman Habib
 
 */

#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
//=============================================================================
//=============================================================================
/**
 *
 * A factory class to be used to maintain/extend the map of available analyses
 * The class is implemented following the Singleton pattern (to make sure a single instance
 * of the class lives in the system).
 * Singleton Pattern (Design Patterns GoF)
 *
 * AnalysisFactory instantiates the factory for and makes registered analyses available 
 * for users. All created analyses will implement the Analysis interface (virtual functions)
 * and clients will not care about which concrete analysis class is in use.
 *
 * @version 1.0
 * @author Ayman Habib
 */

namespace OpenSim { 

class Analysis;

class OSIMSIMULATION_API AnalysisFactory
{
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// DATA
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
private:
	static AnalysisFactory *_factoryInstance;
	ArrayPtrs<Analysis> _analysesList;
	/** Name to be shown by the UI */
	static std::string _displayName;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// METHODS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//=============================================================================
// CONTRUCTION
//=============================================================================
protected:
	AnalysisFactory();   //Protected constructor as per singletons
public:
	virtual ~AnalysisFactory();

	/** Singleton support. Sole method to instantiate and access AnalysisFactory*/
	static AnalysisFactory*	getInstance();

	/** Factory method used to create Analyses of specified name */
	Analysis*  createAnalysis(const std::string &aAnalysisName) const;

	/** Register an analysis type with the fcatory so that it can be instantiated later */
	bool registerAnalysis(Analysis *aAnalysis);

	// Name for display purposes
	const std::string& toString() const;

	const ArrayPtrs<Analysis> &getRegisteredAnalyses();
protected:
	// Method to find out if an aAnalysisName is already registered */
	bool  analysisExists(const std::string &aAnalysisName) const;

//=============================================================================
};	// END of class AnalysisFactory

}; //namespace
//=============================================================================
//=============================================================================


#endif //__AnalysisFactory_h__
