#ifndef _AnalysisFactory_h_
#define _AnalysisFactory_h_
// AnalysisFactory
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
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
