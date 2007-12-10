#ifndef _Analysis_h_
#define _Analysis_h_
// Analysis.h
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

//============================================================================

#include "IntegCallback.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/ArrayPtrs.h>

namespace OpenSim { 

class Model;


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an analysis
 * plugin.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API Analysis: public IntegCallback
{
	OPENSIM_DECLARE_DERIVED(Analysis, IntegCallback);
public:
	//enum { DESCRIP_LENGTH=8192 };

//=============================================================================
// DATA
//=============================================================================
private:
	/** Whether or not to write output of angles in degrees. */
	PropertyBool _inDegreesProp;
	bool &_inDegrees;

	// WORK ARRAYS
	/** Column labels. */
	Array<std::string> _labels;

protected:
	ArrayPtrs<Storage> _storageList;
	bool _printResultFiles;
//=============================================================================
// METHODS
//=============================================================================
private:
	void setNull();
	void setupProperties();
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Analysis(Model *aModel=0);
	Analysis(const std::string &aFileName, bool aUpdateFromXMLNode = true);
	virtual ~Analysis();
	Analysis(const Analysis &aObject);
	virtual Object* copy() const;

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	Analysis& operator=(const Analysis &aAnalysis);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// MODEL
	virtual void setModel(Model *aModel);
	// DEGREES/RADIANS
	void setInDegrees(bool aTrueFalse);
	bool getInDegrees() const;
	// COLUMN LABLES
	void setColumnLabels(const Array<std::string> &aLabels);
	const Array<std::string> &getColumnLabels() const;

#ifndef SWIG
	// These symbols are swigged out because they are not defined and never used!
	// STORAGE INTERVAL
	void setStorageInterval(int aInterval);
	int getStorageInterval() const;
#endif
	virtual ArrayPtrs<Storage>& getStorageList();
	void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }
	bool getPrintResultFiles() const { return _printResultFiles; }

	//--------------------------------------------------------------------------
	// RESULTS
	//--------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class Analysis

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Analysis_h__


