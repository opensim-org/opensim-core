#ifndef __GenericModelMaker_h__
#define __GenericModelMaker_h__

// GenericModelMaker.h
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
#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>

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
 * A class implementing a set of parameters describing
 * a generic musculoskeletal model.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMTOOLS_API GenericModelMaker : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(GenericModelMaker, Object);

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyStr _fileNameProp;
	std::string &_fileName;

	PropertyStr _markerSetFileNameProp;
	std::string &_markerSetFileName;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	GenericModelMaker();
	GenericModelMaker(const GenericModelMaker &aGenericModelMaker);
	virtual ~GenericModelMaker();

#ifndef SWIG
	GenericModelMaker& operator=(const GenericModelMaker &aGenericModelMaker);
#endif
	void copyData(const GenericModelMaker &aGenericModelMaker);

	Model* processModel(const std::string& aPathToSubject="");

	/* Register types to be used when reading a GenericModelMaker object from xml file. */
	static void registerTypes();

	/**
	 * Get file name for generic model
	 */
	const std::string& getModelFileName() const
	{
		return _fileName;
	}

	// Set model file name
	void setModelFileName(const std::string& aFileName)
	{
		_fileName = aFileName;
		_fileNameProp.setValueIsDefault(false);
	}

	const std::string& getMarkerSetFileName() const
	{
		return _markerSetFileName;
	}

	void setMarkerSetFileName(const std::string& aFileName)
	{
		_markerSetFileName = aFileName;
		_markerSetFileNameProp.setValueIsDefault(false);
	}

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class GenericModelMaker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __GenericModelMaker_h__


