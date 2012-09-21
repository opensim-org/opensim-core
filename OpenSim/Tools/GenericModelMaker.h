#ifndef __GenericModelMaker_h__
#define __GenericModelMaker_h__
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  GenericModelMaker.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


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


