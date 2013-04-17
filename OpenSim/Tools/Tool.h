#ifndef _Tool_h_
#define _Tool_h_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Tool.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ArrayPtrs.h>


namespace OpenSim { 


//=============================================================================
//=============================================================================
/**
 * A Tool is an OpenSim abstraction that encapsulates an analysis or series of
 * modeling and analysis steps. Its primary duty is to provide an interface
 * for use by the GUI or as a standalone command line executable. It includes
 * common methods for invoking the tool and performing routine I/O.
 * 
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API Tool : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Tool, Object);

public:
	/** Perturbation types. See setPerturbation(). */
	enum VerboseLevel {Quiet=0, Progress=1, DetailedProgress=2, Debug=3};
//=============================================================================
// DATA
//=============================================================================
protected:
    
	/** Directory for reading inputs (model, settings, etc...) from. */
	PropertyStr _inputsDirProp;
	std::string &_inputsDir;
	
	/** Directory for writing results (new model, states, etc...) to. */
	PropertyStr _resultsDirProp;
	std::string &_resultsDir;
	
	/** How much details to put out while running. */
	VerboseLevel _verboseLevel;
	
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	
	/**
	* Destructor.
	*/
	virtual ~Tool() {};
	
	/**
	* Default constructor.
	*/
	Tool() : _inputsDir(_inputsDirProp.getValueStr()),
		_resultsDir(_resultsDirProp.getValueStr())
		{ setNull(); };
	
	/**
	* Construct from file
	*
	* The object is constructed from the root element of the XML document.
	* The type of object is the tag name of the XML root element.
	*
	* @param aFileName File name of the document.
    * @param aUpdateFromXMLNode
	*/
	Tool(const std::string &aFileName, bool aUpdateFromXMLNode = true):
		Object(aFileName, true), _inputsDir(_inputsDirProp.getValueStr()),
		_resultsDir(_resultsDirProp.getValueStr()) {
			setNull();
			if(aUpdateFromXMLNode) updateFromXMLDocument();
		};
	
	/**
	* Copy constructor.
	*
	* @param aTool to be copied.
	*/
	Tool(const Tool &aTool) : _inputsDir(_inputsDirProp.getValueStr()),
		_resultsDir(_resultsDirProp.getValueStr())
		{setNull(); *this = aTool; };


private:
	/**
	* Set all member variables to their null or default values.
	*/
	void setNull() {
		setupProperties();
		_resultsDir = "./"; 
		_inputsDir = "";
		_verboseLevel = Progress;
	};
	
	/**
	* Connect properties to local pointers.
	*/
	void setupProperties()
	{
		std::string comment;
		comment = "Directory used for writing results.";
		_resultsDirProp.setComment(comment);
		_resultsDirProp.setName("results_directory");
		_propertySet.append( &_resultsDirProp );

		comment = "Directory for input files";
		_inputsDirProp.setComment(comment);
		_inputsDirProp.setName("input_directory");
		_propertySet.append( &_inputsDirProp );
	};
	


	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	
	/**
	* Assignment operator.
	*
	* @return Reference to this object.
	*/
	Tool& operator=(const Tool& source) {
        if (&source != this) {
            Super::operator=(source);	
			_resultsDir   = source._resultsDir; 
			_inputsDir    = source._inputsDir;
			_verboseLevel = source._verboseLevel;
        }
		return *this;
    }

#endif


	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	/** The run() method of a Tool embodies what would be the main() routine 
	    for a standalone program.  Therefore, any OpenSim main program can
		become a Tool executable from the GUI by making it a run() method for
		a new Tool.
		
		It is expected that the run() method be composed of a sequence of calls  
		to underlying computational and reporting objects and should 
	*/
	virtual bool run() SWIG_DECLARE_EXCEPTION=0;

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	/** 
	* Get/set Inputs Directory
	*/
	const std::string& getInputsDir() const { return _inputsDir; }
	void setInputsDir(const std::string& aString) { _inputsDir = aString; }
	/** 
	* Get/set Results Directory
	*/
	const std::string& getResultsDir() const { return _resultsDir; }
	void setResultsDir(const std::string& aString) { _resultsDir = aString; }

	/**
	 * Get/Set verbose level
	 */
	const VerboseLevel getVerboseLevel() const { return _verboseLevel; };
	void setVerboseLevel(const VerboseLevel aVerboseLevel) { _verboseLevel = aVerboseLevel; };
//=============================================================================
};	// END of class Tool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Tool_h__


