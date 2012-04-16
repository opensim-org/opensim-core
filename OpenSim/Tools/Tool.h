#ifndef _Tool_h_
#define _Tool_h_
// Tool.h
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
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
	*/
	Tool(const std::string &aFileName, bool aUpdateFromXMLNode = true):
		Object(aFileName, false), _inputsDir(_inputsDirProp.getValueStr()),
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


