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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/Array.h>

namespace OpenSim { 

class Model;


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an analysis
 * plugin.
 *
 * @author Frank C. Anderson, Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Analysis: public Object
{
	OPENSIM_DECLARE_DERIVED(Analysis, Object);
public:


//=============================================================================
// DATA
//=============================================================================
public:
    Model* _model;
	const Storage* _statesStore;

private:
	/** Whether or not to write output of angles in degrees. */
	PropertyBool _inDegreesProp;
	bool &_inDegrees;

	// WORK ARRAYS
	/** Column labels. */
	Array<std::string> _labels;


protected:

    /** Step interval. */
    PropertyInt _stepIntervalProp;
    int &_stepInterval;

	/** On, off flag. */
	PropertyBool _onProp;
	bool &_on;

	/** Start time for the callback in normalized time. */
	PropertyDbl _startTimeProp;
	double &_startTime;

	/** End time for the callback in normalized time. */
	PropertyDbl _endTimeProp;
	double &_endTime;
	ArrayPtrs<Storage> _storageList;
	bool _printResultFiles;

//=============================================================================
// METHODS
//=============================================================================
private:
	void setNull();
	void setupProperties();

public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	/**
	 * Default constructor.
	 *
	 * @param aModel Model on which the analysis is to be performed.
	 */
	Analysis(Model *aModel=0);

	/**
	 * Construct an object from file.
	 * The object is constructed from the root element of the XML document.
	 * The type of object is the tag name of the XML root element.
	 * @param aFileName File name of the document.
	 */
	Analysis(const std::string &aFileName, bool aUpdateFromXMLNode = true);

	/**
	 * Copy constructor.
	 * @param aAnalysis Object to be copied.
	 * @see Analysis(const XMLDocument *aDocument)
	 * @see Analysis(const char *aFileName)
	 * @see generateXMLDocument()
	 */
	Analysis(const Analysis &aAnalysis);

	virtual ~Analysis();
	virtual Object* copy() const;

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	/**
	 * Assignment operator.
	 * @return Reference to this object.
	 */
	Analysis& operator=(const Analysis &aAnalysis);
#endif

   virtual int
        begin( SimTK::State& s);
    virtual int
        step( const SimTK::State& s, int stepNumber);
    virtual int
        end( SimTK::State& s);


	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// MODEL
	/**
	 * set pointer to model to be analyzed.
	 * @param aModel
	 */
	virtual void setModel(Model& aModel);
	// STATES STORAGE
	/**
	 * set states storage for analysis.
	 * @param aStatesStore
	 */
	virtual void setStatesStore(const Storage& aStatesStore);

	// ON,OFF
	void setOn(bool aTrueFalse);
	bool getOn() const;

	// START,END
	void setStartTime(double aStartTime);
	double getStartTime() const;

	void setEndTime(double aEndTime);
	double getEndTime() const;

	// DEGREES/RADIANS
	/**
	 * Set whether or not to write the output of angles in degrees.
	 * This flag must be set before an analysis is performed to ensure that
	 * the results are in the proper format.
	 * @param aTrueFalse Output will be in degrees if "true" and in radians
	 * if "false".
	 */
	void setInDegrees(bool aTrueFalse);
	bool getInDegrees() const;

    virtual bool proceed(int aStep=0);

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setStepInterval(int aStepInterval);
    int getStepInterval() const;

	// COLUMN LABLES
	/**
	 * Set the column labels for this analysis.
	 * @param aLabels an Array of strings (labels).
	 */
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
	/**
	 * Print the results of the analysis.
	 *
	 * @param aFileName File to which to print the data.
	 * @param aDT Time interval between results (linear interpolation is used).
	 * If not included as an argument or negative, all time steps are printed
	 * without interpolation.
	 *
	 * @return -1 on error, 0 otherwise.
	 */	
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class Analysis

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Analysis_h__


