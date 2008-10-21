#ifndef _AnalysisPlugin_Template_h_
#define _AnalysisPlugin_Template_h_
// AnalysisPlugin_Template.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2008, Stanford University. All rights reserved. 
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
// Headers define the various property types that OpenSim objects can read 
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyBoolArray.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Simulation/Model/Analysis.h>
// Header to define plugin (DLL) interface
#include "osimPluginDLL.h"


//=============================================================================
//=============================================================================
/*
 * A class template for writing a custom Analysis 
 * Currenlty reports the position and orientation of bodies listed in its
 * properties.
 *
 * @author Frank C. Anderson, Ajay Seth
 * @version 1.6
 */
namespace OpenSim { 

class Model;


class OSIMPLUGIN_API AnalysisPlugin_Template : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	// Properties are the user-specified quantities that are read in from
	// file and are used to configure your class.
	// 
	// A property consists of a type, a name, and a value.  You can
	// access each of these by calling methods on the property.  For example,
	//
	// string type = property.getType();
	// string name = property.getName();
	// double value = property.getValueDbl();
	// double x = 1.0;
	// property.setValue(x);
	// 
	// To make writing your code more streamlined, you can initialize
	// a reference to point to the value of a property.  For example,
	//
	// double &_param1 = _param1Prop.getValueDbl();
	//
	// In this way you can write your code using _param1 as a normal
	// variable, instead of using get and set methods on
	// _param1Prop all the time.  The references to the properties
	// (e.g., _param1) are initialized in the initialization list
	// of the class constructors.
	//
	// Below are example member variables of different property
	// types available.
	//
	// To write your own custom analysis, delete the property types
	// you don't need and, if needed, add additional propertes of the
	// appropriate type.  Then, choose meaningful names for the properties.
	// For example, rename _boolProp to _expressInLocalFrameProp and
	// _bool to _expressInLocalFrame.  The names of the member variables
	// use names similar to the type of the variable (e.g., bool, int, dbl),
	// but this is just for convenience.  You can name the variables
	// anything you like, as long as it's a legal name.
	//
	// As a convention, all member variables of a class are
	// preceded with an underscore (e.g., _param1).  In this way,
	// you can tell which variables are member variables and which
	// are not as you code up your analysis.

	/** Boolean Property */
	//PropertyBool _boolProp;
	//bool &_bool;

	/** Boolean Array Property */
	//PropertyBoolArray _boolArrayProp;
	//Array<bool> &_boolArray;

	/** Integer Property */
	//PropertyInt _intProp;
	//int &_int;

	/** Integer Array Property */
	//PropertyIntArray _intArrayProp;
	//Array<int> &_intArray;

	/** Double Property */
	//PropertyDbl _dblProp;
	//double &_dbl;

	/** Double Array Property */
	//PropertyDblArray _dblArrayProp;
	//Array<double> &_dblArray;

	/** Dboule Vec3 Property */
	//PropertyDblVec3 _vec3Prop;
	//SimTK::Vec3 &_vec3;

	/** String Property */
	//PropertyStr _strProp;
	//std::string &_str;

	/** String Array Property */
	PropertyStrArray _strArrayProp;
	Array<std::string> &_bodyNames;

	// In addition to properties, add any additional member variables
	// you need for your analysis.  These variables are not read from
	// or written to file.  They are just variables you use to execute
	// your analysis.  For example, you will almost certainly need a
	// storage object for storing the results of your analysis.

	// Storage object for storing and writing out results.  In general,
	// each storage file that you create will contain only one kind of data.
	// Create a different storage object for each kind of data.  For example,
	// create a _storePos for positions, _storeVel for velocities,
	// _storeAcc for accelerations, etc. */

	// Indices of bodies for kinematics to be reported
	Array<int> _bodyIndices;

	/** Storage for recording body positions. */
	Storage _storePos;

	/** Internal work arrays to hold body positions at each time step. */
	Array<double> _bodypos;

//=============================================================================
// METHODS
//=============================================================================
public:
	/**
	 * Construct an AnalysisPlugin_Template instance with a Model.
	 *
	 * @param aModel Model for which the analysis is to be run.
	 */
	AnalysisPlugin_Template(Model *aModel=0);


	/**
	 * Construct an object from file.
	 *
	 * @param aFileName File name of the document.
	 */
	AnalysisPlugin_Template(const std::string &aFileName);

	/**
	 * Copy constructor.
	 */
	AnalysisPlugin_Template(const AnalysisPlugin_Template &aObject);

	//-------------------------------------------------------------------------
	// DESTRUCTOR
	//-------------------------------------------------------------------------
	virtual ~AnalysisPlugin_Template();

	/** Clone of object */
	virtual Object* copy() const;
private:
	/** Zero data and set pointers to Null */
	void setNull();

	/**
	 * Set up the properties for th analysis.
	 * Each property should have meaningful name and an informative comment.
	 * The name you give each property is the tag that will be used in the XML
	 * file.  The comment will appear before the property in the XML file.
	 * In addition, the comments are used for tool tips in the OpenSim GUI.
	 *
	 * All properties are added to the property set.  Once added, they can be
	 * read in and written to file.
	 */
	void setupProperties();

public:

#ifndef SWIG
	/**
	 * Assign this object to the values of another.
	 *
	 * @return Reference to this object.
	 */
	AnalysisPlugin_Template& operator=(const AnalysisPlugin_Template &aAnalysisPlugin_Template);
#endif

	//========================== Required Methods =============================
	//-------------------------------------------------------------------------
	// GET AND SET
	//-------------------------------------------------------------------------
	virtual void setModel(Model *aModel);

	//-------------------------------------------------------------------------
	// INTEGRATION
	//-------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);

	//-------------------------------------------------------------------------
	// IO
	//-------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");


protected:
	//========================== Internal Methods =============================
	int record(double aT,double *aX,double *aY);
	void constructDescription();
	void constructColumnLabels();
	void setupStorage();

//=============================================================================
}; // END of class AnalysisPlugin_Template
}; //namespace
//=============================================================================
//=============================================================================

#endif // #ifndef __AnalysisPlugin_Template_h__
