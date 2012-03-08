#ifndef _StorageInterface_h_
#define _StorageInterface_h_
// StorageInterface.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005-2010, Stanford University. All rights reserved. 
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

/* Abstract an interface out of the Storage class to be used by readers/writers of other file formats 
 * Author: Ayman Habib 
 */

#include "osimCommonDLL.h"
//=============================================================================
//=============================================================================
/**
 *
 * @version 1.0
 * @author Ayman Habib
 */
namespace OpenSim { 

class StateVector;

class OSIMCOMMON_API StorageInterface : public Object
{
//=============================================================================
// METHODS
//=============================================================================
public:
	// make this constructor explicit so you don't get implicit casting of int to StorageInterface
	StorageInterface(const std::string &aFileName) SWIG_DECLARE_EXCEPTION{};
	StorageInterface(const StorageInterface& aStorageInterface) {};
	virtual Object* copy() const=0;
	virtual ~StorageInterface(){};

#ifndef SWIG
	StorageInterface& operator=(const StorageInterface &aStorageInterface)
	{
		Object::operator=(aStorageInterface);
		return(*this);
	}
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// SIZE
	virtual int getSize() const =0;
	// STATEVECTOR
	virtual StateVector* getStateVector(int aTimeIndex) const =0;
	virtual StateVector* getLastStateVector() const =0;
	// TIME
	virtual double getFirstTime() const =0;
	virtual double getLastTime() const =0;
	virtual int getTimeColumn(Array<double>& rTimes,int aStateIndex=-1) const =0;
	virtual void getTimeColumnWithStartTime(Array<double>& rTimes,double startTime=0.0) const =0;
	// DATA
	virtual int getDataAtTime(double aTime,int aN,Array<double> &rData) const =0;
	virtual void getDataColumn(const std::string& columnName, Array<double>& data, double startTime=0.0) =0;

	//--------------------------------------------------------------------------
	// STORAGE
	//--------------------------------------------------------------------------
	virtual int append(const StateVector &aVec, bool aCheckForDuplicateTime=true) =0;
	virtual int append(const Array<StateVector> &aArray) =0;
	virtual int append(double aT,int aN,const double *aY, bool aCheckForDuplicateTime=true) =0;
	virtual int append(double aT,const SimTK::Vector& aY, bool aCheckForDuplicateTime=true) =0;
	virtual int append(double aT, const SimTK::Vec3& aY,bool aCheckForDuplicateTime=true){
		return append(aT, 3, &aY[0], aCheckForDuplicateTime);
	}
	virtual int store(int aStep,double aT,int aN,const double *aY) =0;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual int findIndex(double aT) const =0;
	virtual int findIndex(int aI,double aT) const =0;
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual void setOutputFileName(const std::string& aFileName) =0;

//=============================================================================
};	// END of class StorageInterface

}; //namespace
//=============================================================================
//=============================================================================

#endif //__StorageInterface_h__
