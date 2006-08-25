// StateVector.cpp
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "rdMath.h"
#include "IO.h"
#include "StateVector.h"



using namespace OpenSim;
using namespace std;



//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
StateVector::~StateVector()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
StateVector::StateVector(double aT,int aN,const double aData[]) :
	_data(0.0)
{
	// INITIAL VALUES
	setNull();

	// SET THE DATA
	setStates(aT,aN,aData);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 */
StateVector::StateVector(const StateVector &aVector) :
	_data(0.0)
{
	// INITIAL VALUES
	setNull();

	// SET STATES
	setStates(aVector.getTime(),aVector.getSize(),&aVector.getData()[0]);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the null or default values of the states.
 */
void StateVector::
setNull()
{
	// DATA
	_t = 0.0;
	_data.setSize(0);
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this statevector to the values of another.
 *
 * @return Reference to this statevector.
 */
StateVector& StateVector::
operator=(const StateVector &aStateVector)
{
	_t = aStateVector._t;
	_data = aStateVector._data;
	return(*this);
}

//-----------------------------------------------------------------------------
// EQUALITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if two statevectors are equal.
 *
 * Equality is very restrictive.  To be equal, two statevectors must have
 * the same values for time and all states.
 *
 * @return True if the two statevectors are equal; false otherwise.
 */
bool StateVector::
operator==(const StateVector &aStateVector) const
{
	if((_t==aStateVector._t) && (_data==aStateVector._data)) return(true);
	return(false);
}

//-----------------------------------------------------------------------------
// LESS THAN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if this statevector is less than another.
 *
 * A statevector is less than another statevector if its time is less than
 * the other's time.
 *
 * @param aStateVector Statevector for which to make the less than test.
 * @return True if this statevector is less than the specified; false otherwise.
 */
bool StateVector::
operator<(const StateVector &aStateVector) const
{
	return(_t < aStateVector._t);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the time stamp of this vector.
 */
void StateVector::
setTime(double aT)
{
	_t = aT;
}
//_____________________________________________________________________________
/**
 * Get the time stamp of this vector.
 */
double StateVector::
getTime() const
{
	return(_t);
}
//_____________________________________________________________________________
/**
 * Set the state values of this vector.
 */
void StateVector::
setStates(double aT,int aN,const double *aData)
{
	_t = aT;
	_data.setSize(aN);
	int size = _data.getSize();
	for(int i=0;i<size;i++) {
		_data[i] = aData[i];
	}
}
//_____________________________________________________________________________
/**
 * Get the size of the data vector.
 */
int StateVector::
getSize() const
{
	return(_data.getSize());
}
//_____________________________________________________________________________
/**
 * Get the data values of this vector.
 */
Array<double>& StateVector::
getData()
{
	return(_data);
}
//_____________________________________________________________________________
/**
 * Get the data values of this vector.
 */
const Array<double>& StateVector::
getData() const
{
	return(_data);
}
//_____________________________________________________________________________
/**
 * Get the data value at a specified index.
 *
 * @param aIndex Index of the desired value:  0 < aIndex < getSize().
 * @param rValue Value of the data point if it is defined.
 * @return 1 on success, 0 on failure.
 */
int StateVector::
getDataValue(int aIndex,double &rValue)
{
	if(aIndex<0) return(0);
	if(aIndex>=_data.getSize()) return(0);

	rValue = _data[aIndex];
	return(1);
}

//_____________________________________________________________________________
/**
 * Set the data value at a specified index.
 *
 * @param aIndex Index of the desired value:  0 < aIndex < getSize().
 * @param aValue Value of the data point if it is defined.
 */
void StateVector::
setDataValue(int aIndex,double &aValue)
{
	if(aIndex<0) return;
	if(aIndex>=_data.getSize()) return;

	_data[aIndex]=aValue;
}

//=============================================================================
// OPERATIONS
//=============================================================================
//-----------------------------------------------------------------------------
// TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Shift the time of this state vector.
 *
 * @param aValue Value by which to shift time.
 */
void StateVector::
shiftTime(double aValue)
{
	_t += aValue;
}
//_____________________________________________________________________________
/**
 * Scale the time of this state vector.
 *
 * @param aValue Value by which to scale time.
 */
void StateVector::
scaleTime(double aValue)
{
	_t *= aValue;
}

//-----------------------------------------------------------------------------
// ADD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Add a value to each state.
 *
 * @param aValue Value to add to each state.
 */
void StateVector::
add(double aValue)
{
	int i;
	for(i=0;i<_data.getSize();i++)  _data[i] += aValue;
}
//_____________________________________________________________________________
/**
 * Add the values held in an array to the corresponding states.
 *
 * Only the first aN states are altered.
 *
 * @param aN Length of aY.
 * @param aY Array of values to add to the states.
 */
void StateVector::
add(int aN,double aY[])
{
	if(aY==NULL) return;
	int i,n=aN;
	if(n>_data.getSize()) n = _data.getSize();
	for(i=0;i<n;i++)  _data[i] += aY[i];
}
//_____________________________________________________________________________
/**
 * Add a value to a state.
 *
 * Only one state is altered.  This function was implemented so that 
 * a value could be added to an entire column of a Storage.
 *
 * @param aN Index of state to be altered.
 * @param aValue Values to add to the state.
 */
void StateVector::
add(int aN,double aValue)
{
	if(aValue==0) return;
	if(aN>_data.getSize()) return;
	_data[aN] += aValue;
}
//_____________________________________________________________________________
/**
 * Add the values held in a state vector to the corresponding states.
 *
 * @param aStateVector State vector to add to the states.
 */
void StateVector::
add(StateVector *aStateVector)
{
	if(aStateVector==NULL) return;

	// GET SIZE
	int n = aStateVector->getSize();
	if(n>_data.getSize()) n = _data.getSize();

	// GET DATA
	Array<double> &data = aStateVector->getData();

	// ADD
	int i;
	for(i=0;i<n;i++)  _data[i] += data[i];
}

//-----------------------------------------------------------------------------
// Subtract
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Subtract a value from each state.
 *
 * @param aValue Value to subtract from each state.
 */
void StateVector::
subtract(double aValue)
{
	int i;
	for(i=0;i<_data.getSize();i++)  _data[i] -= aValue;
}
//_____________________________________________________________________________
/**
 * Subtract the values held in an array from the corresponding states.
 *
 * Only the first aN states are altered.
 *
 * @param aN Length of aY.
 * @param aY Array of values to subtracted from the states.
 */
void StateVector::
subtract(int aN,double aY[])
{
	if(aY==NULL) return;
	int i,n=aN;
	if(n>_data.getSize()) n = _data.getSize();
	for(i=0;i<n;i++)  _data[i] -= aY[i];
}
//_____________________________________________________________________________
/**
 * Subtract the values held in a state vector from the corresponding states.
 *
 * @param aStateVector State vector to subtract from the states.
 */
void StateVector::
subtract(StateVector *aStateVector)
{
	if(aStateVector==NULL) return;

	// GET SIZE
	int n = aStateVector->getSize();
	if(n>_data.getSize()) n = _data.getSize();

	// GET DATA
	Array<double> &data = aStateVector->getData();

	// SUBTRACT
	int i;
	for(i=0;i<n;i++)  _data[i] -= data[i];
}

//-----------------------------------------------------------------------------
// MULTIPLY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Multiply the states by a value.
 *
 * @param aValue Value each state is to be multiplied by.
 */
void StateVector::
multiply(double aValue)
{
	int i;
	for(i=0;i<_data.getSize();i++)  _data[i] *= aValue;
}
//_____________________________________________________________________________
/**
 * Multiply the states by the corresponding values in an array.
 *
 * Only the first aN states are altered.
 *
 * @param aN Length of aY.
 * @param aY Array of values the states are multiplied by.
 */
void StateVector::
multiply(int aN,double aY[])
{
	if(aY==NULL) return;
	int i,n=aN;
	if(n>_data.getSize()) n = _data.getSize();
	for(i=0;i<n;i++)  _data[i] *= aY[i];
}
//_____________________________________________________________________________
/**
 * Multiply the states by the corresponding values in a state vector.
 *
 * @param aStateVector State vector by which to multiply the states.
 */
void StateVector::
multiply(StateVector *aStateVector)
{
	if(aStateVector==NULL) return;

	// GET SIZE
	int n = aStateVector->getSize();
	if(n>_data.getSize()) n = _data.getSize();

	// GET DATA
	Array<double> &data = aStateVector->getData();

	// MULTIPLY
	int i;
	for(i=0;i<n;i++)  _data[i] *= data[i];
}

//-----------------------------------------------------------------------------
// DIVIDE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Divide the states by a value.
 *
 * @param aValue Value each state is to be divided by.
 */
void StateVector::
divide(double aValue)
{
	if(aValue==0.0) {
		printf("StateVector.divide: ERROR- divide by zero\n");
		return;
	}

	int i;
	for(i=0;i<_data.getSize();i++) {
		_data[i] /= aValue;
	}
}
//_____________________________________________________________________________
/**
 * Divide the states by the corresponding values in an array.
 *
 * Only the first aN states are altered.
 *
 * @param aN Length of aY.
 * @param aY Array of values the states are divided by.
 */
void StateVector::
divide(int aN,double aY[])
{
	if(aY==NULL) return;
	int i,n=aN;
	if(n>_data.getSize()) n = _data.getSize();
	for(i=0;i<n;i++) {  
		if(aY[i]==0.0)	_data[i] = rdMath::NAN;
		else	_data[i] /= aY[i];
	}
		
}
//_____________________________________________________________________________
/**
 * Divide the states by the corresponding values in a state vector.
 *
 * @param aStateVector State vector by which to divide the states.
 */
void StateVector::
divide(StateVector *aStateVector)
{
	if(aStateVector==NULL) return;

	// GET SIZE
	int n = aStateVector->getSize();
	if(n>_data.getSize()) n = _data.getSize();

	// GET DATA
	Array<double> &data = aStateVector->getData();

	// DIVIDE
	int i;
	for(i=0;i<n;i++) {  
		if(data[i]==0.0)	_data[i] = rdMath::NAN;
		else	_data[i] /= data[i];
	}
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print the contents of this StateVector to standard out.
 */
void StateVector::
print()
{
	// TIME
	char format[IO_STRLEN];
	sprintf(format,"t=%s,\t\ty[%%d] =",IO::GetDoubleOutputFormat());
	printf(format,_t,_data.getSize());

	// DATA
	sprintf(format," %s",IO::GetDoubleOutputFormat());
	for(int i=0;i<_data.getSize();i++) {
		printf(format,_data[i]);
	}
	printf("\n");
}
//_____________________________________________________________________________
/**
 * Print the contents of this StateVector to file.
 *
 * The number of characters written to file is returned.  If an error
 * occurs, a negative value is returned.
 */
int StateVector::
print(FILE *fp)
{
	// CHECK FILE POINTER
	if(fp==NULL) {
		printf("StateVector.print(FILE*): null file pointer.\n");
		return(-1);
	}

	// TIME
	char format[IO_STRLEN];
	sprintf(format,"%s",IO::GetDoubleOutputFormat());
	int n=0,nTotal=0;
	n = fprintf(fp,format,_t);
	if(n<0) {
		printf("StateVector.print(FILE*): error writing to file.\n");
		return(n);
	}
	nTotal += n;

	// STATES
	sprintf(format,"\t%s",IO::GetDoubleOutputFormat());
	for(int i=0;i<_data.getSize();i++) {
		n = fprintf(fp,format,_data[i]);
		if(n<0) {
			printf("StateVector.print(FILE*): error writing to file.\n");
			return(n);
		}
		nTotal += n;
	}

	// CARRIAGE RETURN
	n = fprintf(fp,"\n");
	if(n<0) {
		printf("StateVector.print(FILE*): error writing to file.\n");
		return(n);
	}
	nTotal += n;

	return(nTotal);
}
