/* -------------------------------------------------------------------------- *
 *                         OpenSim:  StateVector.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
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
StateVector::StateVector(double aT) :
    StateVector(aT, SimTK::Vector_<double>()) {
    // No operation.
}

//_____________________________________________________________________________
/**
 * Create a StateVector with a time-stamp and an array of values.
 *
 * @param aT Time-stamp of the state-vector.
 * @param data Array of values to set the state-vector to.
 */
StateVector::StateVector(double aT, const SimTK::Vector_<double>& data) :
    _data(0.0)
{
    // INITIAL VALUES
    setNull();

    // SET THE DATA
    setStates(aT, data);
}

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
 * 
 * @param aT Time-stamp of the state vector.
 * @param data Array of values to set the state to.
 */
void StateVector::
setStates(double aT, const SimTK::Vector_<double>& data) {
    _t = aT;
    _data.setSize(data.size());
    int size = _data.getSize();
    for(int i = 0; i < size; ++i) {
        _data[i] = data[i];
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
 * @param values Array of values to add to this state-vector.
 */
void StateVector::
add(const SimTK::Vector_<double>& values) {
    if(values.size() == 0)
        return;
    int i, n = values.size();
    if( n > _data.getSize())
        n = _data.getSize();
    for(i = 0; i < n; ++i)
        _data[i] += values[i];
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
 * @param values Array of values to subtract from this state-vector.
 */
void StateVector::subtract(const SimTK::Vector_<double>& values) {
    if(values.size() == 0)
        return;
    int i, n = values.size();
    if(n > _data.getSize())
        n = _data.getSize();
    for(i = 0; i < n; ++i)
        _data[i] -= values[i];
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
 * @param values Array of values to multiply this state-vector with.
 */
void StateVector::multiply(const SimTK::Vector_<double>& values) {
    if(values.size() == 0)
        return;
    int i, n = values.size();
    if(n > _data.getSize())
        n = _data.getSize();
    for(i = 0; i < n; ++i)
        _data[i] *= values[i];
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
        log_error("StateVector.divide: divide by zero.");
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
 * @param values Array of values the states are divided by.
 */
void StateVector::divide(const SimTK::Vector_<double>& values) {
    if(values.size() == 0)
        return;
    int i, n = values.size();
    if(n > _data.getSize())
        n = _data.getSize();
    for(i = 0; i < n; ++i) {  
        if(values[i] == 0.0)
            _data[i] = SimTK::NaN;
        else
            _data[i] /= values[i];
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
        if(data[i]==0.0)    _data[i] = SimTK::NaN;
        else    _data[i] /= data[i];
    }
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print the contents of this StateVector to file.
 *
 * The number of characters written to file is returned.  If an error
 * occurs, a negative value is returned.
 */
int StateVector::
print(FILE *fp) const
{
    // CHECK FILE POINTER
    if(fp==NULL) {
        log_error("StateVector.print(FILE*): null file pointer.");
        return(-1);
    }

    // TIME
    char format[IO_STRLEN];
    sprintf(format,"%s",IO::GetDoubleOutputFormat());
    int n=0,nTotal=0;
    n = fprintf(fp,format,_t);
    if(n<0) {
        log_error("StateVector.print(FILE*): error writing to file.");
        return(n);
    }
    nTotal += n;

    // STATES
    sprintf(format,"\t%s",IO::GetDoubleOutputFormat());
    for(int i=0;i<_data.getSize();i++) {
        n = fprintf(fp,format,_data[i]);
        if(n<0) {
            log_error("StateVector.print(FILE*): error writing to file.");
            return(n);
        }
        nTotal += n;
    }

    // CARRIAGE RETURN
    n = fprintf(fp,"\n");
    if(n<0) {
        log_error("StateVector.print(FILE*): error writing to file.");
        return(n);
    }
    nTotal += n;

    return(nTotal);
}
