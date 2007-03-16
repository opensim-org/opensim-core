// Memory.cpp
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
#include "rdTools.h"
#include "Memory.h"


// CONSTANTS


// STATICS


//=============================================================================
// STRING ARRAYS
//=============================================================================
//-----------------------------------------------------------------------------
// SCIENTIFIC
//-----------------------------------------------------------------------------
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Allocate a new array of strings.
 *
 * @param aNumStrings Number of strings.
 * @param aStringLength Length of each string.
 * @return Pointer to the array of strings.
 */
char** Memory::
NewArrayOfStrings(int aNumStrings,int aStringLength)
{
	if(aNumStrings<=0) return(NULL);
	if(aStringLength<=0) return(NULL);

	// ALLOCATE ARRAY
	char **array = new char*[aNumStrings];
	if(array==NULL) {
		printf("Memory.NewStringArray: ERROR- array allocation failed.\n");
		return(NULL);
	}

	// ALLOCATE STRINGS
	int i,j;
	for(i=0;i<aNumStrings;i++) {
		array[i] = new char[aStringLength];
		if(array[i]==NULL) {
			printf("Memory.NewStringArray: ERROR- string allocation failed.\n");
			for(j=0;j<i;j++) { delete[] array[j];  array[j]=NULL;}
			return(NULL);
		}
	}

	return(array);
}
//_____________________________________________________________________________
/**
 * Delete an array of pointers.  Note that both the pointers held by the
 * array and the array itself are deleted.
 *
 * @param aNumStrings Number of strings held in the array.
 * @param aStringArray Pointer to the array of strings.
 */
void Memory::
DeleteArrayOfStrings(int aNumStrings,char **aArray)
{
	if(aNumStrings<=0) return;
	if(aArray==NULL) return;

	// DELETE STRINGS
	int i;
	for(i=0;i<aNumStrings;i++) {
		if(aArray[i]!=NULL) {
			delete[] aArray[i];
			aArray[i]=NULL;
		}
	}

	// DELETE ARRAY
	delete[] aArray;
	aArray = NULL;
}
