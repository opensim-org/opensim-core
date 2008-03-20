// storageIntegrate.cpp
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

// INCLUDES
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>

// DEFINES


using namespace OpenSim;
#define MAXLEN 2048

//______________________________________________________________________________
/**
 * Integrate a storage instance.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:  storageIntegrate file [outName]
 * [ti] [tf]
 */
int main(int argc,char **argv)
{
	IO::SetPrecision(16);

	// PARSE COMMAND LINE
	char storeName[MAXLEN],outName[MAXLEN];
	double tStart,tEnd;
	if((argc!=2)&&(argc!=3)&&(argc!=5)) {
		printf("Usage:  storageIntegrate file [outName] [ti] [tf]\n");
		exit(-1);
	} else {
		strcpy(storeName,argv[1]);
		if((argc==3)||(argc==5)) {
			strcpy(outName,argv[2]);
		} else {
			strcpy(outName,"storageIntegrate.out");
		}
		if(argc==5){
			sscanf(argv[3],"%lf",&tStart);
			sscanf(argv[4],"%lf",&tEnd);
		}
	}

	// LOAD DATA
	printf("Loading data from file %s.\n",storeName);
	Storage *store = new Storage(storeName);

	// GET START AND END INTEGRATION TIMES IF NONE WERE ENTERED
	if(argc!=5){
		tStart = store->getFirstTime();
		tEnd = store->getLastTime();
	}

	// INTEGRATE
	Storage *result = store->integrate(tStart,tEnd);
	
	// WRITE
	result->print(outName);	

	// CLEANUP
	delete store;
	delete result;

	return(0);
}
