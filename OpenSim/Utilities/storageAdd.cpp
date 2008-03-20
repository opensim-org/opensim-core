// storageAdd.cpp
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

// DEFINES


using namespace OpenSim;
#define MAXLEN 2048

//______________________________________________________________________________
/**
 * Add two storage instances together.  The output storage is called
 * "storageAdd.out" unless an output namne is specified on the command line.
 * Output contains state vectors that occur at the same times
 * as the state vectors in the first argument.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments:  storageAdd file1 file2 [outName]
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	char store1Name[MAXLEN],store2Name[MAXLEN],outName[MAXLEN];
	if((argc!=3)&&(argc!=4)) {
		printf("Usage:  storageAdd store1 store2 [outName]\n");
		exit(-1);
	} else {
		strcpy(store1Name,argv[1]);
		strcpy(store2Name,argv[2]);
		if(argc==4) {
			strcpy(outName,argv[3]);
		} else {
			strcpy(outName,"storageAdd.out");
		}
	}

	// LOAD DATA
	printf("Loading data from files %s and %s\n",store1Name,store2Name);
	Storage *store1 = new Storage(store1Name);
	Storage *store2 = new Storage(store2Name);

	// ADD
	store1->add(store2);

	// WRITE
	store1->print(outName,"w");	

	// CLEANUP
	delete store1;
	delete store2;

	return(0);
}
