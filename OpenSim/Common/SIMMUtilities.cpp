// SIMMUtilities.cpp
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Object.h"
#include "IO.h"
#include "SIMMUtilities.h"


//=============================================================================
// EXPORTED STATIC CONSTANTS
//=============================================================================


using namespace OpenSim;


//=============================================================================
// BONES
//=============================================================================
//_____________________________________________________________________________
/**
 * Load a SIMM bone from file.  The file must be in ASCII format.
 *
 * Memory for the vertices, normals, face counts, and faces is allocated.  The
 * caller is responsible for deleting this memory.
 *
 * @param aFileName File name.
 * @param rBoundingBox Bounding box (xmin, xmax, ymin, ymax, zmin, zmax).
 * @param rNumVerts Number of vertices.
 * @param rVerts Vertices.
 * @param rNormals Normals at the vertices.
 * @param rNumFaces Number of faces or polygons.
 * @param rFaceCounts Number of vertices in each face.
 * @param rFaces List of vertices in each face.
 * @return Total number of face connections.
 */
int SIMMUtilities::
LoadBone(const char *aFileName,double rBoundingBox[6],
			int &rNumVerts,double *&rVerts,double *&rNormals,
			int &rNumFaces,int *&rFaceCounts,int *&rFaces)
{

	// CHECK FILE NAME
	if(aFileName==NULL) return(0);
	if(strlen(aFileName)<=0) return(0);

	// OPEN FILE
	FILE *file = IO::OpenFile(aFileName,"r");
	if(file==NULL) return(0);

	// READ HEADER
	char header[Object::NAME_LENGTH];
	fscanf(file,"%s",header);
	if(strcmp(header,"NORM_ASCII")!=0) {
		fclose(file);
		return(0);
	}
	printf("SIMMUtilities.LoadBone: Reading bone file %s.\n",aFileName);

	// NUMBER OF VERTICES AND FACES
	fscanf(file,"%d %d",&rNumVerts,&rNumFaces);
	if((rNumVerts<=0)||(rNumFaces<=0)) {
		fclose(file);
		return(0);
	}

	// BOUNDING BOX
	fscanf(file,"%lf %lf",&rBoundingBox[0],&rBoundingBox[1]);
	fscanf(file,"%lf %lf",&rBoundingBox[2],&rBoundingBox[3]);
	fscanf(file,"%lf %lf",&rBoundingBox[4],&rBoundingBox[5]);

	// ALLOCATE MEMORY
	rVerts = new double[3*rNumVerts];
	rNormals = new double[3*rNumVerts];
	rFaceCounts = new int[rNumFaces];
	int **faces = new int*[rNumFaces];

	// READ VERTICES AND NORMALS
	int i,I;
	for(I=i=0;i<rNumVerts;i++,I+=3) {
		fscanf(file,"%lf %lf %lf,",&rVerts[I+0],&rVerts[I+1],&rVerts[I+2]);
		fscanf(file,"%lf %lf %lf,",&rNormals[I+0],&rNormals[I+1],&rNormals[I+2]);
	}

	// READ FACE COUNTS AND FACES
	int j,total=0;
	for(i=0;i<rNumFaces;i++) {

		// COUNTS
		faces[i] = NULL;
		fscanf(file,"%d",&rFaceCounts[i]);
		if(rFaceCounts[i]<=0) continue;

		// FACES
		total += rFaceCounts[i];
		faces[i] = new int[rFaceCounts[i]];
		for(j=0;j<rFaceCounts[i];j++) {
			fscanf(file,"%d",&faces[i][j]);
		}
	}

	// CONCATENATE FACES
	rFaces = new int[total];
	for(I=i=0;i<rNumFaces;i++) {
		if(faces[i]==NULL) continue;
		for(j=0;j<rFaceCounts[i];j++,I++) {
			rFaces[I] = faces[i][j];
		}
		delete[] faces[i];
	}
	delete[] faces;

	printf("SIMMUtilities.LoadBone: %d face connections.\n",total);
	return(total);
}

