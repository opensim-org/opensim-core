// SIMMUtilities.cpp
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
const double SIMMUtilities::PI = acos(-1.0);


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

