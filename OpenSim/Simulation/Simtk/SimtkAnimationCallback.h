#ifndef _SimtkAnimationCallback_h_
#define _SimtkAnimationCallback_h_
// SimtkAnimationCallback.h
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Simulation/Model/IntegCallback.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
//=============================================================================
/**
 * A class for generating an animation sequence in Simtk platform based on a
 * forward dynamic simulation.
 *
 * @author Ayman Habib (After rdmAnimationCallback)
 * @version 1.0
 */
namespace OpenSim { 

class OSIMSIMULATION_API SimtkAnimationCallback : public IntegCallback
{
//=============================================================================
// DATA
//=============================================================================
protected:
	
	Transform*		_transforms;
	/** Current simulation time for feedback purposes */
	double			_currentTime;
	double*			_offsets;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	// A Factory method to create the callback, so that it's created on the C++ side
	// and memory menagement on the Java side is side-stepped
	static SimtkAnimationCallback* CreateAnimationCallback(Model *aModel);
protected:
	SimtkAnimationCallback(Model *aModel);
	virtual ~SimtkAnimationCallback();
private:
	void setNull();
public:

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	const double getCurrentTime() const;
	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual int
		step(double *aXPrev,double *aYPrev,int aStep,double aDT,double aT,
		double *aX,double *aY,void *aClientData=NULL);
	virtual int
		begin(int aStep,double aDT,double aT,
		double *aX,double *aY,void *aClientData=NULL)
	{
		_currentTime=aDT;
		return 0;
	}
	const Transform* getBodyTransform(int bodyIndex) const;

	void extractOffsets(Model& displayModel);
public:
	// Load transforms vector from KinematicsEngine
	void getTransformsFromKinematicsEngine(Model& simmModel);
	// Synchronization stuff
	// Implementation of Peterson's Algorithm that doesn't work!
/*
	static int _turn;
	static bool _flag[2];
public:
	void mutex_begin(int i) { 
		int j = 1-i; // i=me, j=other 
		_flag[i] = true;                         
		_turn = j;                               
		//std::cout << "In mutex_begin:" << i << j 
		//	<< _flag[0] << _flag[1] 
		//	<< _turn << std::endl;
		while (_flag[j] && _turn==j) ;// skip 
		//std::cout << "Leave mutex_begin:" << i << std::endl;

	}
	void mutex_end(int i){
		_flag[i] = false;                         
	}
*/
//=============================================================================
};	// END of class SimtkAnimationCallback

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimtkAnimationCallback_h__


