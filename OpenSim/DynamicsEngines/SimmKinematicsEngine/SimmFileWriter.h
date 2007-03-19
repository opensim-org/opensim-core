#ifndef __SimmFileWriter_h__
#define __SimmFileWriter_h__

// SimmFileWriter.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

// INCLUDE
#include <iostream>
#include <fstream>
#include <string>
#include "osimSimmKinematicsEngineDLL.h"

#ifdef SWIG
	#ifdef OSIMSIMMKINEMATICSENGINE_API
		#undef OSIMSIMMKINEMATICSENGINE_API
		#define OSIMSIMMKINEMATICSENGINE_API
	#endif
#endif

namespace OpenSim {

class AbstractModel;
class AbstractMuscle;
class AbstractBody;
class AbstractJoint;
class AbstractCoordinate;
class MarkerSet;

//=============================================================================
//=============================================================================
/**
 * A class for writing SIMM joint and muscle files for any AbstractModel.
 *
 * @author Peter Loan
 * @version 1.0
 */
class  OSIMSIMMKINEMATICSENGINE_API SimmFileWriter
{

//=============================================================================
// DATA
//=============================================================================
protected:
	AbstractModel *_model;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmFileWriter();
	SimmFileWriter(AbstractModel *aModel);
	virtual ~SimmFileWriter();

	bool writeJointFile(const std::string& aFileName) const;
	bool writeMuscleFile(const std::string& aFileName) const;

private:
	bool writeBody(AbstractBody& aBody, const MarkerSet* aMarkerSet, std::ofstream& aStream) const;
	void writeWrapObjects(AbstractBody& aBody, std::ofstream& aStream) const;
	bool writeJoint(AbstractJoint& aJoint, int& aFunctionIndex, std::ofstream& aStream) const;
	bool writeCoordinate(AbstractCoordinate& aCoordinate, int& aFunctionIndex, std::ofstream& aStream) const;
	bool writeMuscle(AbstractMuscle& aMuscle, std::ofstream& aStream) const;
	const std::string& getGravityLabel(double aGravity[3]) const;
//=============================================================================
};	// END of class SimmFileWriter
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmFileWriter_h__


