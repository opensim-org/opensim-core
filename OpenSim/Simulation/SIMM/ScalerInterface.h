#ifndef _ScalerInterface_h_
#define _ScalerInterface_h_

#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Array.h>
namespace OpenSim { 

class Model;
class suBody;
class suJoint;
class ScaleSet;
class simmScalingOptions;

// ScalerInterface.h
// Author: Ayman Habib
/* Copyright (c) 2005, Stanford University and Ayman Habib
 * 
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

class RDSIMULATION_API ScalerInterface 
{
protected:
	Model& _theModel;
public:
	ScalerInterface(Model &aModel):
		_theModel(aModel)
		{}
	virtual bool scaleModel(const ScaleSet& scaleSet, bool aPreserveMassDist, double aFinalMass)
	{
		std::cout<< "Error, ScalerInterface::scaleModel() - not implemented.\n";
		return false;
	};
	
};

}; //namespace
#endif // __ScalerInterface_h__


