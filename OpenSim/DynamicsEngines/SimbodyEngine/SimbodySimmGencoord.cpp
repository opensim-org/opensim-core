// SimbodySimmGencoord.cpp
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>

#include <OpenSim/Simulation/Model/AbstractCoordinate.h>

#include "SimbodySimmGencoord.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define MIN(a,b) ((a)<=(b)?(a):(b))

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodySimmGencoord::~SimbodySimmGencoord()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodySimmGencoord::SimbodySimmGencoord()
{
}

//_____________________________________________________________________________
/**
 * Constructor from a coordinate.
 *
 * @param aCoordinate Coordinate that this gencoord is based on.
 */
SimbodySimmGencoord::SimbodySimmGencoord(const AbstractCoordinate* aCoordinate,
													  int aRestraintFuncUserNumber,
													  int aMinRestraintFuncUserNumber,
													  int aMaxRestraintFuncUserNumber)
{
   _coordinate = aCoordinate;
	_restraintFuncUserNumber = aRestraintFuncUserNumber;
	_minRestraintFuncUserNumber = aMinRestraintFuncUserNumber;
	_maxRestraintFuncUserNumber = aMaxRestraintFuncUserNumber;
}

//_____________________________________________________________________________
/**
 * Write the gencoord to a [SIMM joint] file.
 *
 * @param aStream File to write to.
 */
void SimbodySimmGencoord::write(ofstream& aStream)
{
   aStream << "begingencoord " << _coordinate->getName() << endl;
   double conversion = 1.0;
   if (_coordinate->getMotionType() == AbstractTransformAxis::Rotational)
      conversion = 180.0 / SimTK::Pi;
   aStream << "range " << _coordinate->getRangeMin() * conversion << " "
      << _coordinate->getRangeMax() * conversion << endl;
   aStream << "default_value " << _coordinate->getDefaultValue() * conversion << endl;
   aStream << "tolerance " << _coordinate->getTolerance() * conversion << endl;
   aStream << "PD_stiffness " << _coordinate->getStiffness() << endl;
	aStream << "clamped " << (_coordinate->getClamped() ? "yes" : "no") << endl;
	aStream << "locked " << (_coordinate->getLocked() ? "yes" : "no") << endl;
	const Array<std::string>& keys = _coordinate->getKeys();
	if (keys.getSize() > 0)
	{
		aStream << "keys ";
		
		for (int i = 0; i < MIN(2, keys.getSize()); i++)
			aStream << keys[i] << " ";
		aStream << endl;
	}
	if (_restraintFuncUserNumber >= 0)
		aStream << "restraint f" << _restraintFuncUserNumber << endl;
	if (_minRestraintFuncUserNumber >= 0)
		aStream << "minrestraint f" << _minRestraintFuncUserNumber << endl;
	if (_maxRestraintFuncUserNumber >= 0)
		aStream << "maxrestraint f" << _maxRestraintFuncUserNumber << endl;
	aStream << "active " << (_coordinate->isRestraintActive() ? "yes" : "no") << endl;

   aStream << "endgencoord" << endl << endl;
}

#if 0
	int RFIndex = -1, minRFIndex = -1, maxRFIndex = -1;
	double conversion;

	if (aCoordinate.getMotionType() == AbstractTransformAxis::Rotational)
		conversion = SimTK_RADIAN_TO_DEGREE;
	else
		conversion = 1.0;

	aStream << "begingencoord " << aCoordinate.getName() << endl;
	aStream << "default_value " << aCoordinate.getDefaultValue() * conversion << endl;
	aStream << "range " << aCoordinate.getRangeMin() * conversion << " " << aCoordinate.getRangeMax() * conversion << endl;
	aStream << "tolerance " << aCoordinate.getTolerance() << endl;
	aStream << "pd_stiffness " << aCoordinate.getStiffness() << endl;

	SimmCoordinate* sc = dynamic_cast<SimmCoordinate*>(&aCoordinate);

	/* keys are for SimmCoordinates only */
	if (sc)
	{
		const Array<std::string>& keys = sc->getKeys();
		if (keys.getSize() > 0)
		{
			aStream << "keys ";
			for (int i = 0; i < MIN(2, keys.getSize()); i++)
				aStream << keys[i] << " ";
			aStream << endl;
		}
	}

	aStream << "clamped " << ((aCoordinate.getClamped()) ? ("yes") : ("no")) << endl;
	aStream << "locked " << ((aCoordinate.getLocked()) ? ("yes") : ("no")) << endl;

	/* Restraint functions are for SimmCoordinates only. */
	if (sc)
	{
		aStream << "active " << ((sc->isRestraintActive()) ? ("yes") : ("no")) << endl;

		if (sc->getRestraintFunction())
		{
			RFIndex = aFunctionIndex++;
			aStream << "restraint f" << RFIndex << endl;
		}

		if (sc->getMinRestraintFunction())
		{
			minRFIndex = aFunctionIndex++;
			aStream << "minrestraint f" << minRFIndex << endl;
		}

		if (sc->getMaxRestraintFunction())
		{
			maxRFIndex = aFunctionIndex++;
			aStream << "maxrestraint f" << maxRFIndex << endl;
		}
	}

	aStream << "endgencoord" << endl << endl;

	if (sc)
	{
		Function* func;
		NatCubicSpline* spline;

		if (RFIndex >= 0)
		{
			func = sc->getRestraintFunction();
			if ((spline = dynamic_cast<NatCubicSpline*>(func)))
			{
				aStream << "beginfunction f" << RFIndex << endl;
				for (int i = 0; i < spline->getNumberOfPoints(); i++)
					aStream << "(" << spline->getX()[i] * conversion << ", " << spline->getY()[i] << ")" << endl;
				aStream << "endfunction" << endl << endl;
			}
		}

		if (minRFIndex >= 0)
		{
			func = sc->getMinRestraintFunction();
			if ((spline = dynamic_cast<NatCubicSpline*>(func)))
			{
				aStream << "beginfunction f" << minRFIndex << endl;
				for (int i = 0; i < spline->getNumberOfPoints(); i++)
					aStream << "(" << spline->getX()[i] * conversion << ", " << spline->getY()[i] << ")" << endl;
				aStream << "endfunction" << endl << endl;
			}
		}

		if (maxRFIndex >= 0)
		{
			func = sc->getMaxRestraintFunction();
			if ((spline = dynamic_cast<NatCubicSpline*>(func)))
			{
				aStream << "beginfunction f" << maxRFIndex << endl;
				for (int i = 0; i < spline->getNumberOfPoints(); i++)
					aStream << "(" << spline->getX()[i] * conversion << ", " << spline->getY()[i] << ")" << endl;
				aStream << "endfunction" << endl << endl;
			}
		}
	}
#endif
