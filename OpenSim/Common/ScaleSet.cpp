// INCLUDES
#include "ScaleSet.h"



using namespace OpenSim;
using namespace std;
//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ScaleSet::~ScaleSet(void)
{}

//_____________________________________________________________________________
/**
 * Constructor of a scaleSet from a file.
 */
ScaleSet::ScaleSet(const string& scalesFileName):
Set<Scale>(scalesFileName)
{
	setNull();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Default constructor of a scaleSet.
 */
ScaleSet::ScaleSet():
Set<Scale>()
{
	setNull();
}

void ScaleSet::setNull()
{
	setType("ScaleSet");
	setName("ScaleSet");
}
