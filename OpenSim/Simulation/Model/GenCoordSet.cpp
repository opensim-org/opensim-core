#include "GenCoordSet.h"
#include "GenCoord.h"
#include <OpenSim/Tools/Range.h>
#include <OpenSim/Tools/Set.h>
//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
GenCooSet::~GenCooSet(void)
{}
//_____________________________________________________________________________
/**
 * Constructor from a file
 */
GenCooSet::GenCooSet(const std::string &aFileName):
Set<GenCoord>(aFileName)
{
}
//_____________________________________________________________________________
/**
 * Default constructor
 */
GenCooSet::GenCooSet():
Set<GenCoord>()
{
}
//_____________________________________________________________________________
/** 
 * Get all range mins in an Array 
 **/
void GenCooSet::
getAllMins(Array<double>& aAllMins) const
{
	for(int i=0; i < getSize(); i++){
		aAllMins.append(get(i)->getRange().getMin());
	}
}
//_____________________________________________________________________________
/** 
 * Get all range maxs in an Array 
 **/
void GenCooSet::
getAllMaxs(Array<double>& aAllMaxs) const
{
	for(int i=0; i < getSize(); i++){
		aAllMaxs.append(get(i)->getRange().getMax());
	}
}
//_____________________________________________________________________________
/** 
 * Get all GenCoord names in an Array 
 **/
void GenCooSet::
getAllNames(Array<std::string>& aAllNames) const
{
	for(int i=0; i < getSize(); i++){
		aAllNames.append(get(i)->getName());
	}

}
