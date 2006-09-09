// INCLUDES
#include "MarkerSet.h"
#include "Model.h"
#include <OpenSim/Tools/ScaleSet.h>
#include "Body.h"



using namespace OpenSim;
using namespace std;
//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
MarkerSet::~MarkerSet(void)
{}

//_____________________________________________________________________________
/**
 * Constructor of a markerSet from a file.
 */
MarkerSet::MarkerSet(const string& markersFileName):
Set<Marker>(markersFileName)
{
	setNull();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Default constructor of a markerSet.
 */
MarkerSet::MarkerSet():
Set<Marker>()
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor of a markerSet.
 */
MarkerSet::MarkerSet(const MarkerSet& aMarkerSet):
Set<Marker>(aMarkerSet)
{
	setNull();
	*this = aMarkerSet;
}

//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
#ifndef SWIG

MarkerSet& MarkerSet::
operator=(const MarkerSet &aMarkerSet)
{
	Set<Marker>::operator=(aMarkerSet);
	return (*this);
}
#endif
//_____________________________________________________________________________
/**
 * Check if the marker set uses weights 
 */
bool MarkerSet::
usesWeights() const
{
	bool wighted = false;

	for(int i=0; i < getSize(); i++){
		Marker* nextMarker = get(i);
		if (nextMarker->getWeight()!= 1.0){
			wighted = true;
			break;
		}
	}

	return wighted;
}

void MarkerSet::setNull()
{
	setType("MarkerSet");
	setName("MarkerSet");
}
//_____________________________________________________________________________
/**
 * Bind markers in the marker set to the model.
 * Works by going through the markers and for each one finding the body associated
 * Currently this function returns true if any marker maps to model and so  
 * it does not give complete feedback as
 * to if there're markers that didn't map to model segments
 * @param model is the model used for binding
 * 
 * @return true if any marker is bound to model else false. 
 */
bool MarkerSet::
bindMarkersToModel(Model *model)
{
	bool bound = false;
	for(int i=0; i < getSize(); i++){
		Marker* nextMarker = get(i);
		const string& refSegmentName = nextMarker->getReferenceSegmentName();
		int bodyIndex = model->getBodyIndex(refSegmentName);
		if (bodyIndex>=0){	// SDFast::GROUND = -1
			nextMarker->setRefSegment(bodyIndex);
			Body *body = model->getBody(bodyIndex);
			body->addObserver(*nextMarker);
			bound = true;
		}
	}
	return bound;
}
//_____________________________________________________________________________
/**
 * Get names of markers in the marker set
 */
void MarkerSet::
getMarkerNames(Array<string>& aMarkerNamesArray)
{
	for(int i=0; i < getSize(); i++){
		Marker* nextMarker = get(i);
		aMarkerNamesArray.append(nextMarker->getName());
	}

}
//_____________________________________________________________________________
/**
 * Scale marker set by a set of scale factors
 */
void MarkerSet::
scale(ScaleSet& scaleSet)
{
	Array<double>	scaleFactors(1.0, 3);
	for(int i=0; i < getSize(); i++){
		Marker* nextMarker = get(i);
		const string& refSegmentName = nextMarker->getReferenceSegmentName();
		bool found = false;
		for (int j=0; j < scaleSet.getSize() && !found; j++){
			Scale* nextScale = scaleSet.get(j);
			if (nextScale->getSegmentName() == refSegmentName){
				found = true;
				nextScale->getScaleFactors(scaleFactors);
				nextMarker->scaleBy(scaleFactors);
			}
		}
	}

}
//_____________________________________________________________________________
/**
 * Allow markers to be presented in geometry frame rather than COM frame
 * and use this function to make marker locations relative to COM.
 *
 * @param model is the model whose segment COMs are to be used
 */
void MarkerSet::
makeRelativeToCom(Model *model)
{
	Array<double>	scaleFactors(1.0, 3);
	Array<double>	loc(0.0, 3);

	for(int i=0; i < getSize(); i++){
		Marker* nextMarker = get(i);
		const int markerSegment = nextMarker->getRefSegment();
		Body *segment = model->getBody(markerSegment);
		double	segmentCom[3];
		segment->getCenterOfMass(segmentCom);

		nextMarker->getLocation(loc);
		// Adjust by Com
		for(int j=0; j <3; j++){
			loc[j] -= segmentCom[j];
		}
		nextMarker->setLocation(loc);
	}
}
//_____________________________________________________________________________
/**
 * Retrieve the set of markers in the set
 */
const Set<Marker> &
MarkerSet::getMarkers() const
{
	return (*this);
}
