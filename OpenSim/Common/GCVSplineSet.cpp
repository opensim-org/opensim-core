/* -------------------------------------------------------------------------- *
 *                         OpenSim:  GCVSplineSet.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson
 */


// INCLUDES
#include "common.h"


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
GCVSplineSet::~GCVSplineSet()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
GCVSplineSet::
GCVSplineSet()
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Construct a set of generalized cross-validated splines from file.
 *
 * @param aFileName Name of the file.
 */
GCVSplineSet::
GCVSplineSet(const char *aFileName) :
    FunctionSet(aFileName)
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Construct a set of generalized cross-validated splines based on the states
 * stored in an Storage object.
 *
 * Each column in the Storage object is fit with a spline of the specified
 * degree and is named the name of its corresponding column label.  Note that
 * column labels in the storage object are assumed to be tab delimited.
 *
 * @param aDegree Degree of the constructed splines (1, 3, 5, or 7).
 * @param aStore Storage object.
 * @param aErrorVariance Estimate of the variance of the error in the data to
 * be fit.  If negative, the variance will be estimated.  If 0.0, the fit will
 * try to fit the data points exactly- no smoothing.  If
 * positive, the fits will be smoothed according to the specified variance.
 * The larger the error variance, the more the smoothing.  Note that this is
 * the error variance assumed for each column in the Storage.  If different
 * variances should be set for the various columns, you will need to
 * construct each GCVSpline individually.
 * @see Storage
 * @see GCVSpline
 */
GCVSplineSet::
GCVSplineSet(int aDegree,const Storage *aStore,double aErrorVariance)
{
    setNull();
    if(aStore==NULL) return;
    setName(aStore->getName());

    // CAPACITY
    StateVector *vec = aStore->getStateVector(0);
    if(vec==NULL) return;
    ensureCapacity(2*vec->getSize());

    // CONSTRUCT
    construct(aDegree,aStore,aErrorVariance);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void GCVSplineSet::
setNull()
{
}
//_____________________________________________________________________________
/**
 * Construct a set of generalized cross-validated splines based on the states
 * stored in an Storage object.
 *
 * @param aDegree Degree of the constructed splines (1, 3, 5, or 7).
 * @param aStore Storage object.
 * @param aErrorVariance Error variance for the data.
 */
void GCVSplineSet::
construct(int aDegree,const Storage *aStore,double aErrorVariance)
{
    if(aStore==NULL) return;

    // DESCRIPTION
    setDescription(aStore->getDescription());

    // GET COLUMN NAMES
    const Array<std::string> &labels = aStore->getColumnLabels();
    char tmp[32];
    std::string name;

    // LOOP THROUGH THE STATES
    int nTime=1,nData=1;
    double *times=NULL,*data=NULL;
    GCVSpline *spline;
    //printf("GCVSplineSet.construct:  constructing splines...\n");
    for(int i=0;nData>0;i++) {

        // GET TIMES AND DATA
        nTime = aStore->getTimeColumn(times,i);
        nData = aStore->getDataColumn(i,data);

        // CHECK
        if(nTime!=nData) {
            std::cout << "\nGCVSplineSet.construct: ERR- number of times (" << nTime << ")"
                  << " and number of data (" << nData << ") don't agree.\n";
            break;
        }
        if(nData==0) break;

        // GET COLUMN NAME
        // Note that state i is in column i+1
        if(i+1 < labels.getSize()) {
            name = labels[i+1];
        } else {
            sprintf(tmp,"data_%d",i);
            name = tmp;
        }

        // CONSTRUCT SPLINE
        //printf("%s\t",name);
        spline = new GCVSpline(aDegree,nData,times,data,name,aErrorVariance);
        SimTK::Function* fp = spline->createSimTKFunction();
        delete fp;  

        // ADD SPLINE
        adoptAndAppend(spline);
    }
    //printf("\n%d splines constructed.\n\n",i);

    // CLEANUP
    if(times!=NULL) delete[] times;
    if(data!=NULL) delete[] data;
}


//=============================================================================
// SET AND GET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the function at a specified index.
 *
 * @param aIndex Index of the desired function:  0 <= aIndex < getSize().
 * @return Function at index aIndex.  If aIndex is not value NULL is returned.
 */
GCVSpline* GCVSplineSet::
getGCVSpline(int aIndex) const
{
    GCVSpline& func = (GCVSpline&)get(aIndex);
    return(&func);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a storage object (see Storage) for this spline \set\ or for some
 * derivative of this spline set.
 *
 * @param aDerivOrder Derivative order.  0 constructs from the spline,
 * 1 constructs from the first derivative of the spline, 2 constructs from
 * the second derivative of the spline, etc.
 * @param aDX Spacing of the data points in the independent variable.  If
 * negative the spacing of the independent variable is taken from the
 * original data, as determined from the first non-NULL spline in the set.
 * aDX has a default value of -1.
 * @return Storage object.  If a valid storage object cannot be constructed
 * NULL is returned.
 * @see Storage
 */
Storage* GCVSplineSet::
constructStorage(int aDerivOrder,double aDX)
{
    if(aDerivOrder<0) return(NULL);
    if(getSize()<=0) return(NULL);

    // GET FIRST NON-NULL SPLINE
    GCVSpline *spl;
    int n = getSize();
    for(int i=0;i<n;i++) {
        spl = getGCVSpline(i);
        if(spl!=NULL) break;
    }
    if(spl==NULL) return(NULL);

    // HOW MANY X STEPS
    double xRange = getMaxX() - getMinX();
    int nSteps;
    if(aDX<=0.0) {
        nSteps = spl->getSize();
    } else {
        nSteps = 10 + (int)(xRange/aDX);
    }

    // CONSTRUCT STORAGE OBJECT
    std::string name="";
    if(aDerivOrder==0) {
        name=getName()+"_GCVSpline";
    } else {
        char temp[10];
        sprintf(temp, "%d", aDerivOrder);
        name=getName()+"_GCVSpline_Deriv_"+std::string(temp);
    }
    Storage *store = new Storage(nSteps,name);

    // DESCRIPTION
    store->setDescription(getDescription());

    // SET COLUMN LABELS
    GCVSpline *spline;
    Array<std::string> labels;
    labels.append("time");
    for(int i=0;i<n;i++) {
        spline = getGCVSpline(i);
        if(spline==NULL) {
            char cName[32];
            sprintf(cName,"data_%d",i);
            labels.append(std::string(cName));
        } else {
            labels.append(spline->getName());
        }
    }
    store->setColumnLabels(labels);

    // SET STATES
    Array<double> y(0.0,n);

    // LOOP THROUGH THE DATA
    // constant increments
    if(aDX>0.0) {
        for(double x=getMinX(); x<=getMaxX(); x+=aDX) {
            evaluate(y,aDerivOrder,x);
            store->append(x,n,&y[0]);
        }

    // original independent variable increments
    } else {

        const Array<double> &xOrig = spl->getX();
        for(int ix=0;ix<nSteps;ix++) {

            // ONLY WITHIN BOUNDS OF THE SET
            if(xOrig[ix]<getMinX()) continue;
            if(xOrig[ix]>getMaxX()) break;

            evaluate(y,aDerivOrder,xOrig[ix]);
            store->append(xOrig[ix],n,&y[0]);
        }
    }

    return(store);
}

double GCVSplineSet::getMinX() const
{
    double min = SimTK::Infinity;

    for (int i=0; i<getSize(); i++) {
        const GCVSpline* spl = getGCVSpline(i);
        if (spl && spl->getMinX() < min)
            min = spl->getMinX();
    }

    return min;
}

double GCVSplineSet::getMaxX() const
{
    double max = -SimTK::Infinity;

    for (int i=0; i<getSize(); i++) {
        const GCVSpline* spl = getGCVSpline(i);
        if (spl && spl->getMaxX() > max)
            max = spl->getMaxX();
    }

    return max;
}
