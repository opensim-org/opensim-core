/* -------------------------------------------------------------------------- *
 *                         OpenSim:  GCVSplineSet.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "GCVSplineSet.h"
#include "GCVSpline.h"
#include "Storage.h"


using namespace OpenSim;

GCVSplineSet::~GCVSplineSet() {
    // No operation;
}

GCVSplineSet::GCVSplineSet() {
    setNull();
}

GCVSplineSet::GCVSplineSet(const char *aFileName) :
    FunctionSet(aFileName) {
    setNull();
}
GCVSplineSet::GCVSplineSet(int aDegree,
                           const Storage *aStore,
                           double aErrorVariance) {
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

GCVSplineSet::GCVSplineSet(const TimeSeriesTable& table,
                           const std::vector<std::string>& labels,
                           int degree,
                           double errorVariance) {
    const auto& time = table.getIndependentColumn();
    auto labelsToUse = labels;
    if (labelsToUse.empty()) labelsToUse = table.getColumnLabels();
    for (const auto& label : labelsToUse) {
        const auto& column = table.getDependentColumn(label);
        adoptAndAppend(new GCVSpline(degree, column.size(), time.data(),
                                     &column[0], label, errorVariance));
    }
}

void GCVSplineSet::setNull() {
    // No operation.
}

void GCVSplineSet::construct(int aDegree,
                             const Storage *aStore,
                             double aErrorVariance) {
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
            log_error("GCVSplineSet.construct: number of times ({}) "
                          "and number of data ({}) don't agree.", nTime, nData);
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

GCVSpline* GCVSplineSet::getGCVSpline(int aIndex) const {
    GCVSpline& func = (GCVSpline&)get(aIndex);
    return(&func);
}

Storage* GCVSplineSet::constructStorage(int aDerivOrder,double aDX) {
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
    std::string name = "";
    if(aDerivOrder == 0) {
        name = getName() + "_GCVSpline";
    } else {
        name = fmt::format("{}_GCVSpline_Deriv_{}", getName(), aDerivOrder);
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
