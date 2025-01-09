#ifndef OPENSIM_GCV_SPLINE_SET_H_
#define OPENSIM_GCV_SPLINE_SET_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  GCVSplineSet.h                          *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

// INCLUDES
#include "osimCommonDLL.h"
#include "Object.h"
#include "FunctionSet.h"
#include "TimeSeriesTable.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

class GCVSpline;
class Storage;

/**
 * A class for holding a set of generalized cross-validated splines.
 *
 * @see GCVSpline
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API GCVSplineSet : public FunctionSet {
OpenSim_DECLARE_CONCRETE_OBJECT(GCVSplineSet, FunctionSet);
public:
    GCVSplineSet();

    /**
    * Construct a set of generalized cross-validated splines from file.
    *
    * @param aFileName Name of the file.
    */
    GCVSplineSet(const char *aFileName);

    /**
     * Construct a set of generalized cross-validated splines based on the 
     * states stored in an Storage object.
     *
     * Each column in the Storage object is fit with a spline of the specified
     * degree and is named the name of its corresponding column label. Note that
     * column labels in the storage object are assumed to be tab delimited.
     *
     * @param aDegree Degree of the constructed splines (1, 3, 5, or 7).
     * @param aStore Storage object.
     * @param aErrorVariance Estimate of the variance of the error in the data 
     * to be fit.  If negative, the variance will be estimated.  If 0.0, the 
     * fit will try to fit the data points exactly- no smoothing.  If
     * positive, the fits will be smoothed according to the specified variance.
     * The larger the error variance, the more the smoothing.  Note that this is
     * the error variance assumed for each column in the Storage.  If different
     * variances should be set for the various columns, you will need to
     * construct each GCVSpline individually.
     * @see Storage
     * @see GCVSpline
     */
    GCVSplineSet(int aDegree,const Storage *aStore,double aErrorVariance=0.0);

    /**
     * Construct a set of generalized cross-validated splines based on the 
     * states stored in a TimeSeriesTable.
     *
     * Each column in the TimeSeriesTable is fit with a spline of the specified
     * degree and is named the name of its corresponding column label.  
     *
     * @param table TimeSeriesTable object.
     * @param labels Columns to use from TimeSeriesTable.
     * @param degree Degree of the constructed splines (1, 3, 5, or 7).
     * @param errorVariance Estimate of the variance of the error in the data 
     * to be fit.  If negative, the variance will be estimated.  If 0.0, the 
     * fit will try to fit the data points exactly- no smoothing.  If
     * positive, the fits will be smoothed according to the specified variance.
     * The larger the error variance, the more the smoothing.  Note that this is
     * the error variance assumed for each column in the TimeSeriesTable.  If 
     * different variances should be set for the various columns, you will need 
     * to construct each GCVSpline individually.
     * @see TimeSeriesTable.
     * @see GCVSpline
     */
    GCVSplineSet(const TimeSeriesTable& table,
                 const std::vector<std::string>& labels = {},
                 int degree                             = 5,
                 double errorVariance                   = 0.0);
    virtual ~GCVSplineSet();

private:
    /**
     * Set all member variables to NULL values.
     */
    void setNull();

    /**
     * Construct a set of generalized cross-validated splines based on the 
     * states stored in an Storage object.
     *
     * @param aDegree Degree of the constructed splines (1, 3, 5, or 7).
     * @param aStore Storage object.
     * @param aErrorVariance Error variance for the data.
     */
    void construct(int aDegree,const Storage *aStore,double aErrorVariance);

public:
    /**
     * Get the function at a specified index.
     *
     * @param aIndex Index of the desired function:  0 <= aIndex < getSize().
     * @return Function at index aIndex.  If aIndex is not a valid value NULL is
     *         returned.
     */
    GCVSpline* getGCVSpline(int aIndex) const;
    double getMinX() const;
    double getMaxX() const;

    /**
     * Construct a storage object (see Storage) for this spline set or for 
     * some derivative of this spline set.
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
    Storage* constructStorage(int aDerivOrder,double aDX=-1);

};  // END class GCVSplineSet

}; //namespace

#endif // OPENSIM_GCV_SPLINESET_H_
