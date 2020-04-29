/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ControlSet.cpp                          *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "ControlSet.h"
#include "ControlLinear.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/XMLDocument.h>


using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 *
 * @todo Figure out why the default controls can't be deleted.
 */
ControlSet::~ControlSet()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ControlSet::ControlSet() :
    _ptcMap(-1), _ptpMap(-1)
{
    setNull();
    generateParameterMaps();
}
//_____________________________________________________________________________
/**
 * Construct an control set from file.
 *
 * @param aFileName Name of the file.
 */
ControlSet::ControlSet(const string &aFileName) :
    Set<Control>(aFileName, false), _ptcMap(-1), _ptpMap(-1)
{
    setNull();
    SimTK::Xml::Element e = updDocument()->getRootDataElement(); 
    updateFromXMLNode(e, getDocument()->getDocumentVersion());
    // removeInvalidObjects();
    generateParameterMaps();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSet Analysis set to be copied.
 */
ControlSet::ControlSet(const ControlSet &aSet) :
    Set<Control>(aSet),
    _ptcMap(-1), _ptpMap(-1)
{
    setNull();
    _ptcMap = aSet._ptcMap;
    _ptpMap = aSet._ptpMap;
}

//_____________________________________________________________________________
/**
 * Constructor from a storage
 *
 * @param aStorage to get the data from
 * @param numControls number of columns to convert into controls, 0 for all
 * @param startIndex or columns to convert, default to all
 */
ControlSet::ControlSet(const Storage& aStorage, int numControls, int startIndex)
{
    setNull();
    // LOOP THROUGH LIST
    int j;
    ControlLinear *control;
    int nControls = numControls;
    // If 0 then get all
    if (nControls==0){
        nControls = aStorage.getColumnLabels().getSize()-startIndex-1;  //-1 because ExtractControl assumes no time col.
    }
    for(j=0;j<nControls;j++) {

        // EXTRACT CONTROL NODE
        control = ExtractControl(aStorage,j+startIndex);

        // APPEND ON TO CONTROL SET
        adoptAndAppend(control);
    }

}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the member variables to their null values.
 */
void ControlSet::
setNull()
{
    setName("Control Set");
    setupProperties();
    _ptcMap.setSize(0);
    _ptpMap.setSize(0);
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ControlSet::
setupProperties()
{
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT (=)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this set to another set.
 * This operator makes a complete copy of the specified set; all member
 * variables and objects in the set are copied.  Because all objects are
 * copied, this set takes ownership of the newly allocated objects (i.e.,
 * _memoryOwner is set to true. So, the result is two independent,
 * identical sets, with the possible exception of the _memoryOwner flag.
 *
 * @param aSet Set to be copied.
 * @return Reference to this set.
 */
#ifndef SWIG
ControlSet& ControlSet::
operator=(const ControlSet &aSet)
{   
    // BASE CLASS
    Set<Control>::operator=(aSet);

    // MEMBERS
    _ptcMap = aSet._ptcMap;
    _ptpMap = aSet._ptpMap;

    return(*this);
}
#endif // SWIG



//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// SIZE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the size of the control set.
 *
 * @param aForModelControls Flag indicating whether or not to get the size
 * for only model controls or all controls in the set.  Model controls are
 * controls that are involved in controlling a model; 
 * Example of non-model controls include the final time of a simulation or
 * an initial value for a joint angle; these quantities are for setting up
 * a simulation, they are not involved in controlling a model.  If
 * aForModelControls is true, the number of model controls in the set is
 * returned; if false, the total number of controls in the set is returned.
 */
int ControlSet::
getSize(bool aForModelControls) const
{
    if(!aForModelControls) return( Set<Control>::getSize() );

    int i,n;
    for(n=i=0;i<Set<Control>::getSize();i++) {
        Control& control = get(i);
        if(control.getIsModelControl()) n++;
    }

    return(n);
}

//-----------------------------------------------------------------------------
// CONTROL LIST
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the list of controls of a specified type.
 *
 * This method can be slow, so minimize its use.
 *
 * @param aType Type of the control (e.g., ControlLinear).
 * @param aList Array of indices of the controls of type aType.
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 * @see get()
 */
void ControlSet::
getControlList(const char *aType,Array<int> &rList,bool aForModelControls) const
{
    rList.setSize(0);

    int i;
    int size = getSize(false);
    for(i=0;i<size;i++) {
        Control& control = get(i);
        if(aForModelControls) if(!control.getIsModelControl()) continue;

        if(control.getConcreteClassName()==aType) {
            rList.append(i);
        }
    }
}

//-----------------------------------------------------------------------------
// CONTROL VALUES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the values of the control curves held in this set at a specified time.
 *
 * Normally to modify the value of a control curve setParameterValues()
 * would be used.  The shape of the control curve is determined by the
 * parameters.
 *
 * If parameters exist at the specified time, these parameters are modified
 * to give the control curve the specified value.  Otherwise, new parameters
 * are added to the control curve.
 *
 * @param aT Time at which the value of the control curve is to be set.
 * @param aX Array of control curve values, the length of which should
 * equal the number of controls.
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 * @todo New controls do not appear to be inserting at the correct location
 * in the XML file.  Is this a problem?
 */
void ControlSet::
setControlValues(double aT,const double aX[],
    bool aForModelControls)
{
    int i,j;
    int size = getSize(false);
    for(i=j=0;j<size;j++) {
        Control& control = get(j);
        if(aForModelControls) if(!control.getIsModelControl()) continue;
    
        control.setControlValue(aT,aX[i]);
        i++;
    }
    generateParameterMaps();
}
//_____________________________________________________________________________
/**
 * Set the values of the control curves held in this set at a specified time.
 *
 * Normally to modify the value of a control curve setParameterValues()
 * would be used.  The shape of the control curve is determined by the
 * parameters.
 *
 * If parameters exist at the specified time, these parameters are modified
 * to give the control curve the specified value.  Otherwise, new parameters
 * are added to the control curve.
 *
 * @param aT Time at which the value of the control curve is to be set.
 * @param aX Array of control curve values, the length of which should
 * equal the number of controls.
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 * @todo New controls do not appear to be inserting at the correct location
 * in the XML file.  Is this a problem?
 */
void ControlSet::
setControlValues(double aT,const Array<double> &aX,
    bool aForModelControls)
{
    int i,n;
    int size = getSize(false);
    for(n=i=0;(n<aX.getSize())&&(i<size);i++) {
        Control& control = get(i);
        if(aForModelControls) if(!control.getIsModelControl()) continue;
    
        control.setControlValue(aT,aX[n]);
        n++;
    }
    generateParameterMaps();
}
//_____________________________________________________________________________
/**
 * Get the values of the control curves held in this set at a specified time.
 *
 *  This is the bread-and-butter method of the controls class; it is called
 * repeatedly throughout an integration.
 *
 * @param aT Time at which to get the values of the control curves.
 * @param rX Array of control curve values.
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 */
void ControlSet::
getControlValues(double aT,double rX[],
    bool aForModelControls) const
{
    // GET VALUES
    int i,n;
    int size = getSize(false);
    for(n=i=0;i<size;i++) {
        Control& control = get(i);
        if(aForModelControls) if(!control.getIsModelControl()) continue;
    
        rX[n] = control.getControlValue(aT);
        n++;
    }
}
//_____________________________________________________________________________
/**
 * Get the values of the control curves held in this set at a specified time.
 *
 *  This is the bread-and-butter method of the controls class; it is called
 * repeatedly throughout an integration.
 *
 * @param aT Time at which to get the values of the control curves.
 * @param rX Array of control curve values.
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 */
void ControlSet::
getControlValues(double aT,Array<double> &rX,
    bool aForModelControls) const
{
    // INITIALIZE SIZE
    rX.setSize(0);

    // GET VALUES
    int i;
    int size = getSize(false);
    for(i=0;i<size;i++) {
        Control& control = get(i);
        if(aForModelControls) if(!control.getIsModelControl()) continue;
    
        rX.append(control.getControlValue(aT));
    }
}

//-----------------------------------------------------------------------------
// PARAMETER NUMBER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of control parameters.
 *
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 * @return Number of control parameters.
 */
int ControlSet::
getNumParameters(bool aForModelControls) const
{
    int i,n;
    int size = getSize(false);
    for(n=i=0;i<size;i++) {
        Control& control = get(i);
        if(aForModelControls) if(!control.getIsModelControl()) continue;
        n += control.getNumParameters();
    }
    return(n);
}

//-----------------------------------------------------------------------------
// PARAMETER LISTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a list of parameters for all controls or just the controls that
 * are model controls.  This list can be used to get parameter mins,
 * maxs, and values.
 *
 * @param rList List of parameters.
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 * @see getParameterMins()
 * @see getParameterMaxs()
 * @see getParameterValues()
 */
void ControlSet::
getParameterList(Array<int> &rList,bool aForModelControls) const
{
    rList.setSize(0);

    int i,j,sp,n;
    int size = getSize(false);
    for(sp=i=0;i<size;i++) {
        Control& control = get(i);
        n = control.getNumParameters();
        for(j=0;j<n;j++,sp++) {
            if(aForModelControls) if(!control.getIsModelControl())
                continue;
            rList.append(sp);
        }
    }   
}
//_____________________________________________________________________________
/**
 * Get the list of parameters that affect the control curves at a specified
 * time.  This list can be used to get parameter mins, maxs, and values.
 *
 * @param aT Time in question.
 * @param rList List of parameters.
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 * @see getParameterMins()
 * @see getParameterMaxs()
 * @see getParameterValues()
 */
void ControlSet::
getParameterList(double aT,Array<int> &rList,
    bool aForModelControls) const
{
    rList.setSize(0);

    int i,j,n;
    int size = getSize(false);
    Array<int> list(-1);
    for(n=i=0;i<size;i++) {

        // GET CONTROL
        Control& control = get(i);

        // ACTUATOR CONTROL?
        if(aForModelControls) if(!control.getIsModelControl()) {
            n += control.getNumParameters();
            continue;
        }

        // GET LIST
        control.getParameterList(aT,list);
        for(j=0;j<list.getSize();j++) {
            rList.append(n+list[j]);
        }
        n += control.getNumParameters();
    }   
}
//_____________________________________________________________________________
/**
 * Get the list of parameters that affect the control curves between two
 * specified times and that do NOT affect the control curve below the lower
 * of these two times.
 *
 * This method is useful when solving for a set of controls for a dynamic
 * simulation.  When solving for a set of controls, one always wants to
 * go forward in time.  Therefore, one does not want to change control
 * parameters that affect the control curve at past times.
 *
 * A control parameter is included in the list only if it affects
 * the control curve in the specified time interval AND does NOT
 * affect the control curve below the lower bound of the
 * specified time interval.  So, it is possible that some of the
 * parameters on the returned list could affect the control curve at
 * times greater than the upper bound of the specified time interval.
 *
 * @param aTLower Lower time bound.  The control curves are not affected
 * below this time by any of the returned parameters.
 * @param aTUpper Upper time bound.  The control curves may be affected
 * for times greater than this time.
 * @param rList List of control parameters (their indices to be exact) that
 * affect the curve between aTLower and aTUpper but not before aTLower.
 * @param aForModelControls If true, only model controls are
 * considered.  If false, all controls are considered.
 * @see getParameterMins()
 * @see getParameterMaxs()
 * @see getParameterValues()
 */
void ControlSet::
getParameterList(double aTLower,double aTUpper,Array<int> &rList,
    bool aForModelControls) const
{
    rList.setSize(0);

    int i,j,n;
    int size = getSize(false);
    Array<int> list(-1);
    for(n=i=0;i<size;i++) {

        // GET CONTROL
        Control& control = get(i);

        // ACTUATOR CONTROL?
        if(aForModelControls) if(!control.getIsModelControl()) {
            n += control.getNumParameters();
            continue;
        }

        // GET LIST
        control.getParameterList(aTLower,aTUpper,list);
        for(j=0;j<list.getSize();j++) {
            rList.append(n+list[j]);
        }
        n += control.getNumParameters();
    }   
}

//-----------------------------------------------------------------------------
// PARAMETER MIN AND MAX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the minimum values allowed for the control parameters.
 *
 * @param rMins Array of minimum allowed values.
 * @param aList List of controls.  If NULL, the values for all parameters for
 * all controls in the control set are gotten.  Otherwise, the parameters for
 * the controls in the list are gotten.
 */
void ControlSet::
getParameterMins(Array<double> &rMins,const Array<int> *aList) const
{
    // INITIALIZE SIZE
    rMins.setSize(0);

    // VARIABLE DECLARATIONS
    int i,p;
    int size = getSize(false);

    // NO LIST
    if(aList==NULL) {
        for(i=0;i<size;i++) {
            Control& control = get(i);
            for(p=0;p<control.getNumParameters();p++) {
                rMins.append(control.getParameterMin(p));
            }
        }

    // LIST
    } else {
        int c,sp;
        for(i=0;i<aList->getSize();i++) {

            // PARAMETER
            sp = (*aList)[i];

            // GET CONTROL
            try {
                c = _ptcMap.get(sp);
            } catch(const Exception& x) {
                log_error("Exception: {}", x.getMessage());
                continue;
            }
            Control& control = get(c);
        
            // GET PARAMETER MIN
            p = _ptpMap[sp];
            rMins.append(control.getParameterMin(p));
        }
    }
}
//_____________________________________________________________________________
/**
 * Get the maximum values allowed for the control parameters.
 *
 * @param rMaxs Array of maximum allowed values.
 * @param aList List of controls.  If NULL, the values for all parameters for
 * all controls in the control set are gotten.  Otherwise, the parameters for
 * the controls in the list are gotten.
 */
void ControlSet::
getParameterMaxs(Array<double> &rMaxs,const Array<int> *aList) const
{
    // INITIALIZE SIZE
    rMaxs.setSize(0);

    // VARIABLE DECLARATIONS
    int i,p;
    int size = getSize(false);

    // NO LIST
    if(aList==NULL) {
        for(i=0;i<size;i++) {
            Control& control = get(i);
            for(p=0;p<control.getNumParameters();p++) {
                rMaxs.append(control.getParameterMax(p));
            }
        }

    // LIST
    } else {
        int c,sp;
        for(i=0;i<aList->getSize();i++) {

            // PARAMETER
            sp = (*aList)[i];

            // GET CONTROL
            try {
                c = _ptcMap.get(sp);
            } catch(const Exception& x) {
                log_error("Exception: {}", x.getMessage());
                continue;
            }
            Control& control = get(c);
        
            // GET PARAMETER MAX
            p = _ptpMap[sp];
            rMaxs.append(control.getParameterMax(p));
        }
    }
}

//-----------------------------------------------------------------------------
// PARAMETER VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the values of the control parameters.
 *
 * @param rP Array of parameters values, the length of which should
 * equal the total number of parameters or the size of aList.
 * @param aList List of controls.  If NULL, the values for all parameters for
 * all controls in the control set are gotten.  Otherwise, the parameters for
 * the controls in the list are gotten.
 */
void ControlSet::
getParameterValues(double rP[],const Array<int> *aList) const
{
    // VARIABLE DECLARATIONS
    int i,p;
    int size = getSize(false);

    // NO LIST
    if(aList==NULL) {
        int n;
        for(n=i=0;i<size;i++) {
            Control& control = get(i);
            for(p=0;p<control.getNumParameters();p++,n++) {
                rP[n] = control.getParameterValue(p);
            }
        }

    // LIST
    } else {
        int c,sp;
        for(i=0;i<aList->getSize();i++) {

            // PARAMETER
            sp = (*aList)[i];

            // GET CONTROL
            try {
                c = _ptcMap.get(sp);
            } catch(const Exception& x) {
                log_error("Exception: {}", x.getMessage());
                continue;
            }
            Control& control = get(c);
        
            // GET PARAMETER MAX
            p = _ptpMap[sp];
            rP[i] = control.getParameterValue(p);
        }
    }
}
//_____________________________________________________________________________
/**
 * Get the values of the control parameters.
 *
 * @param rP Array of parameters values, the length of which should
 * equal the total number of parameters or the size of aList.
 * @param aList List of controls.  If NULL, the values for all parameters for
 * all controls in the control set are gotten.  Otherwise, the parameters for
 * the controls in the list are gotten.
 */
void ControlSet::
getParameterValues(Array<double> &rP,const Array<int> *aList) const
{
    rP.setSize(0);

    // VARIABLE DECLARATIONS
    int i,p;
    int size = getSize(false);

    // NO LIST
    if(aList==NULL) {
        for(i=0;i<size;i++) {
            Control& control = get(i);
            for(p=0;p<control.getNumParameters();p++) {
                rP.append(control.getParameterValue(p));
            }
        }

    // LIST
    } else {
        int c,sp;
        for(i=0;i<aList->getSize();i++) {

            // PARAMETER
            sp = (*aList)[i];

            // GET CONTROL
            try {
                c = _ptcMap.get(sp);
            } catch(const Exception& x) {
                log_error("Exception: {}", x.getMessage());
                continue;
            }
            Control& control = get(c);
        
            // GET PARAMETER MAX
            p = _ptpMap[sp];
            rP.append(control.getParameterValue(p));
        }
    }
}
//_____________________________________________________________________________
/**
 * Set the values of the control parameters.
 *
 * @param aP Array of control parameters, the length of which should be
 * either the total number of parameters (if aList=NULL) or the same
 * length as aList.
 * @param aList List of controls.  If NULL, the values for all parameters for
 * all controls in the control set are gotten.  Otherwise, the parameters for
 * the controls in the list are gotten.
 */
void ControlSet::
setParameterValues(const double *aP,const Array<int> *aList)
{
    // VARIABLE DECLARATIONS
    int i,sp,p;
    int size = getSize(false);

    // NO LIST
    if(aList==NULL) {
        for(sp=i=0;i<size;i++) {
            Control& control = get(i);
            for(p=0;p<control.getNumParameters();p++,sp++) {
                control.setParameterValue(p,aP[sp]);
            }
        }

    // LIST
    } else {
        int c;
        int n = aList->getSize();
        for(i=0;i<n;i++) {

            // PARAMETER
            sp = (*aList)[i];

            // GET CONTROL
            try {
                c = _ptcMap.get(sp);
            } catch(const Exception& x) {
                log_error("Exception: {}", x.getMessage());
                continue;
            }
            Control& control = get(c);
        
            // SET PARAMETER
            p = _ptpMap[sp];
            control.setParameterValue(p,aP[i]);
        }
    }
}

//_____________________________________________________________________________
/**
 * Set the values of the control parameters.
 *
 * @param aP Array of control parameters, the length of which should be
 * either the total number of parameters (if aList=NULL) or the same
 * length as aList.
 * @param aList List of controls.  If NULL, the values for all parameters for
 * all controls in the control set are gotten.  Otherwise, the parameters for
 * the controls in the list are gotten.
 */
void ControlSet::
setParameterValues(const Array<double> &aP,const Array<int> *aList)
{
    // VARIABLE DECLARATIONS
    int i,sp,p;
    int size = getSize(false);

    // NO LIST
    if(aList==NULL) {
        for(sp=i=0;i<size;i++) {
            Control& control = get(i);
            for(p=0;p<control.getNumParameters();p++,sp++) {
                if(sp>=aP.getSize()) {
                    log_error("ControlSet.setParameterValues: incorrect number "
                              "of control parameters ({}).", aP.getSize());
                    return;
                }
                control.setParameterValue(p,aP[sp]);
            }
        }

    // LIST
    } else {
        int c;
        int n = aList->getSize();
        if(n > aP.getSize()) {
            log_warn("ControlSet.setParameterValues: the size of the array of "
                     "control parameters ({}) is different than the size of "
                     "the list of controls ({}).", n, aP.getSize());
            n = aP.getSize();
        }

        for(i=0;i<n;i++) {

            // PARAMETER
            sp = (*aList)[i];

            // GET CONTROL
            try {
                c = _ptcMap.get(sp);
            } catch(const Exception& x) {
                log_error("Exception: {}", x.getMessage());
                continue;
            }
            Control& control = get(c);
        
            // SET PARAMETER
            p = _ptpMap[sp];
            control.setParameterValue(p,aP[i]);
        }
    }
}


//=============================================================================
// UTILITY
//=============================================================================
//-----------------------------------------------------------------------------
// SIMPLIFY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Simplify all the control curves in a control set.
 *
 * @param aProperties Set of properties containing the parameters used
 * to carry out the simplifications.  See each control class for the list
 * of properties needed to perform the simplification.
 */
void ControlSet::
simplify(const PropertySet &aProperties)
{
    int i;
    int size = getSize();
    for(i=0;i<size;i++) {

        // GET CONTROL
        Control& control = get(i);

        // SIMPLIFY
        try {
            control.simplify(aProperties);
        } catch(const Exception& x) {
            log_error("Exception: {}", x.getMessage());
        }
    }
}
//-----------------------------------------------------------------------------
// FILTER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Filter all the control curves in a control set.
 * Currently implemented only for ControlLinear controls.
 *
 * @param aT Time at which each control's value is filtered.
 */
void ControlSet::
filter(double aT)
{
    int i;
    int size = getSize();
    for(i=0;i<size;i++) {

        // GET CONTROL
        Control& control = get(i);

        // FILTER
        try {
            if (control.getFilterOn())
            {
                control.filter(aT);
            }
        } catch(const Exception& x) {
            log_error("Exception: {}", x.getMessage());
        }
    }
}
//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct a storage object from the controls in the set over a specified
 * interval using a specified number of points.
 *
 * @param aN Number of times to sample the time interval.
 * @param aT1 Start of the interval.
 * @param aT2 End of the interval.
 * @param aForModelControls
 */
Storage* ControlSet::
constructStorage(int aN,double aT1,double aT2,bool aForModelControls)
{
    if(aN<=2) aN=2;
    double dt = (aT2-aT1) / (aN-1);

    double t;
    Array<double> x(0.0);
    Storage *store = new Storage();
    for(t=aT1;t<aT2;t+=dt) {
        getControlValues(t,x,aForModelControls);
        store->append(t,x.getSize(),&x[0]);
    }

    store->setName(getName());

    return(store);
}
//-----------------------------------------------------------------------------
// PARAMETER MAPS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Map a control-set parameter to a control.
 *
 * In the control set, the parameters for each of the controls are
 * concatenated into an array.  Since there is not a one-to-one
 * correspondence between control-set parameters and controls (i.e., a
 * particular control may have many parameters), it is necessary
 * both to map to which control each control-set parameter belongs and
 * to map to which parameter within a particular control a control-set
 * parameter corresponds.
 *
 * @param aIndex Index of the parameter in this control set.
 * @return Index of the control to which the specified parameter belongs.  If
 * no such parameter exists, -1 is returned.
 * @see mapParameterToParameter()
 */
int ControlSet::
mapParameterToControl(int aIndex) const
{
    if(aIndex<0) return(-1);
    if(aIndex>_ptcMap.getSize()) return(-1);
    return(_ptcMap[aIndex]);
}
//_____________________________________________________________________________
/**
 * Map a control-set parameter to a parameter of a particular control.
 *
 * In the control set, the parameters for each of the controls are
 * concatenated into an array.  Since there is not a one-to-one
 * correspondence between control-set parameters and controls (i.e., a
 * particular control may have many parameters), it is necessary
 * both to map to which control each control-set parameter belongs and
 * to map to which parameter within a particular control a control-set
 * parameter corresponds.
 *
 * @param aIndex Index of the parameter in the control set.
 * @return Index of the parameter with in a particular to which the specified
 * parameter corresponds.  If no such parameter exists, -1 is returned.
 * @see mapParameterToControl()
 */
int ControlSet::
mapParameterToParameter(int aIndex) const
{
    if(aIndex<0) return(-1);
    if(aIndex>_ptcMap.getSize()) return(-1);
    return(_ptcMap[aIndex]);
}
//_____________________________________________________________________________
/**
 * Generate the maps relating a parameter in the control set to a control
 * and a parameter of a control.
 *
 * This method should be called any time there is a change in the number or
 * types of controls held in the control set.
 */
void ControlSet::
generateParameterMaps()
{
    _ptcMap.setSize(0);
    _ptpMap.setSize(0);

    int i,j;
    int size = getSize(false);
    for(i=0;i<size;i++) {

        Control& control = get(i);

        for(j=0;j<control.getNumParameters();j++) {
            _ptcMap.append(i);
            _ptpMap.append(j);
        }
    }
}
//_____________________________________________________________________________
/**
 * convert a column of a storage column into a ControlLinear object and return it
 * index is zero based after time (0 for 
 */
ControlLinear*
ControlSet::ExtractControl(const Storage& storage,int index)
{
    int i;

    // NAME ATTRIBUTE
    const Array<std::string> &columnLabels = storage.getColumnLabels();
    std::string colName = columnLabels.get(index+1);

    // TIME
    double *times = NULL;
    int nTimes = storage.getTimeColumn(times);
    std::unique_ptr<double[]> times_ptr{times};

    // VALUE
    int nValues = nTimes;
    double *values = NULL;
    storage.getDataColumn(index,values);
    std::unique_ptr<double[]> values_ptr{values};

    // CONSTRUCT LINEAR CONTROL NODE
    ControlLinear *control = new ControlLinear;
    control->setName(colName);
    control->clearControlNodes();

    // APPEND CONTROL ELEMENTS
    int n = nTimes;
    if(n>nValues) n = nValues;
    for(i=0;i<n;i++) {
        control->setControlValue(times[i],values[i]);
    }
    return(control);
}
