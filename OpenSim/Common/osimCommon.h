#ifndef _osimCommon_h_
#define _osimCommon_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  osimCommon.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

#include "Simbody.h"

#include "osimCommonDLL.h"
#include "Object.h"
#include "RegisterTypes_osimCommon.h"
#include "FunctionSet.h"
#include "GCVSplineSet.h"
#include "ScaleSet.h"
#include "GCVSpline.h"
#include "IO.h"

#include "Scale.h"
#include "SimmSpline.h"
#include "Constant.h"
#include "Sine.h"
#include "StepFunction.h"
#include "LinearFunction.h"
#include "PiecewiseConstantFunction.h"
#include "PiecewiseLinearFunction.h"

#include "MultiplierFunction.h"
#include "PolynomialFunction.h"

#include "ObjectGroup.h"
#include "StorageInterface.h"
#include "LoadOpenSimLibrary.h"
#include "SmoothSegmentedFunctionFactory.h"

#include "DataTable.h"
#include "TimeSeriesTable.h"

#include "Reporter.h"

#include "AbstractProperty.h"
#include "Adapters.h"
#include "Array.h"
#include "ArrayPtrs.h"
#include "C3DFileAdapter.h"
#include "ComponentConnector.h"
#include "Component.h"
#include "ComponentList.h"
#include "ComponentOutput.h"
#include "Constant.h"
#include "CSVFileAdapter.h"
#include "DataAdapter.h"
#include "DataTable.h"
#include "DebugUtilities.h"
#include "DelimFileAdapter.h"
#include "Event.h"
#include "Exception.h"
#include "FileAdapter.h"
#include "FunctionAdapter.h"
#include "Function.h"
#include "FunctionSet.h"
#include "gcvspl.h"
#include "GCVSpline.h"
#include "GCVSplineSet.h"
#include "IO.h"
#include "LinearFunction.h"
#include "Lmdif.h"
#include "LoadOpenSimLibrary.h"
#include "LogCallback.h"
#include "LogManager.h"
#include "MarkerData.h"
#include "MarkerFrame.h"
#include "MOTFileAdapter.h"
#include "Mtx.h"
#include "MultiplierFunction.h"
#include "ObjectGroup.h"
#include "Object.h"
#include "OptimizationTarget.h"
#include "osimCommonTemplates.h"
#include "PiecewiseConstantFunction.h"
#include "PiecewiseLinearFunction.h"
#include "PolynomialFunction.h"
#include "PropertyBoolArray.h"
#include "PropertyBool.h"
#include "PropertyDblArray.h"
#include "PropertyDbl.h"
#include "PropertyDblVec.h"
#include "Property_Deprecated.h"
#include "PropertyGroup.h"
#include "Property.h"
#include "PropertyIntArray.h"
#include "PropertyInt.h"
#include "PropertyObjArray.h"
#include "PropertyObj.h"
#include "PropertyObjPtr.h"
#include "PropertySet.h"
#include "PropertyStrArray.h"
#include "PropertyStr.h"
#include "PropertyTable.h"
#include "PropertyTransform.h"
#include "Reporter.h"
#include "RootSolver.h"
#include "Scale.h"
#include "ScaleSet.h"
#include "SegmentedQuinticBezierToolkit.h"
#include "Set.h"
#include "Signal.h"
#include "SimmIO.h"
#include "SimmMacros.h"
#include "SimmSpline.h"
#include "Sine.h"
#include "SmoothSegmentedFunctionFactory.h"
#include "SmoothSegmentedFunction.h"
#include "StateVector.h"
#include "StepFunction.h"
#include "Storage.h"
#include "StorageInterface.h"
#include "TimeSeriesTable.h"
#include "TRCFileAdapter.h"
#include "Units.h"
#include "ValueArrayDictionary.h"
#include "ValueArray.h"
#include "VectorFunction.h"
#include "VectorFunctionUncoupledNxN.h"
#include "XMLDocument.h"
#include "XYFunctionInterface.h"


#include <fstream>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <cstring>
#include <cctype>
#include <limits>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cassert>
#include <stdexcept>
#include <cstdlib>
#include <regex>
#include <exception>


#endif // _osimCommon_h_
