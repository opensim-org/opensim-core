#ifndef OPENSIM_OPENSIM_HEADERS_OPENSIM_H_
#define OPENSIM_OPENSIM_HEADERS_OPENSIM_H_
/* This header file is only used with SWIG to create bindings.
*/

#include <OpenSim/version.h>
#include <SimTKsimbody.h>
#include <OpenSim/Common/osimCommonDLL.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/AbstractProperty.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/PropertyGroup.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ObjectGroup.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/StateVector.h>
#include <OpenSim/Common/StorageInterface.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Scale.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/StepFunction.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/Sine.h>
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>
#include <OpenSim/Common/XYFunctionInterface.h>
#include <OpenSim/Common/FunctionSet.h>

#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Common/Component.h>

#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Common/DataTable.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/DataAdapter.h>
#include <OpenSim/Common/FileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Common/DelimFileAdapter.h>
#include <OpenSim/Common/MOTFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <OpenSim/Common/C3DFileAdapter.h>

#endif // OPENSIM_OPENSIM_HEADERS_OPENSIM_H_

