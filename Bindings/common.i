
%feature("director") OpenSim::AnalysisWrapper;
%feature("director") OpenSim::SimtkLogCallback;
%feature("notabstract") ControlLinear;

%rename(OpenSimObject) OpenSim::Object;
%rename(OpenSimException) OpenSim::Exception;

/* rest of header files to be wrapped */
%include <OpenSim/version.h>

// osimCommon Library
%include <OpenSim/Common/osimCommonDLL.h>
%include <OpenSim/Common/Exception.h>
%template(IndexOutOfRangeSizeT) OpenSim::IndexOutOfRange<size_t>;
%include <OpenSim/Common/Array.h>
%include <OpenSim/Common/ArrayPtrs.h>
%include <OpenSim/Common/AbstractProperty.h>
%include <OpenSim/Common/Property.h>
%include <OpenSim/Common/PropertyGroup.h>
%template(ArrayPtrsPropertyGroup) OpenSim::ArrayPtrs<OpenSim::PropertyGroup>;
%template (PropertyString) OpenSim::Property<std::string>;
%include <OpenSim/Common/Object.h>
%include <OpenSim/Common/ObjectGroup.h>

%include <OpenSim/Common/Set.h>
%include <OpenSim/Common/StateVector.h>
%template(ArrayStateVector) OpenSim::Array<OpenSim::StateVector>;
%include <OpenSim/Common/StorageInterface.h>
%include <OpenSim/Common/Storage.h>
%template(ArrayStorage) OpenSim::ArrayPtrs<OpenSim::Storage>;
%include <OpenSim/Common/Units.h>
%include <OpenSim/Common/IO.h>
%include <OpenSim/Common/Function.h>

%template(SetFunctions) OpenSim::Set<OpenSim::Function>;
%include <OpenSim/Common/FunctionSet.h>

%include <OpenSim/Common/Constant.h>
%include <OpenSim/Common/SimmSpline.h>
%include <OpenSim/Common/StepFunction.h>
%include <OpenSim/Common/PiecewiseConstantFunction.h>
%include <OpenSim/Common/LinearFunction.h>
%include <OpenSim/Common/PiecewiseLinearFunction.h>
%include <OpenSim/Common/MultiplierFunction.h>
%include <OpenSim/Common/GCVSpline.h>
%include <OpenSim/Common/Sine.h>
%include <OpenSim/Common/PolynomialFunction.h>

%include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
%include <OpenSim/Common/SmoothSegmentedFunction.h>

%include <OpenSim/Common/XYFunctionInterface.h>
%template(ArrayXYPoint) OpenSim::Array<XYPoint>;
%template(ArrayBool) OpenSim::Array<bool>;
%template(ArrayDouble) OpenSim::Array<double>;
%template(ArrayInt) OpenSim::Array<int>;
%template(ArrayStr) OpenSim::Array<std::string>;
%template(ArrayObjPtr) OpenSim::Array<OpenSim::Object*>;
%template(ArrayPtrsObj) OpenSim::ArrayPtrs<OpenSim::Object>;

%include <OpenSim/Common/ComponentOutput.h>
%template(OutputDouble) OpenSim::Output<double>;
%template(OutputVec3) OpenSim::Output<SimTK::Vec3>;
%template(OutputTransform) OpenSim::Output<SimTK::Transform>;
%template(OutputVector) OpenSim::Output<SimTK::Vector>;

%include <OpenSim/Common/ComponentConnector.h>

%include <OpenSim/Common/ComponentList.h>
// TODO rename to singular form.
%template(ComponentsList) OpenSim::ComponentList<OpenSim::Component>;
%template(ComponentIterator) OpenSim::ComponentListIterator<OpenSim::Component>;

%include <OpenSim/Common/Component.h>
%template(getComponentsList) OpenSim::Component::getComponentList<OpenSim::Component>;

%include <OpenSim/Common/Scale.h>
%template(SetScales) OpenSim::Set<OpenSim::Scale>;
%include <OpenSim/Common/ScaleSet.h>
%include <OpenSim/Common/MarkerFrame.h>
%include <OpenSim/Common/MarkerData.h>

 // %include <Bindings/std.i>
%shared_ptr(OpenSim::AbstractDataTable);
%shared_ptr(OpenSim::DataTable_<double, double>);
%shared_ptr(OpenSim::DataTable_<double, SimTK::Vec3>);
%shared_ptr(OpenSim::TimeSeriesTable_<double>);
%shared_ptr(OpenSim::TimeSeriesTable_<SimTK::Vec3>);
%ignore OpenSim::AbstractDataTable::clone;
%ignore OpenSim::AbstractDataTable::getTableMetaData;
%ignore OpenSim::AbstractDataTable::updTableMetaData;
%ignore OpenSim::AbstractDataTable::getIndependentMetaData;
%ignore OpenSim::AbstractDataTable::setIndependentMetaData;
%ignore OpenSim::AbstractDataTable::getDependentsMetaData;
%ignore OpenSim::AbstractDataTable::setDependentsMetaData;
%include <OpenSim/Common/DataTable.h>
%include <OpenSim/Common/TimeSeriesTable.h>
%template(DataTable)           OpenSim::DataTable_<double, double>;
%template(DataTableVec3)       OpenSim::DataTable_<double, SimTK::Vec3>;
%template(TimeSeriesTable)     OpenSim::TimeSeriesTable_<double>;
%template(TimeSeriesTableVec3) OpenSim::TimeSeriesTable_<SimTK::Vec3>;

%include <OpenSim/Common/Event.h>
%template(StdVectorEvent) std::vector<OpenSim::Event>;
%template(StdMapStringTimeSeriesTableVec3)
        std::map<std::string, 
                 std::shared_ptr<OpenSim::TimeSeriesTable_<SimTK::Vec3>>>;
%shared_ptr(OpenSim::DataAdapter)
%shared_ptr(OpenSim::FileAdapter)
%shared_ptr(OpenSim::DelimFileAdapter)
%shared_ptr(OpenSim::MOTFileAdapter)
%shared_ptr(OpenSim::CSVFileAdapter)
%shared_ptr(OpenSim::TRCFileAdapter)
%shared_ptr(OpenSim::C3DFileAdapter)
%template(StdMapStringDataAdapter)
        std::map<std::string, std::shared_ptr<OpenSim::DataAdapter>>;
%template(StdMapStringAbstractDataTable)
        std::map<std::string, std::shared_ptr<OpenSim::AbstractDataTable>>;
%include <OpenSim/Common/DataAdapter.h>
%include <OpenSim/Common/FileAdapter.h>
%include <OpenSim/Common/TRCFileAdapter.h>
%include <OpenSim/Common/DelimFileAdapter.h>
%include <OpenSim/Common/MOTFileAdapter.h>
%include <OpenSim/Common/CSVFileAdapter.h>
%include <OpenSim/Common/C3DFileAdapter.h>
