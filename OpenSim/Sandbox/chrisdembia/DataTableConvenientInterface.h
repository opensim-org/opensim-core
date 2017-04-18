// AbstractDataTable
class AbstractDataTable {
public:
    using TableMetaData       = ValueArrayDictionary;
    using DependentsMetaData  = ValueArrayDictionary;
    using IndependentMetaData = ValueArrayDictionary;

    AbstractDataTable()                                      = default;
    AbstractDataTable(const AbstractDataTable&)              = default;
    AbstractDataTable(AbstractDataTable&&)                   = default;
    AbstractDataTable& operator=(const AbstractDataTable&)   = default;
    AbstractDataTable& operator=(AbstractDataTable&&)        = default;
    virtual std::unique_ptr<AbstractDataTable> clone() const = 0;
    virtual ~AbstractDataTable()                             = default;
    const TableMetaData& getTableMetaData() const;
    TableMetaData& updTableMetaData();
    size_t getNumRows() const;
    size_t getNumColumns() const;
    std::vector<std::string> getColumnLabels() const; // <---
    void setColumnLabels(const std::vector<std::string>&); // <---
    template <typename Range> void setColumnLabels(const Range&); // <---
    int getColumnIndex(std::string columnLabel) const; // <---
    const IndependentMetaData& getIndependentMetaData() const;
    void setIndependentMetaData(
            const IndependentMetaData& independentMetaData);
    const SimTK::AbstractValue& getDependentsMetadata(
                                   std::string columnLabel,
                                   std::string key) const; // <---
    template <typename T>
    const T& getDependentsMetadata(std::string columnLabel,
                                   std::string key) const; // <---
    const DependentsMetaData& getDependentsMetaData() const;
    void setDependentsMetaData(const DependentsMetaData& dependentsMetaData);
};

// DataTable_
template<typename ETX = double, typename ETY = SimTK::Real>
class DataTable_ : public AbstractDataTable {
public:
    using RowVector     = SimTK::RowVector_<ETY>;
    using RowVectorView = SimTK::RowVectorView_<ETY>;
    using VectorView    = SimTK::VectorView_<ETY>;

    DataTable_()                             = default;
    DataTable_(const DataTable_&)            = default;
    DataTable_(DataTable_&&)                 = default;
    DataTable_& operator=(const DataTable_&) = default;
    DataTable_& operator=(DataTable_&&)      = default;
    ~DataTable_()                            = default;

    std::unique_ptr<AbstractDataTable> clone() const override;
    void appendRow(const ETX& indRow, const RowVector& depRow);
    RowVectorView getRowAtIndex(size_t index) const;
    RowVectorView getRow(const ETX& ind) const;
    RowVectorView updRowAtIndex(size_t index);
    RowVectorView updRow(const ETX& ind);
    const std::vector<ETX>& getIndependentColumn() const;
    VectorView getDependentColumnAtIndex(size_t index) const;
    VectorView getDependentColumn(std::string columnLabel) const; // <---
    void setIndependentColumnAtIndex(size_t index, const ETX& value);
};

// TimeSeriesTable_
template<typename ETY = SimTK::Real>
class TimeSeriesTable_ : public DataTable_<double, ETY> {
public:
    using RowVector = SimTK::RowVector_<ETY>;

    TimeSeriesTable_()                                   = default;
    TimeSeriesTable_(const TimeSeriesTable_&)            = default;
    TimeSeriesTable_(TimeSeriesTable_&&)                 = default;
    TimeSeriesTable_& operator=(const TimeSeriesTable_&) = default;
    TimeSeriesTable_& operator=(TimeSeriesTable_&&)      = default;
    ~TimeSeriesTable_()                                  = default;

    TimeSeriesTable_(const DataTable_<double, ETY>& datatable);
}; // TimeSeriesTable_


