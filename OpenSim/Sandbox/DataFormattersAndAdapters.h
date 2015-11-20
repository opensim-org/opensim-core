
// Two main class families here:
//      DataStreamFormatter
//          TRCDataStreamFormatter
//          MotDataStreamFormatter
//          ...
//
// and  DataAdapter
//          StreamDataAdapter
//              StringDataAdapter
//          FileDataAdapter
//          (plus two helper classes)
//
// There is also C3dFormatter which is just a wrapper for BTK; it is not
// a DataStreamFormatter or DataAdapter. Alternative: create an abstract
// DataFormatter base class and derive DataStreamFormatter and C3dFormatter
// from it; I don't think that's necessary.


//==============================================================================
//                    DATA STREAM FORMATTER classes
//==============================================================================
/** Read/write DataTables from/to a stream whose format is known by the 
concrete DataStreamFormatter object. Also maintains a static registry of these
objects indexed by a string identifying the expected format, and serves as
a factory for concrete DataStreamFormatter objects. 

Note that concrete objects of this type are associated with a *format*; they
are not associated with any particular *stream*. The stream is supplied at the
time it is to be read from or written to, and the same Formatter object can be
used for different streams as long as they have the right format.
**/
class DataStreamFormatter {
public:
    // Use non-virtual interface design pattern.

    /** Given an input stream expected to be in the format associated with this 
    Formatter, read DataTables out of it. The particular set of DataTables 
    returned will vary based on the format and the stream contents. **/
    OutputTables read(std::istream& in) const;

    /** Given an output stream, append the given tables to it in the format
    associated with this Formatter. **/
    void write(std::ostream& out, InputTables);

    // Registry

    /** Use the Formatter registry to create an appropriate stream formatter
    for the given format key. **/ 
    static unique_ptr<DataStreamFormatter> 
        createFormatter(const std::string& formatKey);

    /** Return true if there is a Formatter suitable for the given format. **/
    static bool hasFormatter(const std::string& formatKey);

    /** Transfer ownership of the formatter object to the registry and associate
    it with the given key, which is typically a file extension. **/
    static void registerDataStreamFormatter
       (const std:string&                 formatKey,
        unique_ptr<DataStreamFormatter>&& formatter);

    // Bookkeeping stuff

    virtual ~DataStreamFormatter() = default;

    /** Produce an identical copy of this concrete DataStreamFormatter. Internal
    state, if any, is reset in the copy to just-constructed condition. **/
    unique_ptr<DataStreamFormatter> clone() const {return implementClone();}
protected:
    DataStreamFormatter() = default;

    // Virtual interface is protected.
    virtual unique_ptr<DataStreamFormatter> implementClone() const = 0;
    virtual OutputTables implementRead(std::istream&) const = 0;
    virtual void implementWrite(std::ostream&, InputTables) = 0;
private:
    // The static registry map.
};

/** This is a DataStreamFormatter that understands the trc format. **/
class TRCDataStreamFormatter : public DataStreamFormatter {
public:
    TRCDataStreamFormatter();

private:
    unique_ptr<DataStreamFormatter> implementClone() const
    {   return TRCDataStreamFormatter(*this); }

    OutputTables implementRead(std::istream& trcIn) const override;
    void implementWrite(std::ostream& trcOut, InputTables) override;
};

/** This is a DataStreamFormatter that understands the mot format. **/
class MotDataStreamFormatter : public DataStreamFormatter {
public:
    MotDataStreamFormatter();

private:
    unique_ptr<DataStreamFormatter> implementClone() const
    {   return MotDataStreamFormatter(*this); }

    OutputTables implementRead(std::istream& motIn) const override;
    void implementWrite(std::ostream& motOut, InputTables) override;
};

//==============================================================================
//                         C3D FORMATTER
//==============================================================================
/** A standalone wrapper class for the BTK library; this is not a 
DataStreamFormatter. **/
class C3dFormatter {
public:
    // Useful stuff for dealing with BTK.
};

//==============================================================================
//                       DATA ADAPTER classes
//==============================================================================
/** Associated with a data source/sink of a known type and can read and write
DataTables from/to that source/sink. 

Note that these objects are associated with both a format and a data source,
which might be a file or cin or a string or whatever.
**/
class DataAdapter {
public:
    OutputTables read() const;
    void write(InputTables);
    virtual ~DataAdapter() = default;

protected:
    DataAdapter() = default;
    virtual OutputTables implementRead() const = 0;
    virtual void implementWrite(InputTables) = 0;
};

/** This is a DataAdapter for use with an already-existing read/write stream.
For read-only streams (istream) like std::cin or write-only (ostream) like
std::cout we'll need IStreamDataAdapter and OStreamDataAdapter also. Those
should raise exceptions if disallowed read() or write() is called.
**/
class StreamDataAdapter : public DataAdapter {
public:
    /** Associate this adapter with a stream and a format for that stream. It
    is an error if the formatKey cannot be found in the DataStreamFormatter
    registry. **/
    StreamDataAdapter(iostream& stream, const std::string& formatKey) 
    :   m_stream(stream) {
        m_formatter = DataStreamFormatter::createFormatter(formatKey);
    }
private:
    OutputTables implementRead() const override 
    {   return m_formatter->read(m_stream); }
    void implementWrite(InputTables input) override 
    {   return m_formatter->write(m_stream, input); }

    std::iostream&                  m_stream;
    unique_ptr<DataStreamFormatter> m_formatter;
};

/** A string adapter is a kind of stream adapter. **/
class StringDataAdapter : public StreamDataAdapter {
public:
    StringDataAdapter(const std::string& data, const std::string& formatKey)
    :   StreamDataAdapter(std::stringstream(data), formatKey) {}
};

/** Helper class for when you know this file will be handled using one of
the DataStreamFormatters. Users will use the FileDataAdapter instead. **/
class FileStreamDataAdapter : public StreamDataAdapter {
public:
    FileStreamDataAdapter(const std::string& pathName)
    :   StreamDataAdapter(std::fstrea(pathName), 
                          findExtension(pathName)) {}
};

/** Helper class for when you know this file will be handled using the
C3dFormatter. Users will use the FileDataAdapter instead. **/
class C3dFileDataAdapter : public DataAdapter {
public:
    C3dFileDataAdapter(const std::string& c3dFileName) 
    :   m_formatter(c3dFilesName) {}
private:
    OutputTables implementRead() const override 
    {   return m_formatter->read(m_stream); }
    void implementWrite(InputTables input) override 
    {   return m_formatter->write(m_stream, input); }

    C3dFormatter m_formatter;
};


/** Instantiate a DataAdapter for a particular file given by name. This class 
figures out the appropriate format to use for the file. **/
class FileDataAdapter : public DataAdapter {
public:
    FileDataAdapter(const std::string& pathName) {
        const std::string ext = findExtension(pathName);
        if (ext == "c3d") {
            m_adapter.reset(new C3dFileDataAdapter(pathName));
        } else
            m_adapter.reset(new FileStreamDataAdapter(pathName));
    }
private:
    // These just forward to the actual DataAdapter in use.
    OutputTables implementRead() const override 
    {   return m_adapter->read(); }
    void implementWrite(InputTables input) override 
    {   return m_adapter->write(input); }

    unique_ptr<DataAdapter> m_adapter;
};