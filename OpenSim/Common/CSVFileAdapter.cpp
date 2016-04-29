


namespace OpenSim {

CSVFileAdapter::CSVFileAdapter() :
    DelimFileAdapter(",", // delimiter for read
                     ","  // delimiter for write
                     ) {}

CSVFileAdapter*
CSVFileAdapter::clone() const {
    return new CSVFileAdapter{*this};
}

}
