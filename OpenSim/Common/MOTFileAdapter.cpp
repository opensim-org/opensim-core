


namespace OpenSim {

MOTFileAdapter::MOTFileAdapter() :
    DelimFileAdapter(" \t", // delimites for read
                     "\t"   // delimiter for write
                     ) {}

MOTFileAdapter*
MOTFileAdapter::clone() const {
    return new MOTFileAdapter{*this};
}

}
