
/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"

%include "std_vector.i"
%template(StdVectorUnsigned) std::vector<unsigned>;
%template(StdVectorInt)      std::vector<int>;
%template(StdVectorDouble)   std::vector<double>;
%template(StdVectorString)   std::vector<std::string>;
%template(StdVectorBool)     std::vector<bool>;

%include "std_pair.i"
%template(StdPairIntInt)       std::pair<int, int>;
%template(StdVectorPairIntInt) std::vector<std::pair<int, int>>;

%include "std_map.i"
%include <std_shared_ptr.i>
%include <std_unique_ptr.i>
