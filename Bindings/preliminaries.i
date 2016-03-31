
/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"

%include "std_vector.i"
%template(StdVecInt)    std::vector<int>;
%template(StdVecDouble) std::vector<double>;
%template(StdVecString) std::vector<std::string>;

%include "std_map.i"
%include <std_shared_ptr.i>
