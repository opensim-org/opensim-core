%include "std_string.i"

%include "std_vector.i"
%template(StdVectorDouble) std::vector<double>;
%template(StdVectorString) std::vector<std::string>;

%include "std_map.i"
%include <std_shared_ptr.i>
