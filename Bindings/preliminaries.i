
/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"

%include "std_vector.i"
%template(StdVectorUnsigned) std::vector<unsigned>;
%template(StdVectorInt)      std::vector<int>;
%template(StdVectorDouble)   std::vector<double>;
%template(StdVectorString)   std::vector<std::string>;

%include "std_map.i"
%include <std_shared_ptr.i>

// unique_ptr
// ----------
// https://stackoverflow.com/questions/27693812/how-to-handle-unique-ptrs-with-swig
namespace std {
        %feature("novaluewrapper") unique_ptr;
        template <typename Type>
        struct unique_ptr {
            typedef Type* pointer;
            explicit unique_ptr( pointer Ptr );
            unique_ptr (unique_ptr&& Right);
            template<class Type2, Class Del2> unique_ptr( unique_ptr<Type2, Del2>&& Right );
            unique_ptr( const unique_ptr& Right) = delete;
            pointer operator-> () const;
            pointer release ();
            void reset (pointer __p=pointer());
            void swap (unique_ptr &__u);
            pointer get () const;
            operator bool () const;
            ~unique_ptr();
        };
}
