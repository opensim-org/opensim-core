namespace OpenSim {
    /** Proxy class to replace SimTK::RowVectorView & SimTK::VectorView for
    Java wrapping. As of swig 3.0.8, java wrapping causes seg-fault with these
    classes.                                                                  */
    template<typename T, bool>
    class FixedSizeArray_ {
    public:
        FixedSizeArray_()                                  = default;
        FixedSizeArray_(const FixedSizeArray_&)            = default;
        FixedSizeArray_(FixedSizeArray_&&)                 = default;
        FixedSizeArray_& operator=(const FixedSizeArray_&) = default;
        FixedSizeArray_& operator=(FixedSizeArray_&&)      = default;

        FixedSizeArray_(SimTK::RowVectorView_<T>&& rowView) {
            for(int i = 0; i < rowView.ncol(); ++i)
                _data.push_back(&rowView[i]);
        }

        FixedSizeArray_(SimTK::VectorView_<T>&& vecView) {
            for(int i = 0; i < vecView.nrow(); ++i)
                _data.push_back(&vecView[i]);
        }

        T get(size_t i) {
            return *(_data.at(i));
        }

        void set(size_t i, const T& value) {
            *(_data.at(i)) = value;
        }

        size_t size() {
            return _data.size();
        }
        
    private:
        std::vector<T*> _data;
    };
} // namespace OpenSim
