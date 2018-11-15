/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: InvokeOnCopy.h                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifndef MUSCOLLO_INVOKEONCOPY_H
#define MUSCOLLO_INVOKEONCOPY_H

/// Invoke a function F on an object of type T whenever making a copy of it.
/// This class template is based on SimTK::ReinitOnCopy.
/// This also invokes F in the move constructor and move assignment operator.
template <class T, void (*F)(T&)>
class InvokeOnCopy : public T {

    /** @cond **/ // These confuse doxygen.
    static_assert(!std::is_array<T>::value,
            "InvokeOnCopy<T> does not allow T to be an array.");

    static_assert(   std::is_copy_constructible<T>::value
                  && std::is_copy_assignable<T>::value,
            "InvokeOnCopy<T> requires type T to have an accessible copy "
            "constructor and copy assignment operator.");

    static_assert(std::is_destructible<T>::value,
            "InvokeOnCopy<T> requires type T to have an accessible destructor.");
    /** @endcond **/

public:
    using T::T;
    using T::operator=;

    InvokeOnCopy() = default;

    /// Copy constructor sets the value and invokes F.
    InvokeOnCopy(const InvokeOnCopy& source) : T(source.getT()) { F(updT()); }

    explicit InvokeOnCopy(const T& value) : T(value) {}

    InvokeOnCopy(InvokeOnCopy&& source)
            : T(std::move(static_cast<T&&>(source))) { F(updT()); }

    /// Copy assignment invokes F after the copy.
    InvokeOnCopy& operator=(const InvokeOnCopy& other)
    {   T::operator=(other.getT()); F(updT()); return *this; }

    InvokeOnCopy& operator=(InvokeOnCopy&& source)
    {   T::operator=(static_cast<T&&>(source)); F(updT()); return *this; }

    /// Assignment from an object of type `T` uses `T`'s copy assignment
    /// operator and invokes F.
    InvokeOnCopy& operator=(const T& value)
    {   T::operator=(value); F(updT()); return *this; }

    /// Return a const reference to the contained object of type `T`.
    const T& getT() const {return static_cast<const T&>(*this);}
    /// Return a writable reference to the contained object of type `T`.
    T&       updT()       {return static_cast<T&>(*this);}

};

#endif // MUSCOLLO_INVOKEONCOPY_H
