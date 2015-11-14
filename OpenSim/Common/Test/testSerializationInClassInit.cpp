/* -------------------------------------------------------------------------- *
 *                OpenSim:  testSerializationInClassInit.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/GCVSpline.h>

using namespace SimTK;
using SimTK::Vec3;

namespace OpenSim {

class Bar;

// TODO test deserialization, etc. or, at least the "value is default" stuff.

// TODO try an initializer list as the initial value of a property; making my
// own custom type. or use brace initialization to call the constructor of a
// class. e.g. OpenSim_DECLARE_PROPERTY(x, Constant, "", {1, 2, 3, 4}).

// This class shows all the property declarations/constructions that were
// possible (to the best of my knowledge) in OpenSim 3.3. Some of the
// declarations are commented out since we have made them
// backwards-incompatible. Thus, this class shows what users might have to
// change in their code when upgrading from OpenSim 3.3 to 4.0.
class BackwardsCompatibility33 : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(BackwardsCompatibility33, Object);
public:
OpenSim_DECLARE_PROPERTY(alpha, double, "built-in type.");
OpenSim_DECLARE_PROPERTY(zeta, Vec3, "non-Object.");
OpenSim_DECLARE_PROPERTY(beta, Constant, "Concrete class (included).");
//OpenSim_DECLARE_PROPERTY(gamma, Function, "Abstract class (included).");
//OpenSim_DECLARE_PROPERTY(neptune, Bar, "Forward-declared class.");

OpenSim_DECLARE_OPTIONAL_PROPERTY(alphao, double, "built-in type.");
OpenSim_DECLARE_OPTIONAL_PROPERTY(zetao, Vec3, "non-Object.");
OpenSim_DECLARE_OPTIONAL_PROPERTY(betao, Constant, "Concrete.");
OpenSim_DECLARE_OPTIONAL_PROPERTY(gammao, Function, "Abstract.");
OpenSim_DECLARE_OPTIONAL_PROPERTY(neptuneo, Bar, "Forward-declared.");

// With an initial value provided.
OpenSim_DECLARE_OPTIONAL_PROPERTY(alphaoi, double, "built-in type.");
OpenSim_DECLARE_OPTIONAL_PROPERTY(zetaoi, Vec3, "non-Object.");
OpenSim_DECLARE_OPTIONAL_PROPERTY(betaoi, Constant, "Concrete.");
OpenSim_DECLARE_OPTIONAL_PROPERTY(gammaoi, Function, "Abstract.");
OpenSim_DECLARE_OPTIONAL_PROPERTY(neptuneoi, Bar, "Forward-declared.");

OpenSim_DECLARE_UNNAMED_PROPERTY(Constant, "Concrete.");
OpenSim_DECLARE_UNNAMED_PROPERTY(Function, "Abstract.");
OpenSim_DECLARE_UNNAMED_PROPERTY(Bar, "Forward-declared.");

OpenSim_DECLARE_LIST_PROPERTY(alphal, double, "built-in type.");
OpenSim_DECLARE_LIST_PROPERTY(zetal, Vec3, "non-Object.");
OpenSim_DECLARE_LIST_PROPERTY(betal, Constant, "Concrete.");
OpenSim_DECLARE_LIST_PROPERTY(gammal, Function, "Abstract.");
OpenSim_DECLARE_LIST_PROPERTY(neptunel, Bar, "Forward-declared.");

// These will use the constructProperty_() variant that doesn't take arguments.
OpenSim_DECLARE_LIST_PROPERTY(alphal0, double, "built-in type.");
OpenSim_DECLARE_LIST_PROPERTY(zetal0, Vec3, "non-Object.");
OpenSim_DECLARE_LIST_PROPERTY(betal0, Constant, "Concrete.");
OpenSim_DECLARE_LIST_PROPERTY(gammal0, Function, "Abstract.");
OpenSim_DECLARE_LIST_PROPERTY(neptunel0, Bar, "Forward-declared.");

OpenSim_DECLARE_LIST_PROPERTY_SIZE(alphals, double, 3, "built-in type.");
OpenSim_DECLARE_LIST_PROPERTY_SIZE(zetals, Vec3, 3, "non-Object.");
OpenSim_DECLARE_LIST_PROPERTY_SIZE(betals, Constant, 3, "Concrete.");
// Don't know how to make a list property of abstract type that must have more
// than 0 elements (SIZE, ATLEAST, RANGE).
// TODO OpenSim_DECLARE_LIST_PROPERTY_SIZE(gammals, Function, 3, "Abstract.");
OpenSim_DECLARE_LIST_PROPERTY_SIZE(neptunels, Bar, 3, "Forward-declared.");

OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(alphall, double, 2, "built-in type.");
OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(zetall, Vec3, 2, "non-Object.");
OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(betall, Constant, 2, "Concrete.");
// TODO OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(gammall, Function, 2, "Abstract.");
OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(neptunell, Bar, 2, "Forward-declared.");

OpenSim_DECLARE_LIST_PROPERTY_ATMOST(alphalm, double, 5, "built-in type.");
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(zetalm, Vec3, 5, "non-Object.");
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(betalm, Constant, 5, "Concrete.");
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(gammalm, Function, 5, "Abstract.");
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(neptunelm, Bar, 5, "Forward-declared class.");

// These will use the constructProperty_() variant that doesn't take arguments.
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(alphalm0, double, 5, "built-in type.");
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(zetalm0, Vec3, 5, "non-Object.");
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(betalm0, Constant, 5, "Concrete.");
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(gammalm0, Function, 5, "Abstract.");
OpenSim_DECLARE_LIST_PROPERTY_ATMOST(neptunelm0, Bar, 5, "Forward-declared class.");

OpenSim_DECLARE_LIST_PROPERTY_RANGE(alphalr, double, 4, 6, "built-in type.");
OpenSim_DECLARE_LIST_PROPERTY_RANGE(zetalr, Vec3, 4, 6, "non-Object.");
OpenSim_DECLARE_LIST_PROPERTY_RANGE(betalr, Constant, 4, 6, "Concrete.");
// TODO OpenSim_DECLARE_LIST_PROPERTY_RANGE(gammalr, Function, 4, 6, "Abstract.");
OpenSim_DECLARE_LIST_PROPERTY_RANGE(neptunelr, Bar, 4, 6, "Forward-declared");

BackwardsCompatibility33();
};

class DefaultInitialized : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(DefaultInitialized, Object);
public:
    OpenSim_DECLARE_PROPERTY(alpha, double, "Alpha.");
};

class UserInitialized : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(UserInitialized, Object);
public:
    OpenSim_DECLARE_PROPERTY(alpha, double, "Alpha.", 2);
};

class Uninitialized : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Uninitialized, Object);
public:
    OpenSim_DECLARE_PROPERTY_UNINIT(alpha, double, "Alpha.");
    Uninitialized() {
        constructProperty_alpha(2);
    }
};

class CustomCopy : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(CustomCopy, Object);
public:
    OpenSim_DECLARE_PROPERTY_UNINIT(alpha, double, "Alpha.");
    CustomCopy(const CustomCopy& obj) {
        copyProperty_alpha(obj);
    }
};

// Definition of forward-declared class.
class Bar : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Bar, Object);
};


BackwardsCompatibility33::BackwardsCompatibility33() 
{
    constructProperty_alpha(8);
    constructProperty_zeta(Vec3());
    constructProperty_beta(Constant());
    // TODO the next two are omitted.

    constructProperty_alphao();
    constructProperty_zetao();
    constructProperty_betao();
    constructProperty_gammao();
    constructProperty_neptuneo();
    set_alphao(25);
    set_zetao(Vec3(3, 1, 4));
    set_betao(Constant(42));
    set_gammao(GCVSpline());
    set_neptuneo(Bar());

    constructProperty_alphaoi(13);
    constructProperty_zetaoi(Vec3(36, 13, 1));
    constructProperty_betaoi(Constant(13));
    constructProperty_gammaoi(GCVSpline());
    constructProperty_neptuneoi(Bar());

    constructProperty_Constant(Constant());
    constructProperty_Function(GCVSpline());
    constructProperty_Bar(Bar());
    
    constructProperty_alphal(std::vector<double>{6, 3, 2, 7});
    constructProperty_zetal(std::vector<Vec3>{Vec3(1.5, 3, 4.6), Vec3(6)});
    constructProperty_betal(std::vector<Constant>{Constant()});
    constructProperty_gammal(std::vector<Function>());
    append_gammal(Constant(1));
    constructProperty_neptunel(std::vector<Bar>{Bar(), Bar()});

    constructProperty_alphal0();
    constructProperty_zetal0();
    constructProperty_betal0();
    constructProperty_gammal0();
    constructProperty_neptunel0();
    append_alphal0(91);
    append_zetal0(Vec3(15));
    append_betal0(Constant(12));
    append_gammal0(GCVSpline());
    append_neptunel0(Bar());

    constructProperty_alphals(std::vector<double>{6, 5, 4});
    constructProperty_zetals(std::vector<Vec3>{Vec3(1), Vec3(9), Vec3(6)});
    constructProperty_betals(
            std::vector<Constant>{Constant(), Constant(), Constant()});
    // TODO constructProperty_gammals(std::vector<Function>{Constant()});
    constructProperty_neptunels(std::vector<Bar>{Bar(), Bar(), Bar()});

    constructProperty_alphall(std::vector<double>{8, 6});
    constructProperty_zetall(std::vector<Vec3>{Vec3(10), Vec3(1), Vec3(9)});
    constructProperty_betall(std::vector<Constant>{Constant(), Constant()});
    // TODO constructProperty_gammall(std::vector<Function>{Constant()});
    constructProperty_neptunell(std::vector<Bar>{Bar(), Bar(), Bar(), Bar()});

    constructProperty_alphalm(std::vector<double>{1, 8, 7});
    constructProperty_zetalm(std::vector<Vec3>());
    constructProperty_betalm(std::vector<Constant>{Constant()});
    constructProperty_gammalm(std::vector<Function>());
    append_gammalm(GCVSpline());
    constructProperty_neptunelm(
            std::vector<Bar>{Bar(), Bar(), Bar(), Bar(), Bar()});

    constructProperty_alphalm0();
    constructProperty_zetalm0();
    constructProperty_betalm0();
    constructProperty_gammalm0();
    constructProperty_neptunelm0();
    append_alphalm0(5);
    append_zetalm0(Vec3(8));
    append_betalm0(Constant(10));
    append_gammalm0(Constant(8));
    append_neptunelm0(Bar());

    constructProperty_alphalr(std::vector<double>{1, 2, 3, 4, 5});
    constructProperty_zetalr(
            std::vector<Vec3>{Vec3(), Vec3(), Vec3(), Vec3(), Vec3(), Vec3()});
    constructProperty_betalr(
            std::vector<Constant>{Constant(), Constant(), Constant(), Constant()});
    // TODO constructProperty_gammalr(std::vector<Function>{GCVSpline()});
    constructProperty_neptunelr(std::vector<Bar>{Bar(), Bar(), Bar(), Bar()});

}

} // namespace

#define COPY_AND_ACCESS(prop_name) \
    SimTK_ASSERT_ALWAYS(obj.get_##prop_name() == copy->get_##prop_name(), \
            "Property '" #prop_name "' was not copied correctly.");

#define COPY_AND_ACCESS_LIST(prop_name) \
    for (int i = 0; i < obj.getProperty_##prop_name().size(); ++i) { \
        SimTK_ASSERT_ALWAYS(obj.get_##prop_name(i) == copy->get_##prop_name(i), \
                "Property '" #prop_name "' was not copied correctly."); \
    }

template <typename T>
void testCopyAndAccess() {
    T obj;
    auto* copy = obj.clone();

    COPY_AND_ACCESS(alpha);
    COPY_AND_ACCESS(zeta);
    COPY_AND_ACCESS(beta);
    // TODO COPY_AND_ACCESS(gamma);
    // TODO COPY_AND_ACCESS(neptune);

    COPY_AND_ACCESS(alphao);
    COPY_AND_ACCESS(zetao);
    COPY_AND_ACCESS(betao);
    COPY_AND_ACCESS(gammao);
    COPY_AND_ACCESS(neptuneo);

    COPY_AND_ACCESS(alphaoi);
    COPY_AND_ACCESS(zetaoi);
    COPY_AND_ACCESS(betaoi);
    COPY_AND_ACCESS(gammaoi);
    COPY_AND_ACCESS(neptuneoi);

    // UNNAMED properties.
    COPY_AND_ACCESS(Constant);
    COPY_AND_ACCESS(Function);
    COPY_AND_ACCESS(Bar);

    COPY_AND_ACCESS_LIST(alphal);
    COPY_AND_ACCESS_LIST(zetal);
    COPY_AND_ACCESS_LIST(betal);
    COPY_AND_ACCESS_LIST(gammal);
    COPY_AND_ACCESS_LIST(neptunel);

    COPY_AND_ACCESS_LIST(alphal0);
    COPY_AND_ACCESS_LIST(zetal0);
    COPY_AND_ACCESS_LIST(betal0);
    COPY_AND_ACCESS_LIST(gammal0);
    COPY_AND_ACCESS_LIST(neptunel0);

    COPY_AND_ACCESS_LIST(alphals);
    COPY_AND_ACCESS_LIST(zetals);
    COPY_AND_ACCESS_LIST(betals);
    // TODO COPY_AND_ACCESS_LIST(gammals);
    COPY_AND_ACCESS_LIST(neptunels);

    COPY_AND_ACCESS_LIST(alphall);
    COPY_AND_ACCESS_LIST(zetall);
    COPY_AND_ACCESS_LIST(betall);
    // TODO COPY_AND_ACCESS_LIST(gammall);
    COPY_AND_ACCESS_LIST(neptunell);

    COPY_AND_ACCESS_LIST(alphalm);
    COPY_AND_ACCESS_LIST(zetalm);
    COPY_AND_ACCESS_LIST(betalm);
    COPY_AND_ACCESS_LIST(gammalm);
    COPY_AND_ACCESS_LIST(neptunelm);

    COPY_AND_ACCESS_LIST(alphalm0);
    COPY_AND_ACCESS_LIST(zetalm0);
    COPY_AND_ACCESS_LIST(betalm0);
    COPY_AND_ACCESS_LIST(gammalm0);
    COPY_AND_ACCESS_LIST(neptunelm0);

    COPY_AND_ACCESS_LIST(alphalr);
    COPY_AND_ACCESS_LIST(zetalr);
    COPY_AND_ACCESS_LIST(betalr);
    // TODO COPY_AND_ACCESS_LIST(gammalr);
    COPY_AND_ACCESS_LIST(neptunelr);
}

using namespace OpenSim;

int main(int argc, char* argv[]) {
    SimTK_START_TEST("testSerializationInClassInit");
        SimTK_SUBTEST(testCopyAndAccess<BackwardsCompatibility33>);
        // TODO SimTK_SUBTEST1(testCopyAndAccess, DefaultInitialized());
        // TODO SimTK_SUBTEST1(testCopyAndAccess, UserInitialized());
        // TODO SimTK_SUBTEST1(testCopyAndAccess, Uninitialized());
    SimTK_END_TEST();
}


