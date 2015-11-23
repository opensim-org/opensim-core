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
#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/SimmSpline.h>

using namespace OpenSim;
using namespace SimTK;
using SimTK::Vec3;

// We use deprecated versions of constructProperties_() for testing, but
// we don't to see the warnings since we use these methods intentionally.
#if defined(__clang__) || defined(__GNUG__)
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__MSC_VER)
    #pragma warning( disable : 4996 )
#endif

namespace OpenSim {

class Bar;
class Foo;

class MyComponent : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(MyComponent, Component);
public:
    MyComponent() = default;
    MyComponent(int val) : value(val) {}
    bool operator==(const MyComponent& other) const {
        return value == other.value;
    }
private:
    int value;
};

// TODO test deserialization, etc. or, at least the "value is default" stuff.

// TODO try an initializer list as the initial value of a property; making my
// own custom type. or use brace initialization to call the constructor of a
// class. e.g. OpenSim_DECLARE_PROPERTY(x, Constant, "", {1, 2, 3, 4}).

// TODO test deprecation warnings.
// TODO Due to a bug in our xml syntax, we must choose types for the unnamed
// properties that are not used for any of the other properties.
//
// TODO to properly find bugs, I must check that the properties have the
// CORRECT value.

// Helper macros.
// ==============

// Declare without an initial value argument.
// ------------------------------------------
// TODO test *ALL* if set to _UNINIT.
#define DECLARE_ALL_PROPERTIES(suffix) \
    DECLARE_REQUIRED_PROPERTIES(suffix) \
    DECLARE_UNNAMED_PROPERTIES(suffix) \
    DECLARE_OPTIONAL_PROPERTIES(suffix) \
    DECLARE_OPTIONAL_INIT_PROPERTIES(suffix) \
    DECLARE_LIST_PROPERTIES(suffix) \
    DECLARE_LIST_EMPTY_PROPERTIES(suffix) \
    DECLARE_LIST_SIZE_PROPERTIES(suffix) \
    DECLARE_LIST_ATLEAST_PROPERTIES(suffix) \
    DECLARE_LIST_ATMOST_PROPERTIES(suffix) \
    DECLARE_LIST_ATMOST_EMPTY_PROPERTIES(suffix) \
    DECLARE_LIST_RANGE_PROPERTIES(suffix)

#define DECLARE_REQUIRED_PROPERTIES(suffix) \
    OpenSim_DECLARE_PROPERTY##suffix(alpha, double, "built-in type."); \
    OpenSim_DECLARE_PROPERTY##suffix(zeta, Vec3, "non-Object."); \
    OpenSim_DECLARE_PROPERTY##suffix(beta, Constant, "Concrete class (included)."); \
    OpenSim_DECLARE_PROPERTY(gamma, Function, "Abstract class (included)."); \
    OpenSim_DECLARE_PROPERTY(neptune, Bar, "Forward-declared class.");
// TODO enable these other properties for the UNINIT case.

#define DECLARE_UNNAMED_PROPERTIES(suffix) \
    OpenSim_DECLARE_UNNAMED_PROPERTY##suffix(SimmSpline, "Concrete."); \
    OpenSim_DECLARE_UNNAMED_PROPERTY##suffix(Component, "Abstract."); \
    OpenSim_DECLARE_UNNAMED_PROPERTY##suffix(Foo, "Forward-declared."); 

#define DECLARE_OPTIONAL_PROPERTIES(suffix) \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(alphao, double, "built-in type."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(zetao, Vec3, "non-Object."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(betao, Constant, "Concrete."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(gammao, Function, "Abstract."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(neptuneo, Bar, "Forward-declared.");

// With an initial value provided.
#define DECLARE_OPTIONAL_INIT_PROPERTIES(suffix) \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(alphaoi, double, "built-in type."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(zetaoi, Vec3, "non-Object."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(betaoi, Constant, "Concrete."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(gammaoi, Function, "Abstract."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(neptuneoi, Bar, "Forward-declared.");

#define DECLARE_LIST_PROPERTIES(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(alphal, double, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(zetal, Vec3, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(betal, Constant, "Concrete."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(gammal, Function, "Abstract."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(neptunel, Bar, "Forward-declared.");

// These will use the constructProperty_() variant that doesn't take arguments.
#define DECLARE_LIST_EMPTY_PROPERTIES(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(alphal0, double, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(zetal0, Vec3, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(betal0, Constant, "Concrete."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(gammal0, Function, "Abstract."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(neptunel0, Bar, "Forward-declared.");

#define DECLARE_LIST_SIZE_PROPERTIES(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(alphals, double, 3, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(zetals, Vec3, 3, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(betals, Constant, 3, "Concrete."); \
    /* Don't know how to make a list property of abstract type that must have more */ \
    /* than 0 elements (SIZE, ATLEAST, RANGE). */ \
    /* TODO OpenSim_DECLARE_LIST_PROPERTY_SIZE(gammals, Function, 3, "Abstract."); */ \
    OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(neptunels, Bar, 3, "Forward-declared.");

#define DECLARE_LIST_ATLEAST_PROPERTIES(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(alphall, double, 2, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(zetall, Vec3, 2, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(betall, Constant, 2, "Concrete."); \
    /* TODO OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(gammall, Function, 2, "Abstract."); */ \
    OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(neptunell, Bar, 2, "Forward-declared.");

#define DECLARE_LIST_ATMOST_PROPERTIES(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(alphalm, double, 5, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(zetalm, Vec3, 5, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(betalm, Constant, 5, "Concrete."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(gammalm, Function, 5, "Abstract."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(neptunelm, Bar, 5, "Forward-declared class.");

#define DECLARE_LIST_ATMOST_EMPTY_PROPERTIES(suffix) \
    /* These will use the constructProperty_() variant that doesn't take arguments. */ \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(alphalm0, double, 5, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(zetalm0, Vec3, 5, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(betalm0, Constant, 5, "Concrete."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(gammalm0, Function, 5, "Abstract."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(neptunelm0, Bar, 5, "Forward-declared class.");

#define DECLARE_LIST_RANGE_PROPERTIES(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(alphalr, double, 4, 6, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(zetalr, Vec3, 4, 6, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(betalr, Constant, 4, 6, "Concrete."); \
    /* TODO OpenSim_DECLARE_LIST_PROPERTY_RANGE(gammalr, Function, 4, 6, "Abstract."); */ \
    OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(neptunelr, Bar, 4, 6, "Forward-declared");


// Declare *with* an initial value argument.
// -----------------------------------------
// "UI" stands for "USERINIT".
#define DECLARE_ALL_PROPERTIES_UI(suffix) \
    DECLARE_REQUIRED_PROPERTIES_UI(suffix) \
    DECLARE_UNNAMED_PROPERTIES_UI(suffix) \
    DECLARE_OPTIONAL_PROPERTIES_UI(suffix) \
    DECLARE_OPTIONAL_INIT_PROPERTIES_UI(suffix) \
    DECLARE_LIST_PROPERTIES_UI(suffix) \
    DECLARE_LIST_EMPTY_PROPERTIES_UI(suffix) \
    DECLARE_LIST_SIZE_PROPERTIES_UI(suffix) \
    DECLARE_LIST_ATLEAST_PROPERTIES_UI(suffix) \
    DECLARE_LIST_ATMOST_PROPERTIES_UI(suffix) \
    DECLARE_LIST_ATMOST_EMPTY_PROPERTIES_UI(suffix) \
    DECLARE_LIST_RANGE_PROPERTIES_UI(suffix)

#define DECLARE_REQUIRED_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_PROPERTY##suffix(alpha, double, "built-in type.", 12); \
    OpenSim_DECLARE_PROPERTY##suffix(zeta, Vec3, "non-Object.", Vec3(12, 8, 15)); \
    OpenSim_DECLARE_PROPERTY##suffix(beta, Constant, "Concrete class (included).", Constant(19)); \
    /* OpenSim_DECLARE_PROPERTY##suffix(gamma, Function, "Abstract class (included)."); */ \
    /* OpenSim_DECLARE_PROPERTY##suffix(neptune, Bar, "Forward-declared class."); */

#define DECLARE_UNNAMED_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_UNNAMED_PROPERTY##suffix(SimmSpline, "Concrete.", SimmSpline()); \
    /* OpenSim_DECLARE_UNNAMED_PROPERTY##suffix(Component, "Abstract.", MyComponent(12)); */ \
    /* OpenSim_DECLARE_UNNAMED_PROPERTY##suffix(Foo, "Forward-declared."); */

// TODO doesn't make sense. REMOVE.
#define DECLARE_OPTIONAL_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(alphao, double, "built-in type."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(zetao, Vec3, "non-Object."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(betao, Constant, "Concrete."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(gammao, Function, "Abstract."); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(neptuneo, Bar, "Forward-declared.");

// With an initial value provided.
#define DECLARE_OPTIONAL_INIT_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(alphaoi, double, "built-in type.", 52); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(zetaoi, Vec3, "non-Object.", Vec3(7, 1, 5)); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(betaoi, Constant, "Concrete.", Constant(1)); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(gammaoi, Function, "Abstract.", GCVSpline()); \
    OpenSim_DECLARE_OPTIONAL_PROPERTY##suffix(neptuneoi, Bar, "Forward-declared.", Bar(5));

#define DECLARE_LIST_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(alphal, double, "built-in type.", \
            std::vector<double>{1,2}); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(zetal, Vec3, "non-Object.", \
            {Vec3(0), Vec3(15), Vec3(17, 12, 1)}); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(betal, Constant, "Concrete.", \
            std::vector<Constant>{Constant(), Constant(6)}); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(gammal, Function, "Abstract.", \
            std::vector<Function>()); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(neptunel, Bar, "Forward-declared.", \
            {Bar(1.5), Bar(5)});

// These will use the constructProperty_() variant that doesn't take arguments.
// TODO doesn't make sense. REMOVE.
#define DECLARE_LIST_EMPTY_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(alphal0, double, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(zetal0, Vec3, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(betal0, Constant, "Concrete."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(gammal0, Function, "Abstract."); \
    OpenSim_DECLARE_LIST_PROPERTY##suffix(neptunel0, Bar, "Forward-declared.");

#define DECLARE_LIST_SIZE_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(alphals, double, 3, "built-in type.", \
            {6, 3, 1}); \
    OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(zetals, Vec3, 3, "non-Object.", \
            std::vector<Vec3>{Vec3(5), Vec3(0), Vec3(1)}); \
    OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(betals, Constant, 3, "Concrete.", \
            {Constant(60), Constant(5), Constant(21)}); \
    /* Don't know how to make a list property of abstract type that must have more */ \
    /* than 0 elements (SIZE, ATLEAST, RANGE). */ \
    /* TODO OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(gammals, Function, 3, "Abstract."); */ \
    OpenSim_DECLARE_LIST_PROPERTY_SIZE##suffix(neptunels, Bar, 3, "Forward-declared.", \
            std::vector<Bar>{Bar(5), Bar(12)});

#define DECLARE_LIST_ATLEAST_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(alphall, double, 2, "built-in type.", \
            {5, 1, 17}); \
    OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(zetall, Vec3, 2, "non-Object.", \
            {Vec3(12), Vec3(10)}); \
    OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(betall, Constant, 2, "Concrete.", \
            {Constant(7), Constant(0), Constant(-16), Constant(-10)}); \
    /* TODO OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(gammall, Function, 2, "Abstract."); */ \
    OpenSim_DECLARE_LIST_PROPERTY_ATLEAST##suffix(neptunell, Bar, 2, "Forward-declared.", \
            {Bar(4.1), Bar(9)});

#define DECLARE_LIST_ATMOST_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(alphalm, double, 5, "built-in type.", \
            {19, 12, 6, 1, 4}); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(zetalm, Vec3, 5, "non-Object.", \
            {Vec3(-1, -2, -3), Vec3(-7, 5, 3)}); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(betalm, Constant, 5, "Concrete.", \
            {Constant(-58)}); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(gammalm, Function, 5, "Abstract.", \
            std::vector<Function>()); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(neptunelm, Bar, 5, "Forward-declared class.", \
            {Bar(5), Bar(10), Bar(-5)});

// TODO doesn't make sense. REMOVE.
#define DECLARE_LIST_ATMOST_EMPTY_PROPERTIES_UI(suffix) \
    /* These will use the constructProperty_() variant that doesn't take arguments. */ \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(alphalm0, double, 5, "built-in type."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(zetalm0, Vec3, 5, "non-Object."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(betalm0, Constant, 5, "Concrete."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(gammalm0, Function, 5, "Abstract."); \
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST##suffix(neptunelm0, Bar, 5, "Forward-declared class.");

#define DECLARE_LIST_RANGE_PROPERTIES_UI(suffix) \
    OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(alphalr, double, 4, 6, "built-in type.", \
            {1, 2, 7, 5, 3, 9}); \
    OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(zetalr, Vec3, 4, 6, "non-Object.", \
            {Vec3(-3), Vec3(0), Vec3(-15, 1, 3), Vec3(-1), Vec3(5)}); \
    OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(betalr, Constant, 4, 6, "Concrete.", \
            {Constant(1), Constant(), Constant(5), Constant(-1)}); \
    /* TODO OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(gammalr, Function, 4, 6, "Abstract."); */ \
    OpenSim_DECLARE_LIST_PROPERTY_RANGE##suffix(neptunelr, Bar, 4, 6, "Forward-declared", \
            Array<Bar>(Bar(5), 4));


// Construct.
// ----------
#define CONSTRUCT_ALL_PROPERTIES \
    CONSTRUCT_REQUIRED_PROPERTIES \
    CONSTRUCT_UNNAMED_PROPERTIES \
    CONSTRUCT_OPTIONAL_PROPERTIES \
    CONSTRUCT_OPTIONAL_INIT_PROPERTIES \
    CONSTRUCT_LIST_PROPERTIES \
    CONSTRUCT_LIST_EMPTY_PROPERTIES \
    CONSTRUCT_LIST_SIZE_PROPERTIES \
    CONSTRUCT_LIST_ATLEAST_PROPERTIES \
    CONSTRUCT_LIST_ATMOST_PROPERTIES \
    CONSTRUCT_LIST_ATMOST_EMPTY_PROPERTIES \
    CONSTRUCT_LIST_RANGE_PROPERTIES

#define CONSTRUCT_REQUIRED_PROPERTIES \
    constructProperty_alpha(8); \
    constructProperty_zeta(Vec3(0)); \
    constructProperty_beta(Constant()); \
    constructProperty_gamma(GCVSpline()); \
    constructProperty_neptune(Bar());

#define CONSTRUCT_UNNAMED_PROPERTIES \
    constructProperty_SimmSpline(SimmSpline()); \
    constructProperty_Component(MyComponent()); \
    constructProperty_Foo(Foo());

#define CONSTRUCT_OPTIONAL_PROPERTIES \
    constructProperty_alphao(); \
    constructProperty_zetao(); \
    constructProperty_betao(); \
    constructProperty_gammao(); \
    constructProperty_neptuneo(); \

#define CONSTRUCT_OPTIONAL_INIT_PROPERTIES \
    constructProperty_alphaoi(13); \
    constructProperty_zetaoi(Vec3(36, 13, 1)); \
    constructProperty_betaoi(Constant(13)); \
    constructProperty_gammaoi(GCVSpline()); \
    constructProperty_neptuneoi(Bar());

#define CONSTRUCT_LIST_PROPERTIES \
    constructProperty_alphal(std::vector<double>{6, 3, 2, 7}); \
    constructProperty_zetal(std::vector<Vec3>{Vec3(1.5, 3, 4.6), Vec3(6)}); \
    constructProperty_betal(std::vector<Constant>{Constant()}); \
    constructProperty_gammal(std::vector<Function>()); \
    constructProperty_neptunel(std::vector<Bar>{Bar(), Bar(12)});

#define CONSTRUCT_LIST_EMPTY_PROPERTIES \
    constructProperty_alphal0(); \
    constructProperty_zetal0(); \
    constructProperty_betal0(); \
    constructProperty_gammal0(); \
    constructProperty_neptunel0(); \

#define CONSTRUCT_LIST_SIZE_PROPERTIES \
    constructProperty_alphals(std::vector<double>{6, 5, 4}); \
    constructProperty_zetals(std::vector<Vec3>{Vec3(1), Vec3(9), Vec3(6)}); \
    constructProperty_betals( \
            std::vector<Constant>{Constant(), Constant(), Constant()}); \
    /* TODO constructProperty_gammals(std::vector<Function>{Constant()}); */ \
    constructProperty_neptunels(std::vector<Bar>{Bar(), Bar(), Bar(10)});

#define CONSTRUCT_LIST_ATLEAST_PROPERTIES \
    constructProperty_alphall(std::vector<double>{8, 6}); \
    constructProperty_zetall(std::vector<Vec3>{Vec3(10), Vec3(1), Vec3(9)}); \
    constructProperty_betall(std::vector<Constant>{Constant(), Constant()}); \
    /* TODO constructProperty_gammall(std::vector<Function>{Constant()}); */\
    constructProperty_neptunell(std::vector<Bar>{Bar(), Bar(), Bar(), Bar()});

#define CONSTRUCT_LIST_ATMOST_PROPERTIES \
    constructProperty_alphalm(std::vector<double>{1, 8, 7}); \
    constructProperty_zetalm(std::vector<Vec3>()); \
    constructProperty_betalm(std::vector<Constant>{Constant()}); \
    constructProperty_gammalm(std::vector<Function>()); \
    constructProperty_neptunelm( \
            std::vector<Bar>{Bar(5), Bar(2), Bar(), Bar(), Bar()});

#define CONSTRUCT_LIST_ATMOST_EMPTY_PROPERTIES \
    constructProperty_alphalm0(); \
    constructProperty_zetalm0(); \
    constructProperty_betalm0(); \
    constructProperty_gammalm0(); \
    constructProperty_neptunelm0(); \

#define CONSTRUCT_LIST_RANGE_PROPERTIES \
    constructProperty_alphalr(std::vector<double>{1, 2, 3, 4, 5}); \
    constructProperty_zetalr( \
            std::vector<Vec3>{Vec3(0), Vec3(1), Vec3(2), Vec3(3), Vec3(4), Vec3(5)}); \
    constructProperty_betalr( \
            std::vector<Constant>{Constant(), Constant(), Constant(), Constant()}); \
    /* TODO constructProperty_gammalr(std::vector<Function>{GCVSpline()}); */ \
    constructProperty_neptunelr(std::vector<Bar>{Bar(), Bar(), Bar(), Bar()});


// This is to ensure all properties have values. We only set properties that
// don't have a default value.
#define SET_ALL_PROPERTY_VALUES \
    SET_OPTIONAL_PROPERTY_VALUES \
    SET_LIST_PROPERTY_VALUES \
    SET_LIST_EMPTY_PROPERTY_VALUES \
    SET_LIST_ATMOST_PROPERTY_VALUES \
    SET_LIST_ATMOST_EMPTY_PROPERTY_VALUES

#define SET_OPTIONAL_PROPERTY_VALUES \
    set_alphao(25); \
    set_zetao(Vec3(3, 1, 4)); \
    set_betao(Constant(42)); \
    set_gammao(GCVSpline()); \
    set_neptuneo(Bar());

#define SET_LIST_PROPERTY_VALUES \
    append_gammal(Constant(1));

#define SET_LIST_EMPTY_PROPERTY_VALUES \
    append_alphal0(91); \
    append_zetal0(Vec3(15)); \
    append_betal0(Constant(12)); \
    append_gammal0(GCVSpline()); \
    append_neptunel0(Bar());

#define SET_LIST_ATMOST_PROPERTY_VALUES \
    append_gammalm(GCVSpline());

#define SET_LIST_ATMOST_EMPTY_PROPERTY_VALUES \
    append_alphalm0(5); \
    append_zetalm0(Vec3(8)); \
    append_betalm0(Constant(10)); \
    append_gammalm0(Constant(8)); \
    append_neptunelm0(Bar());


#define COPY_ALL_PROPERTIES \
    COPY_REQUIRED_PROPERTIES \
    COPY_UNNAMED_PROPERTIES \
    COPY_OPTIONAL_PROPERTIES \
    COPY_OPTIONAL_INIT_PROPERTIES \
    COPY_LIST_PROPERTIES \
    COPY_LIST_EMPTY_PROPERTIES \
    COPY_LIST_SIZE_PROPERTIES \
    COPY_LIST_ATLEAST_PROPERTIES \
    COPY_LIST_ATMOST_PROPERTIES \
    COPY_LIST_ATMOST_EMPTY_PROPERTIES \
    COPY_LIST_RANGE_PROPERTIES

#define COPY_REQUIRED_PROPERTIES \
    copyProperty_alpha(source); \
    copyProperty_zeta(source); \
    copyProperty_beta(source); \
    /* TODO copyProperty_gamma(source); */ \
    /* TODO copyProperty_neptune(source); */ \

#define COPY_UNNAMED_PROPERTIES \
    copyProperty_SimmSpline(source); \
    /* copyProperty_Component(source); */ \
    /* copyProperty_Foo(source); */

#define COPY_OPTIONAL_PROPERTIES \
    copyProperty_alphao(source); \
    copyProperty_zetao(source); \
    copyProperty_betao(source); \
    copyProperty_gammao(source); \
    copyProperty_neptuneo(source);

#define COPY_OPTIONAL_INIT_PROPERTIES \
    copyProperty_alphaoi(source); \
    copyProperty_zetaoi(source); \
    copyProperty_betaoi(source); \
    copyProperty_gammaoi(source); \
    copyProperty_neptuneoi(source);

#define COPY_LIST_PROPERTIES \
    copyProperty_alphal(source); \
    copyProperty_zetal(source); \
    copyProperty_betal(source); \
    copyProperty_gammal(source); \
    copyProperty_neptunel(source);

#define COPY_LIST_EMPTY_PROPERTIES \
    copyProperty_alphal0(source); \
    copyProperty_zetal0(source); \
    copyProperty_betal0(source); \
    copyProperty_gammal0(source); \
    copyProperty_neptunel0(source);

#define COPY_LIST_SIZE_PROPERTIES \
    copyProperty_alphals(source); \
    copyProperty_zetals(source); \
    copyProperty_betals(source); \
    /* TODO copyProperty_gammals(source); */  \
    copyProperty_neptunels(source);

#define COPY_LIST_ATLEAST_PROPERTIES \
    copyProperty_alphall(source); \
    copyProperty_zetall(source); \
    copyProperty_betall(source); \
    /* TODO copyProperty_gammall(source); */ \
    copyProperty_neptunell(source);

#define COPY_LIST_ATMOST_PROPERTIES \
    copyProperty_alphalm(source); \
    copyProperty_zetalm(source); \
    copyProperty_betalm(source); \
    copyProperty_gammalm(source); \
    copyProperty_neptunelm(source);

#define COPY_LIST_ATMOST_EMPTY_PROPERTIES \
    copyProperty_alphalm0(source); \
    copyProperty_zetalm0(source); \
    copyProperty_betalm0(source); \
    copyProperty_gammalm0(source); \
    copyProperty_neptunelm0(source);

#define COPY_LIST_RANGE_PROPERTIES \
    copyProperty_alphalr(source); \
    copyProperty_zetalr(source); \
    copyProperty_betalr(source); \
    /* TODO copyProperty_gammalr(source); */  \
    copyProperty_neptunelr(source);


// Class declarations.
// ===================

// This class shows all the property declarations/constructions that were
// possible (to the best of my knowledge) in OpenSim 3.3. Some of the
// declarations are commented out since we have made them
// backwards-incompatible. Thus, this class shows what users might have to
// change in their code when upgrading from OpenSim 3.3 to 4.0.
class BackwardsCompatibility33 : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(BackwardsCompatibility33, Object);
public:
    DECLARE_ALL_PROPERTIES()
    BackwardsCompatibility33();
    void setPropertyValues();
};

// Similar to the above class, but this time we implement a custom copy
// constructor and thus must call copyProperty_()
class BackwardsCompatibility33CustomCopy : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(BackwardsCompatibility33CustomCopy, Object);
public:
    DECLARE_ALL_PROPERTIES()
    BackwardsCompatibility33CustomCopy();
    // Copy constructor.
    BackwardsCompatibility33CustomCopy(const BackwardsCompatibility33CustomCopy&);
    void setPropertyValues();
};

// TODO may be unnecessary; achieved by backwards-compatibility.
//class DefaultInitialized : public Object {
//    OpenSim_DECLARE_CONCRETE_OBJECT(DefaultInitialized, Object);
//public:
//    // TODO
//};

// Test the new _UNINIT macro variants that are introduced in version 4.0.
class Uninitialized40 : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Uninitialized40, Object);
public:
    DECLARE_REQUIRED_PROPERTIES(_UNINIT)
    DECLARE_UNNAMED_PROPERTIES(_UNINIT)
    Uninitialized40();
    void setPropertyValues();
};

class Uninitialized40CustomCopy : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Uninitialized40CustomCopy, Object);
public:
    DECLARE_REQUIRED_PROPERTIES(_UNINIT)
    DECLARE_UNNAMED_PROPERTIES(_UNINIT)
    Uninitialized40CustomCopy();
    Uninitialized40CustomCopy(const Uninitialized40CustomCopy&);
    void setPropertyValues();
};

// Test the new macro variants that take a default value.
class UserInitialized40 : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(UserInitialized40, Object);
public:
    DECLARE_REQUIRED_PROPERTIES_UI(_USERINIT)
    DECLARE_UNNAMED_PROPERTIES_UI(_USERINIT)
    // TODO more properties.

    // No need to separately construct the properties, so we don't need
    // to define the default constructor ourselves :).

    void setPropertyValues();
};

class UserInitialized40CustomCopy : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(UserInitialized40CustomCopy, Object);
public:
    DECLARE_REQUIRED_PROPERTIES_UI(_USERINIT)
    DECLARE_UNNAMED_PROPERTIES_UI(_USERINIT)
    // TODO more properties.

    UserInitialized40CustomCopy() = default;
    UserInitialized40CustomCopy(const UserInitialized40CustomCopy&);

    void setPropertyValues();
};

// Definition of forward-declared class.
class Bar : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Bar, Object);
public:
    Bar() : value(0) {}
    Bar(int val) : value(val) {}
    bool operator==(const Bar& other) const {
        return value == other.value;
    }
private:
    int value;
};
class Foo : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Foo, Object);
public:
    Foo() : value(0) {}
    Foo(int val) : value(val) {}
    bool operator==(const Foo& other) const {
        return value == other.value;
    }
private:
    int value;
};

// Member definitions.
// ===================

// BackwardsCompatibility33
// ------------------------
BackwardsCompatibility33::BackwardsCompatibility33() {
    CONSTRUCT_ALL_PROPERTIES
}

void BackwardsCompatibility33::setPropertyValues() {
    SET_ALL_PROPERTY_VALUES
}

// BackwardsCompatibility33CustomCopy
// ----------------------------------
BackwardsCompatibility33CustomCopy::BackwardsCompatibility33CustomCopy() {
    CONSTRUCT_ALL_PROPERTIES
}

BackwardsCompatibility33CustomCopy::BackwardsCompatibility33CustomCopy(
        const BackwardsCompatibility33CustomCopy& source) : Object(source) {
    COPY_ALL_PROPERTIES
}

void BackwardsCompatibility33CustomCopy::setPropertyValues() {
    SET_ALL_PROPERTY_VALUES
}

// Uninitialized40
// ---------------
Uninitialized40::Uninitialized40() {
    CONSTRUCT_REQUIRED_PROPERTIES
    CONSTRUCT_UNNAMED_PROPERTIES
    // TODO include the rest of the properties
}

void Uninitialized40::setPropertyValues() {
// TODO     SET_PROPERTY_VALUES
}

// Uninitialized40CustomCopy
// -------------------------
Uninitialized40CustomCopy::Uninitialized40CustomCopy() {
    CONSTRUCT_REQUIRED_PROPERTIES
    CONSTRUCT_UNNAMED_PROPERTIES
    // TODO include the rest of the properties
}

Uninitialized40CustomCopy::Uninitialized40CustomCopy(
        const Uninitialized40CustomCopy& source) : Object(source) {
    COPY_REQUIRED_PROPERTIES
    COPY_UNNAMED_PROPERTIES
    // TODO include the rest of the properties
}

void Uninitialized40CustomCopy::setPropertyValues() {
// TODO     SET_PROPERTY_VALUES
}

// UserInitialized40
// -----------------
void UserInitialized40::setPropertyValues() {
// TODO     SET_PROPERTY_VALUES
}

// UserInitialized40CustomCopy
// ---------------------------
UserInitialized40CustomCopy::UserInitialized40CustomCopy(
        const UserInitialized40CustomCopy& source) : Object(source) {
    COPY_REQUIRED_PROPERTIES
    COPY_UNNAMED_PROPERTIES
    // TODO include the rest of the properties
}

void UserInitialized40CustomCopy::setPropertyValues() {
// TODO    SET_PROPERTY_VALUES
}

} // namespace

// TODO remove 
    /*std::cout << #prop_name << ": " << obj.get_##prop_name() << " " << copy->get_##prop_name() << std::endl; \*/
#define COMPARE_TO_COPY(prop_name) \
    SimTK_ASSERT_ALWAYS(obj.get_##prop_name() == copy->get_##prop_name(), \
            "Property '" #prop_name "' was not copied correctly.");

#define COMPARE_TO_COPY_LIST(prop_name) \
    for (int i = 0; i < obj.getProperty_##prop_name().size(); ++i) { \
        SimTK_ASSERT_ALWAYS(obj.get_##prop_name(i) == copy->get_##prop_name(i), \
                "Property '" #prop_name "' was not copied correctly."); \
    }

/* For backwards compatibility, we have in some cases changed the
implementation of constructProperty_prop_name() to just call set_prop_name().
However, constructProperty_prop_name() marks the provided value as default,
whereas set_prop_name() marks the value as *not* default.  However, in this
particular case, the value *should* be default to preserve behavior. So this
test just ensures that our new implementations of constructProperty_prop_name()
preserves the default value. We test this by making sure that no properties are
written when we serialize, since that's the correct behavior when the defaults
are set up correctly. */
template <typename T>
void testDefaults() {
    T obj;
    obj.print("default_" + obj.getConcreteClassName() + ".xml");

    Xml::Document xml("default_" + obj.getConcreteClassName() + ".xml");
    auto elem = xml.getRootElement().getRequiredElement(obj.getConcreteClassName());
    // Must use filter since there *should* be a comment node, and we want to
    // ignore it.
    SimTK_ASSERT_ALWAYS(!elem.hasNode(Xml::ElementNode),
            "All of the properties of this object have their default value, "
            "but were not properly marked as such.");
}

template <typename T>
void testCopyAndAccess() {
    T obj;
    obj.setPropertyValues();
    auto* copy = obj.clone();

    COMPARE_TO_COPY(alpha);
    COMPARE_TO_COPY(zeta);
    COMPARE_TO_COPY(beta);
    // TODO COMPARE_TO_COPY(gamma);
    // TODO COMPARE_TO_COPY(neptune);

    COMPARE_TO_COPY(SimmSpline);
    // TODO COMPARE_TO_COPY(Component);
    // TODO COMPARE_TO_COPY(Foo);

    /* TODO
    COMPARE_TO_COPY(alphao);
    COMPARE_TO_COPY(zetao);
    COMPARE_TO_COPY(betao);
    COMPARE_TO_COPY(gammao);
    COMPARE_TO_COPY(neptuneo);

    COMPARE_TO_COPY(alphaoi);
    COMPARE_TO_COPY(zetaoi);
    COMPARE_TO_COPY(betaoi);
    COMPARE_TO_COPY(gammaoi);
    COMPARE_TO_COPY(neptuneoi);

    COMPARE_TO_COPY_LIST(alphal);
    COMPARE_TO_COPY_LIST(zetal);
    COMPARE_TO_COPY_LIST(betal);
    COMPARE_TO_COPY_LIST(gammal);
    COMPARE_TO_COPY_LIST(neptunel);

    COMPARE_TO_COPY_LIST(alphal0);
    COMPARE_TO_COPY_LIST(zetal0);
    COMPARE_TO_COPY_LIST(betal0);
    COMPARE_TO_COPY_LIST(gammal0);
    COMPARE_TO_COPY_LIST(neptunel0);

    COMPARE_TO_COPY_LIST(alphals);
    COMPARE_TO_COPY_LIST(zetals);
    COMPARE_TO_COPY_LIST(betals);
    // TODO COMPARE_TO_COPY_LIST(gammals);
    COMPARE_TO_COPY_LIST(neptunels);

    COMPARE_TO_COPY_LIST(alphall);
    COMPARE_TO_COPY_LIST(zetall);
    COMPARE_TO_COPY_LIST(betall);
    // TODO COMPARE_TO_COPY_LIST(gammall);
    COMPARE_TO_COPY_LIST(neptunell);

    COMPARE_TO_COPY_LIST(alphalm);
    COMPARE_TO_COPY_LIST(zetalm);
    COMPARE_TO_COPY_LIST(betalm);
    COMPARE_TO_COPY_LIST(gammalm);
    COMPARE_TO_COPY_LIST(neptunelm);

    COMPARE_TO_COPY_LIST(alphalm0);
    COMPARE_TO_COPY_LIST(zetalm0);
    COMPARE_TO_COPY_LIST(betalm0);
    COMPARE_TO_COPY_LIST(gammalm0);
    COMPARE_TO_COPY_LIST(neptunelm0);

    COMPARE_TO_COPY_LIST(alphalr);
    COMPARE_TO_COPY_LIST(zetalr);
    COMPARE_TO_COPY_LIST(betalr);
    // TODO COMPARE_TO_COPY_LIST(gammalr);
    COMPARE_TO_COPY_LIST(neptunelr);
    */
}


int main(int argc, char* argv[]) {
    SimTK_START_TEST("testSerializationInClassInit");

        // Register types.
        Object::registerType(Bar());
        Object::registerType(Foo());
        Object::registerType(MyComponent());
        Object::registerType(BackwardsCompatibility33());
        Object::registerType(BackwardsCompatibility33CustomCopy());
        Object::registerType(Uninitialized40());
        Object::registerType(Uninitialized40CustomCopy());
        Object::registerType(UserInitialized40());
        Object::registerType(UserInitialized40CustomCopy());

        SimTK_SUBTEST(testCopyAndAccess<BackwardsCompatibility33>);
        SimTK_SUBTEST(testCopyAndAccess<BackwardsCompatibility33CustomCopy>);
        // TODO SimTK_SUBTEST(testCopyAndAccess<DefaultInitialized>);
        SimTK_SUBTEST(testCopyAndAccess<Uninitialized40>);
        SimTK_SUBTEST(testCopyAndAccess<Uninitialized40CustomCopy>);
        SimTK_SUBTEST(testCopyAndAccess<UserInitialized40>);
        SimTK_SUBTEST(testCopyAndAccess<UserInitialized40CustomCopy>);

        SimTK_SUBTEST(testDefaults<BackwardsCompatibility33>);
        SimTK_SUBTEST(testDefaults<BackwardsCompatibility33CustomCopy>);
        // TODO SimTK_SUBTEST(testDefaults<DefaultInitialized>);
        SimTK_SUBTEST(testDefaults<Uninitialized40>);
        SimTK_SUBTEST(testDefaults<Uninitialized40CustomCopy>);
        SimTK_SUBTEST(testDefaults<UserInitialized40>);
        SimTK_SUBTEST(testDefaults<UserInitialized40CustomCopy>);

    SimTK_END_TEST();
}


