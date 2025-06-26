Guidelines for Contributing to OpenSim-Core
===========================================
OpenSim is a community resource that is housed in the OpenSim-Core repository.
We encourage everyone to contribute to the OpenSim project. This could be adding new code for a feature, improving an algorithm, or letting us know about a bug or making a feature request. The purpose of our contribution policy is to ensure that all code in OpenSim-Core has undergone real scrutiny, thereby reducing the likelihood of errors.

Even great programmers benefit from additional eyeballs on their code to catch bugs, avoid duplication and inefficiency, watch for standards violations, and to check that the code is as clear to others as it is to its author. We appreciate contributions and our development team is collaborative and constructive -- don't be shy!

**Public advisory:** OpenSim is an open source project licensed under very liberal terms intended to encourage use by *anyone*, for *any purpose*. When you make a contribution to the OpenSim project, **you are agreeing** to do so under those same terms. The details are [below](#contributor-license-agreement). If you are uncomfortable with these terms, please [contact us](https://github.com/orgs/opensim-org/teams/owners) before contributing.

Contents:

- [Ways to Contribute](#ways-to-contribute)
- [Making a Pull Request](#making-a-pull-request-pr)
- [Writing tests](#writing-tests)
- [Running Moco tests](#running-moco-tests)
- [Checking for Memory Leaks with LibASAN](#checking-for-memory-leaks-with-libasan)
- [Coding Standards](#coding-standards)
- [List of Contributors](#list-of-contributors)
- [Contributor License Agreement](#contributor-license-agreement)

Thank you for contributing!

Ways to Contribute
------------------
There are lots of ways to contribute to the OpenSim project, and people with widely varying skill sets can make meaningful contributions. Please don't think your contribution has to be momentous to be appreciated. See a typo? Tell us about it or fix it! Here are some contribution ideas:

- Use OpenSim and let us know how you're using it by posting to the [OpenSim user forum](https://simtk.org/forums/viewforum.php?f=91).  
- Ask and/or answer questions on the forum.
- File bug reports, documentation problems, feature requests, and developer discussion topics using the GitHub [Issue tracker](https://github.com/opensim-org/opensim-core/issues).
- Submit GitHub Pull Requests providing new features, examples, or bug fixes to code or documentation (see below).
- If our hard work has helped you with yours, please consider acknowledging your use of OpenSim and encourage others to do so. Please cite the following papers:

    Delp SL, Anderson FC, Arnold AS, Loan P, Habib A, John CT, Guendelman E, Thelen DG (2007) OpenSim: open-source software to create and analyze dynamic simulations of movement. *IEEE Trans Biomed Eng* 54:1940–50.

    Seth A, Hicks JL, Uchida TK, Habib A, Dembia CL, Dunne JJ, Ong CF, DeMers MS, Rajagopal A, Millard M, Hamner SR, Arnold EM, Yong JR, Lakshmikanth SK, Sherman MA, Ku JP, Delp SL (2018) OpenSim: Simulating musculoskeletal dynamics and neuromuscular control to study human and animal movement. *PLoS Computational Biology* 14(7):e1006223.



Making a Pull Request (PR)
--------------------------
Please don't surprise us with a big out-of-the-blue pull request (PR). Preferably, target an existing Issue and post a comment to that effect in the Issue. That way, you can engage in discussion with others to coordinate your work with theirs and avoid duplication, see what people think of the approach you have planned, and so on. If there is not yet a relevant Issue in place, a great way to start is by creating one, and then engaging in an Issue conversation.

If you have never made a PR before, there are excellent GitHub tutorials to familiarize you with the concepts and steps like the [GitHub Bootcamp](https://help.github.com/categories/bootcamp/).

Most PRs will need to be reviewed by at least one member of the OpenSim Dev Team. The Dev Team is a Team within the opensim-org GitHub organization. Dev Team members are listed [here](https://github.com/orgs/opensim-org/teams/dev-team/members).

When you are ready to make a PR, please adhere to the guidelines below. 

1. To help the people who review your pull request, include a detailed description of what changes or additions the PR includes and what you have done to test and verify the changes. Reference the existing Issue (or set of Issues) that documents the problem the PR is adressing (e.g., use ``#<issue_number>`` ). Also, please make sure detailed information about the commits is easily available.

2. Make sure that your request conforms to our [coding standards](#coding-standards).

3. Make sure that your code executes as intended and that *all* tests pass on your local machine before making a pull request. The [README.md](https://github.com/opensim-org/opensim-core) explains how to run the tests. If your changes introduce runtime options or branching in the code, please ensure that all options or branches are being tested and that exceptions are being thrown in invalid scenarios.

4. Typo fixes can be merged by any member of the Development (Dev) Team.

5. Updates to comments, Doxygen, compiler compatibility, CMake files, or continuous integration files must be reviewed by at least one member of the Dev Team before being merged. The original author or the reviewer(s) may merge the pull request.

6. Any other changes to the code require review by one member of the Dev Team, and manual testing by one member of the Dev Team. A reviewer may determine that a second reviewer from the Dev Team is required for the pull request to be accepted; you may also suggest whether the pull request should require one or two reviewers. You or the reviewers may merge the pull request once the reviewers accept the pull request.

7. If your pull request involves adding a new class or performing a major object or algorithm refactor, the Dev Team can assist with assigning appropriate reviewers. 

8. As the changes introduced by your pull request become finalized throughout the review process, you should decide if your changes warrant being mentioned in the change log. If so, update the [CHANGELOG.md](https://github.com/opensim-org/opensim-core/blob/master/CHANGELOG.md) with an additional commit to your pull request.

9. CI must be run for all changes, except Matlab tests and examples, non-Doxygen markdown files, and non-Doxygen comments. CI tests must pass before merge, unless the author and reviewer(s) deem the failures unrelated to the change.

A few additional practices will help streamline the code review process. Please use tags (i.e., @user_name) and quoting to help keep the discussion organized. Please also call for a meeting or Skype call when discussions start to stagnate. In addition, we recommend getting input on your interface design before implementing a major new component or other change.

It is important that reviewers also review the effect that your PR has on the doxygen documentation. To facilitate this, we automatically upload the doxygen documentation for each PR to [myosin.sourceforge.net](http://myosin.sourceforge.net); you can view the documentation for a specific PR at `myosin.sourceforge.net/<issue-number>`.


Writing tests
-------------
There are directories of tests scattered throughout the repository. Each
directory contains most of the files (e.g., .osim model files) necessary to run
the tests. When building, these files are copied over into the build tree. Some
of these files are used by multiple tests from different parts of the
repository. To avoid duplicating such files within the repository, the files should
go into the `OpenSim/Tests/shared` directory. You can then use the
`OpenSimCopySharedTestFiles` CMake macro (in `cmake/OpenSimMacros.osim`) to
copy the necessary shared files to the proper build directory. *DO NOT CHANGE* files
that are already in `OpenSim/Tests/shared`; you could inadvertently weaken tests
that rely on some obscure aspect of the files.


Platform-specific tests
-----------------------
The Catch2 testing framework can be used to write platform-specific tests. Such
tests should be avoided if possible, but may be necessary in certain cases (e.g.,
slightly different numerical optimization behavior on different platforms). For
example, if you want to write a test that only runs on Windows, you can use the
`TEST_CASE()` or `TEMPLATE_TEST_CASE` macros with the tag `"[win]"`:

```cpp
TEST_CASE("MyTest", "[win]") {
    // ...
}
```

```cpp
TEMPLATE_TEST_CASE("MyTest", "[win]", TestType) {
    // ...
}
```

Specifying the tag `"[win]"` means that this test will be _excluded_ on Mac and
Linux. The tags `"[mac]"` and `"[linux]"` can be used similarly for tests specific 
to Mac or Linux. If you want to run a test on two platforms but not the third,
use combined tags (e.g., `"[win/linux]"`, `"[mac/win]"`, or `"[unix]"`). Do not
concatenate tags; for example, `"[win][linux]"`) will not run on Linux or Windows
since Windows excludes Linux-only tests and vice-versa. Specifying no tag means 
that the test will run on all platforms. The table below summarizes the tags that 
can be used to specify platform-specific tests.

| Platform(s)       | Tag(s)                                       |
|-------------------|----------------------------------------------|
| Windows           | `"[win]"`                                    |
| Mac               | `"[mac]"`                                    |
| Linux             | `"[linux]"`                                  |
| Mac and Linux     | `"[unix]"`, `"[mac/linux]"`, `"[linux/mac]"` |
| Windows and Mac   | `"[win/mac]"`, `"[mac/win]"`                 |
| Windows and Linux | `"[win/linux]"`, `"[linux/win]"`             |
| All platforms     | (no tag)                                     |

Building GUI
-------------
In case you suspect your changes to the opensim-core is breaking API (can break client
code including the OpenSim application/GUI) you can include the tag [build-gui] in the
commit message and a separate build of the GUI (from the opensim-gui repository) will be
triggered.

Running Moco tests
------------------
In general, Moco's tests depend on the CasADi library, whose use is determined 
by the `OPENSIM_WITH_CASADI` and CMake variable. The CTests are designed to 
succeed regardless of the value of the CMake variable: if `OPENSIM_WITH_CASADI` 
is off, Moco's C++ tests are run with arguments `"~*MocoCasADiSolver*" "~[casadi]"`, 
which excludes Catch2 templatized tests using MocoCasADiSolver and other tests that 
are tagged as relying on CasADi. If the test executables are run without CTest (e.g., 
debugging a project in Visual Studio), the tests will fail if `OPENSIM_WITH_CASADI` is 
false; for the tests to pass, provide the argument `"~*MocoCasADiSolver*" "~[casadi]"`.

Checking for Memory Leaks with LibASAN
--------------------------------------

The easiest way to check for memory leaks is to use LibASAN on Linux or Mac. If you are
using Windows 11 then you can use WSL2 to create a Linux virtual machine. Alternatively, you
can use Windows' native support for libASAN, which is documented [here](https://devblogs.microsoft.com/cppblog/addresssanitizer-asan-for-windows-with-msvc/).

Make sure you have the `clang` and `clang++` C/C++ compilers installed (best
to google this), and then compile OpenSim in a terminal with the relevant flags:

```bash
# enable libASAN when compiling C++ sources
export CXXFLAGS="-fsanitize=address"

# link the libASAN runtime when linking binaries
export LDFLAGS="-fsanitize=address"

# configure dependencies, for example:
cmake -S dependencies/ -B ../osim-deps-build -DCMAKE_INSTALL_PREFIX=${PWD}/../osim-deps-install

# build dependencies, for example:
cmake --build ../osim-deps-build/

# configure OpenSim, for example:
cmake -S . -B ../osim-build -DOPENSIM_DEPENDENCIES_PATH=${PWD}/../osim-deps-install

# build OpenSim, for example:
cmake --build ../osim-build/

# (then run/test something: the runtime should now perform memory checks)
```


Coding Standards
----------------
- [Header guards](#header-guards)
- [Creating new OpenSim objects](#creating-new-opensim-objects)
- [Assignment operators in C++](#assignment-operators-in-c)
- [Documenting your code](#documenting-your-code)
- [Each line of text should be at most 80 characters](#each-line-of-text-should-be-at-most-80-characters)
- [Replace tabs with four spaces](#replace-tabs-with-four-spaces)
- [Renaming classes in the OpenSim API](#renaming-classes-in-the-opensim-api)
- [Naming conventions](#naming-conventions)
- [Throw and return are not functions](#throw-and-return-are-not-functions)
- [Always use pre-increment and pre-decrement operators when you have a choice](#always-use-pre-increment-and-pre-decrement-operators-when-you-have-a-choice)
- [Place pointer and reference symbols with the type](#place-pointer-and-reference-symbols-with-the-type)
- [Setters and pass-by-value](#setters-and-pass-by-value)
- [Removing methods](#removing-methods)

### Header guards

Header guards are preprocessor defines that surround every header file to prevent it from being included multiple times. OpenSim header guards should be written like this:

```cpp
#ifndef OPENSIM_PROPERTY_TABLE_H_
#define OPENSIM_PROPERTY_TABLE_H_
 // ... stuff ...
#endif // OPENSIM_PROPERTY_TABLE_H_
```
This matches the scheme used in Simbody and avoids all of the following problems. (Trailing underscore, but not leading, is allowed.)

Common problems with header guards:

1. They can interfere with user code, and
2. They violate the C++ standard.

OpenSim uses many very common class names, like `Object` and `Array`. These are likely to appear in other code libraries as well, so anyone who combines the OpenSim API with other libraries or their own code may have conflicts. A simple rule to avoid conflicts is for the header guards to be unique symbols, easily achieved by including the product name in them (that is, they should contain `OPENSIM`).

As a reminder, the C++ standard prohibits user code from using identifiers that contain double underscore (`__`) or begin with an underscore followed by a capital letter (`_P`). Those symbols are reserved for the compiler and the standard library (`std`). Use of symbols like that means it is subject to conflict with the compiler, either now or in future releases or new platforms. Hopefully those conflicts would cause compiler errors, but that is not guaranteed.

One other minor point is that preprocessor macros should typically have very ugly names with `LOTS_OF_CAPS` to make it obvious that they are not ordinary identifiers, and to avoid conflicts with NiceUpperCamelCaseIdentifiers used for class names.

### Creating new OpenSim objects

Every OpenSim object class now automatically defines a typedef “Super” that refers to the immediate parent (“superclass”) of that class. If an object has to delegate something to its parent, use “Super” rather than explicitly listing the parent class. If the parent changes due to future refactoring, your code will still compile but it will be wrong! This is not a hypothetical problem - this construct was developed because these bugs kept recurring. By using automatically defined `Super` you can completely avoid these issues in the future.

Example:

In MyNewComponent.h:

```cpp
class MyNewComponent : public SomeIntermediateClassDerivedFromObject {
 ...
 void extendBaseClassMethod() override;
}
```

In MyNewComponent.cpp:

```cpp
void MyNewComponent::extendBaseClassMethod() {
 Super::extendBaseClassMethod(); // invoke the parent’s method
 //NOT: SomeIntermediateClassDerivedFromObject::extendBaseClassMethod()

 // now do the local stuff
}
```

Now if someone changes the class structure later so that the Component's parent changes in the header file, the code in the .cpp file (which will have been long forgotten) will automatically change its behavior.

### Assignment operators in C++

You should let the compiler automatically generate the copy constructor and copy assignment operator for your classes whenever possible. But sometimes you have to write one. Here is the basic template for copy assignment:


```cpp
MyClass& operator=(const MyClass& source) {
 if (&source != this) {
   // copy stuff from source to this
 }
 return *this;
}
```

You run into problems if copy assignment operators are missing lines 2 and 4. Since the “copy stuff” part often begins by deleting the contents of “this”, a self assignment like a=a will fail without those lines; that is always supposed to work (and does for all the built-in and std library types). Of course no one intentionally does that kind of assignment, but they occur anyway in general code since you don’t always know where the source comes from.

If the “copy stuff” part consists only of assignments that work for self assignment, then you can get away without the test, but unless you’ve thought it through carefully you should just get in the habit of putting in the test.

### Documenting your code

Doxygen only looks in your .h files; it does not generate documentation from .cpp files. Thus, comments in .cpp files don't need to follow doxygen formatting, and in fact they should not because it is confusing and makes it look like there is API documentation when there isn't. You should mostly use `//`-style comments in .cpp files, and be sure you are addressing your comments to the right audience -- no doxygen reader will ever see them.

We produce two sets of doxygen-generated documentation, one for (scripting)
users and one for C++ developers. The primary difference is that the user
documentation only shows public members, while the developer documentation also
shows protected members, nested classes, etc. When writing doxygen comments,
you can use `\internal` or `\if developer ... \endif`
for documentation that is only intended for developers.

Read more about doxygen on this page: [Guide to Building Doxygen](http://simtk-confluence.stanford.edu:8080/display/OpenSim/Guide+to+Building+Doxygen)

### Each line of text should be at most 80 characters

It is common to display multiple code windows side-by-side. For the code to display nicely,
it is necessary to choose a limit on the line length.

### Replace tabs with four spaces

Please be sure that your IDE (code editor) is set to replace tabs with four spaces. You should never allow tab characters to get into your code. They will look great to you, but most other people will see your code as randomly formatted.

If you use Visual Studio, goto Tools:Options:Text Editor:C/C++:Tabs, set tab size=indent size=4, and check the "Insert spaces" button.

### Renaming classes in the OpenSim API

Sometimes it makes sense to change the name of a class in OpenSim because the name is confusing or doesn't reflect the desired function. This seemingly innocent, and usually desirable, refactoring has some side effects that API developers should be aware of so that changes do not break working functionality.

**Deserialization:** The code that reads objects from XML files keys on the String representing the class name to create corresponding objects (e.g., "PinJoint" class shows in XML as ``<PinJoint>``). If you change the name of PinJoint (e.g., to MyPinJoint) you need to make sure old models that have the tag <PinJoint> still work. Normally this is captured by test cases. If you decide to make the change, you'll have to edit the file "RegisterTypes_osimSimulation.cpp" and add the line ```Object::renameType("PinJoint", "MyPinJoint")```, so that the deserialization code knows how to handle the XML tag.

**Swig wrapping and GUI:** Most API users don't build the GUI, however they should continue to build the JavaWrap project used by the GUI to make sure changes on the C++ side do not cause serious problems downstream to either the GUI or scripts that we'll be distributing that utilize the Java wrapping. The mechanics for this procedure are as follows:
- Turn on JavaWrapping in CMake (`BULD_JAVA_WRAPPING=on`). You must have SWIG and Java installed.
- Build JavaWrap project to run SWIG (see README.md for obtaining SWIG).
- Run test case testContext, which simulates a few GUI calls.

If a class is not included in the wrapping interface file [OpenSim/Java/swig/javaWrapOpenSim.i](https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Wrapping/Java/swig/javaWrapOpenSim.i) then the class is likely not used by the GUI and so is safe to change; otherwise, please consult with GUI developers first before renaming.

### Naming conventions
Please follow the convention that property, input, and output names use the `lower_case_with_underscores` convention, while class names use `UpperCamelCaseWithoutUnderscores`. This ensures no conflicts with XML tag names and makes it easy to tell a property name from a class name.

#### Functions and methods
Names should begin with a verb and use the `lowerCamelCase` convention.

```cpp
getBodyVelocity()
setDefaultLength()
```

We have some conventional starting verbs; you should use the same ones when they apply and avoid them if your methods are doing something different.

   verb   | meaning
----------|---------
`get`     | Return a const reference to something that has already been computed.
`set`     | Change the value of some internal quantity; may cause invalidation of dependent computations.
`upd`     | (update) Return a writable reference to some internal variable. May cause invalidation of dependent computations.
`find`    | Perform a small calculation (e.g., find the distance between two points) and return the result without changing anything else.
`calc`    | (calculate) Perform an expensive calculation and return the result. Does not cause any other changes.
`realize` | Initiate state-dependent computations and cache results internally; no result returned.
`add`     | Add the object (Component) to an internal list of references. Should not take over ownership. 
`adopt`   | Take over ownership (e.g., `Set::adoptAndAppend()`).
`extend`  | A virtual method intended to extend a defining capability of a Base class; can either be pure virtual or not. The first line of the derived class implementation must be `Super::extend<DoSomething>()`. For example, a ModelComponent knows how to ``connectToModel``, but the details of how each concrete ModelComponent type does this is implemented by the derived class.
`implement` | A virtual method intended to implement a *pure* virtual function of a Base class. The derived class's implementation does *not* call any method on `Super`.
`express` | Express a vector in a different basis (i.e., without translation). Typically used as `Frame::expressVectorIn*()`.

### ``throw`` and ``return`` are not functions

In C++, ``throw`` and ``return`` are not functions. It is misleading to enclose their arguments in parentheses. That is, you should write “return x;” not “return(x);”. A parenthesized expression is not treated the same as a function argument list. For example f(a,b) and return(a,b) mean different things (the former is a 2-argument function call; the latter is an invocation of the rarely-used “comma operator”).

### Always use pre-increment and pre-decrement operators when you have a choice

Both pre-increment i and post-increment i are available. When you don’t look at the result, they are logically equivalent. For simple types they are physically equivalent too. But for complicated types (like iterators), the pre-increment is much cheaper computationally because it doesn’t require separate storage for saving the previous result. Therefore, you should get in the habit of using pre-increment in all your loops:


```cpp
/*YES*/ for (int i = 0; i < limit; ++i);

/*NO*/ for (int i = 0; i < limit; i++);
```

This will prevent you from using the wrong operator in the expensive cases, which are not always obvious.

Of course, in cases where you actually need the pre- or post-value for something, you should use the appropriate operator.

### Place pointer and reference symbols with the type

References and pointers create new types. That is “T”, “T*”, and “T&” are three distinct types. You can tell because you can make typedefs like this:

```cpp
typedef T  SameAsT;

typedef T* PointerToT;

typedef T& ReferenceToT;

// and then declare

SameAsT      t1,      t2;      // both are type T

PointerToT   tptr1,   tptr2;   // both are type T*

ReferenceToT tref1=a, tref2=b; // both are type T&
```

Therefore, you should place the ``*`` and ``&`` next to the type, not the variable, because logically they are part of the type. Unfortunately, the C language had a bug in its syntax which has been inherited by C++. A line like `char* a,b` is treated like `char* a; char b;` rather than `char* a; char* b;` but if we write `typedef char* CharPtr;` then `CharPtr a,b` declares both to be pointers. There is no perfect solution because the language is broken. However, there is no problem in argument lists (since each variable has to have its own type). We recommend that you simply avoid the misleading multiple-declaration form when using pointers or references. Just use separate declarations or a typedef. Then always put the ``*`` and ``&`` with the type where they belong. So argument lists should look like this:

```cpp
/*YES*/ f(int I, string& name, char* something);

/*NO*/  f(int I, string &name, char *something);
```

### Setters and pass-by-value

Setter functions usually make a copy of the passed-in argument, and therefore
it is often preferable to use pass-by-value and then `std::move` the argument
into a member variable (rather than to pass-by-reference).

https://stackoverflow.com/questions/270408/is-it-better-in-c-to-pass-by-value-or-pass-by-constant-reference

### Removing methods

If you are cleaning up a class and decide to remove a method, then it's necessary to remove both the prototype from the header and the implementation from the .cpp file (if any). While C++ doesn't complain, leaving the prototype in the header file with no implementation anywhere causes problems for wrapping. Swig runs only on the headers and has no way of knowing whether there's an implementation or not. Since the methods end up being exported, they then have to be resolved at compile time of the osimJavaJNI project.

List of Contributors
--------------------
Here is a partial list of contributors to the OpenSim C++ code base and Java/Python wrapping via Swig (not to the OpenSim project as a whole); please let us know if a name is missing (including yours!).

Real name          | GitHub Id    | Contributions/expertise
-------------------|--------------|-------------------------
Ayman Habib        |@aymanhab     |Applications developer; Original code base; Java wrapping
Ajay Seth          |@aseth1       |Lead developer; Component API; Musculoskeletal dynamics
Chris Dembia       |@chrisdembia  |Build system; Component API; Python wrapping; Documentation
Michael Sherman    |@sherm1       |Properties; Component API; Documentation; [Simbody](https://github.com/simbody/simbody)
Jennifer Hicks     |@jenhicks     |Project manager; Documentation
Matthew Millard    |@mjhmilla     |Muscle modeling
Tim Dorn           |@twdorn       |Probes and ProbeReporter
Peter Eastman      |@peastman     |Contact and Prescribed Forces
Kevin Xu           |@kevunix      |64bit build, Bug fixes
Thomas Uchida      |@tkuchida     |Muscle & Metabolics modeling; Bug fixes
Soha Pouya         |@sohapouya    |BodyActuator; Test cases; Bug fixes
Matt DeMers        |@msdemers     |Frames; Bug fixes
James Dunne        |@jimmyDunne   |Documentation; Examples
Chand John         |@unusualinsights|RRATool; Controller example; Bug fixes
Cassidy Kelly      |@cassidykelly |Testing; Properties; Bug fixes
Nabeel Allana      |@nallana      |ExpressionBasedCoordinateForce; McKibbenActuator
Jack Wang          |@jmwang       |Bug fixes; Linux compilation
Ian Stavness       |@stavness     |Mac-Xcode build
Weiguang Si        |@JustinSi     |Model properties; Bug fixes
Apoorva Rajagopal  |@apoorvar     |Xcode compatibility
Jason Moore        |@moorepants   |Linux build; Typo fix
Jenny Yong         |@jryong       |Actuator outputs; Typo fix
Carmichael Ong     |@carmichaelong|Typo fix
Sam Hamner         |              |PointOnLineConstraint; Controller example; Doxygen
Frank Anderson     |              |Original code base; CMC; Musculoskeletal modeling
Peter Loan         |              |Original code base; SIMM Translator; WrapObjects
Kate Saul          |              |Original MuscleAnalysis
Jack Middleton     |              |Initial Simbody integration
Jeffrey Reinbolt   |              |Static Optimization; Examples; Musculoskeletal modeling
Shrinidhi Lakshmikanth|@klshrinidhi|Data interface
Andrew LaPre       |@ankela       |IK error output to file
Neil Dhir		   |@wagglefoot   |Python API contributions; specifically example usages
Akshay Patel       |@akshaypatel1811|Python examples
Colin Smith        |@clnsmith     |Blankevoort1991Ligament
Adam Kewley        |@adamkewley   |Controller performance

Contributor License Agreement
-----------------------------
OpenSim is licensed under the permissive [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0). OpenSim users are not required to follow our noble egalitarian principles, nor to share their profits with us, nor even to acknowledge us (though they often do and it is greatly appreciated). When you make a contribution in any of the ways described above, you are agreeing to allow your contribution to be used under the same terms, adding no additional restrictions to the OpenSim project nor requirements on OpenSim users.

Specifically, by contributing you are agreeing to the following terms:

  1. The code, text, or images you submit are your original work (you own and retain the copyright) or you otherwise have the right to submit the work.
  2. You grant the OpenSim project, developers, and users a nonexclusive, irrevocable license to use your submission and any necessary intellectual property, under terms of the Apache 2.0 license.
  3. No part of your contribution is covered by a viral ("copyleft") license like GPL or LGPL.
  4. You are capable of granting these rights for the contribution.

If your contribution contains others' open source code licensed under Apache 2.0 or other non-viral license like BSD, MIT, or ZLib, it is probably fine. But be sure to mention that in the Pull Request you submit so we can discuss it.

Thank you for agreeing to these terms.

If you remain unsure, you can compile and distribute your work (e.g., a new Component) as an OpenSim plugin licensed under different terms.  We encourage OpenSim users to share their work as plugins at least until the code can be thoroughly tested and reviewed through the PR process.
