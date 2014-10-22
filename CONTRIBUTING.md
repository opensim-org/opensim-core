Guidelines for Contributing to OpenSim-Core
===========================================

We encourage everyone to contribute to the OpenSim-Core repository. This could be making a pull request, as described below, or letting us know about a bug or issue. We've set up the following guidelines for making contributions through a pull request. The purpose of our contribution policy is to ensure that all code in OpenSim-Core has undergone real scrutiny, thereby reducing the likelihood of errors. 

Even great programmers benefit from additional eyeballs on their code to catch bugs, avoid duplication and inefficiency, watch for standards violations, and to check that the code is as clear to others as it is to its author. We appreciate contributions and our development team is collaborative and constructive -- don't be shy!


Making a Pull Request
---------------------
1. To help the people who review your pull request, you must include a detailed description of what changes or additions the pull request includes and what you have done to test and verify the changes. If there is an existing issue or set of issues that documents the problem the pull request is solving, you can reference it. But, please make sure detailed information about the commits is easily available.

2. Make sure that your request conforms to OpenSim's [coding standards](http://simtk-confluence.stanford.edu:8080/display/OpenSim/OpenSim+Coding+Standards).

3. Make sure that tests pass on your local machine before making a pull request. The [README.md](https://github.com/opensim-org/opensim-core) mentions how to run the tests.

4. Typo fixes can be merged by any member of the Dev Team.

5. Updates to comments. Doxygen, compiler compatibility, or CMake files must be reviewed by at least one member of the Dev Team before being merged. The original author or the reviewer(s) may merge the pull request.

6. Any other changes to the code require review by at least two members of the Dev Team. If the pull request involves adding a new class or performing a major object/algorithm refactor, one of these reviewers must be an Owner. The Owners and Dev Team are Teams within the opensim-org GitHub organization. The original author may NOT merge the pull request. 

A few additional practices will help streamline the code review process. Please use tags (i.e. @user_name) and quoting to help keep the discussion organized. Please also call for a meeting or Skype call when discussions start to stagnate. In addition, we recommend getting input on your interface design before implementing a major new component or other change.

Thank you for contributing!


Checking for Memory Leaks through GitHub
----------------------------------------
If you want to check memory leaks, you can have Travis-CI run valgrind on the test cases. Just put `[ci valgrind]` in your commit message.


Contributors
------------
Here is a partial list of contributors; please let us know if you know of a missing name.

- Frank C. Anderson
- Allison S. Arnold
- Scott L. Delp
- Matt S. DeMers
- Tim Dorn
- Brian Garner
- Saryn R. Goldberg
- Eran Guendelman
- Ayman Habib
- Samuel R. Hamner
- Jennifer L. Hicks
- Katherine R. S. Holzbaur
- Chand T. John
- Cassidy Kelly
- May Q. Liu
- Peter Loan
- Jack Middleton
- Matthew Millard
- Paul C. Mitiguy
- Soha Pouya
- Jeffrey A. Reinbolt
- Ajay Seth
- Michael A. Sherman
- Justin Si
- Darryl G. Thelen
- Kevin Xu
