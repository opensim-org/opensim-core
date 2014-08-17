Guidelines for contributing to OpenSim-Core
===========================================

Outdated guidelines can be found [here](http://simtk-confluence.stanford.edu:8080/display/OpenSim/Contributing+to+the+OpenSim+Source+Code).

We also have [coding standards](http://simtk-confluence.stanford.edu:8080/display/OpenSim/OpenSim+Coding+Standards) that your contribution must follow.

Making a pull request
---------------------
To help the people who review your pull request, please include a detailed description of what changes or additions the pull request includes and what you have done to test and verify the changes. If there is an existing issue or set of issues that documents the problem the pull request is solving, you can reference it. But, please make sure detailed information about the commit is easily available. 


Notes
-----
If you want to check memory leaks, you can have Travis-CI run valgrind on the test cases. Just put `[ci valgrind]` in your commit message.

Thank you for contributing!
