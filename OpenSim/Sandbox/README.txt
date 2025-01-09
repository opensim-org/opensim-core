This folder holds example code that shows what we *want* to be able to do
with OpenSim in the future. The folder contains a spectrum of ideas, from
far-out explorations to nearly-complete prototypes.

This folder can include C++ (.cpp, .h) files, Python (.py) files, MATLAB (.m)
files, etc. The folder can also contain Markdown (.md) files if it's easier to
express an idea with a mixture of text and pseudocode; the Markdown files
render nicely on the GitHub website.

There is an executable target for every
C++ file whose name begins with `future`, but these executables are not
built by default (not part of the "all" target). These executables are *not*
tests.

If you have pseudocode that you know does *not* compile, you should put it in a
file named as `pseudo*.cpp`.

The pocket guide (OpenSimPocketGuide.pdf) is a Google Doc that anyone can edit
at
https://docs.google.com/document/d/1OAKAXQTxUw02zohSkaIUzOol6qmlE2t_Idv8DJPfaLQ/edit.
If you make substantial edits to the Google Doc, please download it as a PDF
and update the copy in the repository (with a pull request).
