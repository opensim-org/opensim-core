Contributing
============

This document describes coding conventions for Moco.

Setters and pass-by-value
-------------------------

Setter functions usually make a copy of the passed-in argument, and therefore
it is often preferable to use pass-by-value and then `std::move` the argument
into a member variable (rather than to pass-by-reference).

https://stackoverflow.com/questions/270408/is-it-better-in-c-to-pass-by-value-or-pass-by-constant-reference
