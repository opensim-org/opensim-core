Building Moco for Mac
=====================
First, download moco from the git repository:

- `git clone https://github.com/stanfordnmbl/opensim-moco.git`

Install the following:
- `gfortran`
- `pkgconfig`
- `autoreconf`
- `aclocal`
- `glibtoolize`
- `wget`
- `cmake`
- `doxygen` (optional)

You can install these with Homebrew:

```bash
brew install cmake pkgconfig gcc autoconf libtool automake wget doxygen
```

Navigate to the directory where Moco is installed.


ex: `cd /Users/xxxx/Documents/github/opensim-moco`

Run build_on_mac from the terminal.

ex `./build_on_mac`
