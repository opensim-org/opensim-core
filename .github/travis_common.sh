# Terminate on error, print every line
set -ev

cd $TRAVIS_BUILD_DIR
# Stop build if comment contains [skip travis].
if $(git log -n1 --format="%B" | grep --quiet '\[skip travis\]'); then exit; fi 
  
cmake --version # To help debug any cmake-related issues.
  
## Ensure that there are no tabs in source code.
# GREP returns 0 (true) if there are any matches, and
# we don't want any matches. If there are matches,
# print a helpful message, and make the test fail by using "false".
# The GREP command here checks for any tab characters in the the files
# that match the specified pattern. GREP does not pick up explicit tabs
# (e.g., literally a \t in a source file).
cd $TRAVIS_BUILD_DIR
# - if grep --line-num --recursive --exclude-dir="*dependencies*" --include={CMakeLists.txt,*.cpp,*.c,*.h} -P "\t" . ; then echo "Tabs found in the lines shown above. See CONTRIBUTING.md about tabs."; false; fi

## Set up environment variables.
# Only if compiling with gcc, update environment variables to use the new
# gcc.
if [ "$CXX" = "g++" ]; then export CXX="g++-4.9" CC="gcc-4.9"; fi
if [[ "$TRAVIS_OS_NAME" = "linux" && "$CXX" = "clang++" ]]; then export CXX="clang++-3.5" CC="clang-3.5"; fi

## Temporary hack to find libblas and liblapack.
# TODO. Currently Simbody is using Travis CI's Ubuntu 14.04 VMs, which link with 
# liblapack.so.3 and libblas.so.3. These files don't exist on the 12.04 machines.
if [ "$TRAVIS_OS_NAME" = "linux" ]; then mkdir ~/lib; fi
if [ "$TRAVIS_OS_NAME" = "linux" ]; then ln -s /usr/lib/liblapack.so ~/lib/liblapack.so.3; fi
if [ "$TRAVIS_OS_NAME" = "linux" ]; then ln -s /usr/lib/libblas.so ~/lib/libblas.so.3; fi
if [ "$TRAVIS_OS_NAME" = "linux" ]; then export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/lib; fi

## Doxygen.
# Need a doxygen that is more recent than that available through apt-get.
if [[ "$DOXY" = "on" && "$TRAVIS_OS_NAME" = "linux" ]]; then mkdir ~/doxygen && cd ~/doxygen; fi
if [[ "$DOXY" = "on" && "$TRAVIS_OS_NAME" = "linux" ]]; then wget http://ftp.stack.nl/pub/users/dimitri/doxygen-1.8.10.linux.bin.tar.gz; fi
if [[ "$DOXY" = "on" && "$TRAVIS_OS_NAME" = "linux" ]]; then tar xzf doxygen-1.8.10.linux.bin.tar.gz; fi


## Install SWIG to build Java/python wrapping.
if [[ "$WRAP" = "on" ]]; then mkdir ~/swig-source && cd ~/swig-source; fi
if [[ "$WRAP" = "on" ]]; then wget https://github.com/swig/swig/archive/rel-$SWIG_VER.tar.gz; fi
if [[ "$WRAP" = "on" ]]; then tar xzf rel-$SWIG_VER.tar.gz && cd swig-rel-$SWIG_VER; fi
if [[ "$WRAP" = "on" ]]; then sh autogen.sh && ./configure --prefix=$HOME/swig --disable-ccache && make && make -j8 install; fi

# Below is a description of the process for securely uploading files to
# Sourceforge, taken from https://oncletom.io/2016/travis-ssh-deploy/.
#
# The following link is a great page to learn about SSH.
# https://www.digitalocean.com/community/tutorials/understanding-the-ssh-encryption-and-connection-process
#
# Contact chrisdembia if you need the login information for opensim-bot at
# sourceforge, to manage myosin.sourceforge.net.
#
# You must install the travis command-line tool: `gem install travis`
# Locally, from the root of the repository:
# Create a 4096-bit RSA key, providing a comment.
# $ ssh-keygen -t rsa -b 4096 -C 'opensim-bot@sourceforge.net' -f .github/.deploy_myosin_sourceforge_rsa
# When prompted for a passphrase, just hit enter (twice).
# First make a backup copy of .travis.yml.
# $ cp .travis-ci.yml travis-ci-backup.yml
# Encrypt the private key, add decryption line to .travis.yml. 
# $ travis encrypt-file .github/.deploy_myosin_sourceforge_rsa --add
# Copy the relevant parts of the added decryption line to
# .github/travis_dependencies.sh and .github/travis_buildtest.sh.
# Remove the unencrypted private key. DO NOT commmit the unencrypted private
# key.
# $ rm -f .github/.deploy_myosin_sourceforge_rsa
# Move the encrypted key to the .github folder.
# $ mv .deploy_myosin_sourceforge_rsa.enc .github/
# Manually, log into the sourceforge website (user opensim-bot) and add the
# public key (contents of .github/.deploy_myosin_sourceforge_rsa.pub) in
# Account Settings > SSH Settings.
# Now you can delete the public key file from your local machine.
# Commit the encrypted private key and the changes to .travis.yml.
# $ git commit .travis.yml .github/.deploy_myosin_sourceforge_rsa.enc

