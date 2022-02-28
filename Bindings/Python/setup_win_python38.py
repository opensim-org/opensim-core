#!/usr/bin/env python
import sys
import os
import fileinput
if (sys.version_info.major == 3 and sys.version_info.minor >= 8 and sys.platform.startswith('win')):
   if (len(sys.argv)==1):
      install_path = os.path.abspath('../../bin')
      new_path = install_path.replace(os.sep, '/')
      print ('install path found as '+ new_path)
   else:
      new_path = 'os.path.dirname(os.path.realpath(__file__))' # Conda layout on windows
   with fileinput.FileInput('opensim/__init__.py', inplace=True, backup='.bak') as file:
     for line in file:
          print(line.replace('DLL_PATH', new_path), end='')
