#!/usr/bin/env python
import sys
import os
import fileinput
if (sys.version_info.major == 3 and sys.version_info.minor >= 8 and sys.platform.startswith('win')):
   if (len(sys.argv)==2):
      install_path = os.path.abspath('../../bin')
   else:
      install_path = os.path.abspath('opensim') # Conda layout on windows
   new_path = install_path.replace(os.sep, '/')
   print ('install path found as '+ new_path)
   with fileinput.FileInput('opensim/__init__.py', inplace=True, backup='.bak') as file:
     for line in file:
          print(line.replace('DLL_PATH', "'" + new_path + "'"), end='')
