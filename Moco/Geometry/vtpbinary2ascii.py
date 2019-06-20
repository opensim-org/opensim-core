import os
import sys
import vtk

for filename in os.listdir('.'):
    if not filename.endswith('.vtp'): continue
    with file(filename) as f:
        if 'format="appended"' in f.read():
            print(filename)
            f.close()

            reader = vtk.vtkXMLPolyDataReader()
            reader.SetFileName(filename)
            
            writer = vtk.vtkXMLPolyDataWriter()
            #writer = vtk.vtkSTLWriter()
            writer.SetFileName(filename)
            writer.SetInputConnection(reader.GetOutputPort())
            writer.SetDataModeToAscii()
            writer.Write()
