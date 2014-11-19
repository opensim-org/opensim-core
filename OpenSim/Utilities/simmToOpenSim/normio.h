/*******************************************************************************

   NORMIO.H

   Author: Peter Loan

   Date: 23-OCT-98

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.

*******************************************************************************/

FileType check_file_type(char filename[]);
ReturnCode read_polyhedron(PolyhedronStruct* ph, char filename[], SBoolean run_norm);
ReturnCode read_binary_file(PolyhedronStruct* ph, char filename[]);
ReturnCode read_ascii_file(PolyhedronStruct* ph, char filename[]);
ReturnCode read_old_ascii_file(PolyhedronStruct* ph, char filename[]);
ReturnCode read_wavefront_file(PolyhedronStruct* ph, char filename[], SBoolean run_norm);
ReturnCode read_stl_ascii_file(PolyhedronStruct* ph, char filename[], SBoolean run_norm);
ReturnCode read_stl_binary_file(PolyhedronStruct* ph, char filename[], SBoolean run_norm);
ReturnCode write_binary_file(char filename[], BoundingCube* bc,
               PolyhedronStruct polyhedron[], int num_polyhedra,
               int vertex_offset);
ReturnCode write_ascii_file(char filename[],  BoundingCube* bc,
              PolyhedronStruct polyhedron[], int num_polyhedra,
              NormOptions* opt);
ReturnCode write_old_ascii_file(char filename[], PolyhedronStruct polyhedron[],
                int num_polyhedra, NormOptions* opt);
ReturnCode write_binary_separates(char filename[], PolyhedronStruct polyhedron[],
                int num_polyhedra, int vertex_offset);
ReturnCode write_ascii_separates(char filename[], PolyhedronStruct polyhedron[],
               int num_polyhedra, NormOptions* opt);
