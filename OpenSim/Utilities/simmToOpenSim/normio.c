/*******************************************************************************

   NORMIO.C

   Author: Peter Loan

   Date: 23-OCT-98

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.

*******************************************************************************/
#include "universal.h"

#define _POSIX_ 1

#include <fcntl.h>
#include <io.h>

#include "globals.h"
#include "functions.h"
#include "normio.h"
#include "normtools.h"

// Linux doesn't use O_BINARY
#ifndef O_BINARY
#define O_BINARY 0
#endif


/*************** DEFINES (for this file only) *********************************/


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static int filekey = 133;
char new_ascii_label[] = "NORM_ASCII";
char stl_ascii_label[] = "solid";
static SBoolean verbose = no;


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another .c file) *************/


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void name_polyhedron(PolyhedronStruct* ph, char filename[]);


/* ---------------------------------------------------------------------------
   BINARY COMPATIBILITY ROUTINES:  the following 3 routines are used to support
     reading binary bone files on little-endian Win32 machines.  -- KMS 12/17/98
------------------------------------------------------------------------------ */
static void flip_bytes(char* buf, int size)
{
   char* p = buf;
   char* q = buf + size - 1;

   for ( ; p < q; p++, q--)
   {
      char c = *p;
      
      *p = *q;
      *q = c;
   }
}


static int read_binary(int fd, char* buf, int size)
{
   int bytes_read = read(fd, buf, size);

#ifdef WIN32
   flip_bytes(buf, size);
#endif

   return bytes_read;
}


static int read_binary_array(int fd, char* buf, int n, int size)
{
   int nBytes = n * size;
   int bytes_read = read(fd, buf, nBytes);
   
#ifdef WIN32
   {
      char *p = buf, *end = buf + nBytes;

      for ( ; p < end; p += size)
         flip_bytes(p, size);
   }
#endif

   return bytes_read;
}


static int write_binary(int fd, char* buf, int size)
{
   int bytes_written;

#ifdef WIN32
   flip_bytes(buf, size);
#endif

   bytes_written = write(fd, buf, size);

#ifdef WIN32
   flip_bytes(buf, size);
#endif

   return bytes_written;
}


static int write_binary_array(int fd, char* buf, int n, int size)
{
   int bytes_written, nBytes = n * size;

#ifdef WIN32
   char *p = buf, *end = buf + nBytes;

   for ( ; p < end; p += size)
      flip_bytes(p, size);
#endif

   bytes_written = write(fd, buf, nBytes);

#ifdef WIN32
   p = buf;
   
   for ( ; p < end; p += size)
      flip_bytes(p, size);
#endif

   return bytes_written;
}


/* PARSE_STRING: this routine scans a string and extracts a variable from
 * it. The type of variable that it extracts is specified in one of the
 * arguments. It returns the unused portion of the string so that more
 * variables can be extracted from it.
 */

static char* parse_string(char str_buffer[], VariableType var_type, void* dest_var)
{
   char tmp_buffer[CHARBUFFER], *buffer_ptr;

   if (str_buffer == NULL)
      return NULL;

   buffer_ptr = tmp_buffer;

   while (CHAR_IS_WHITE_SPACE(*str_buffer))
      str_buffer++;

   if (var_type == type_char)
   {
      *((char*)dest_var) = *str_buffer;
      return str_buffer + 1;
   }

   if (var_type == type_string)
   {
      if (STRING_IS_NULL(str_buffer))
         *((char*)dest_var) = STRING_TERMINATOR;
      else
         (void)sscanf(str_buffer,"%s", (char*)dest_var);
      return str_buffer + strlen((char*)dest_var);
   }

   if (var_type == type_double)
   {
      *((double*)dest_var) = ERROR_DOUBLE;
      if (STRING_IS_NOT_NULL(str_buffer))
      {
         while (*str_buffer == '-' || *str_buffer == '.' || *str_buffer == '+' ||
        *str_buffer == 'e' || *str_buffer == 'E' ||
        *str_buffer == 'd' || *str_buffer == 'D' ||
        (*str_buffer >= '0' && *str_buffer <= '9'))
            *(buffer_ptr++) = *(str_buffer++);
         *buffer_ptr = STRING_TERMINATOR;
         if (sscanf(tmp_buffer,"%lg",(double*)dest_var) != 1)
            *((double*)dest_var) = ERROR_DOUBLE;
      }
      return str_buffer;
   }

   if (var_type == type_int)
   {
      *((int*)dest_var) = ERRORINT;
      if (STRING_IS_NOT_NULL(str_buffer))
      {
         while (*str_buffer == '-' || (*str_buffer >= '0' && *str_buffer <= '9'))
            *(buffer_ptr++) = *(str_buffer++);
         *buffer_ptr = STRING_TERMINATOR;
         if (sscanf(tmp_buffer,"%d",(int*)dest_var) != 1)
        *((int*)dest_var) = ERRORINT;
      }
      return str_buffer;
   }

   (void)sprintf(errorbuffer,"Unknown variable type (%d) passed to parse_string().",
       (int)var_type);
   error(none,errorbuffer);

   return NULL;
}

FileType check_file_type(char filename[])
{
   int fpb, fkb, len;
   char fkey[CHARBUFFER];
   FILE* fpa;
   FileType ft=unknown;

   // First try to open the file.
   if ((fpb = simm_open(filename, O_RDONLY | O_BINARY | O_RAW)) == -1)
      return file_not_found;
   else
      close(fpb);

    len = strlen(filename);

   // Check suffix to see if it's a Wavefront .obj file.
    if (len > 4)
    {
        if (STRINGS_ARE_EQUAL(&filename[len-4], ".obj"))
            return wavefront;
    }

   // Binary STL files should not start with "solid" in the header,
   // but some do anyway. To load these, their suffixes must be
   // changed to .stlb.
    if (len > 5)
    {
        if (STRINGS_ARE_EQUAL(&filename[len-5], ".stlb"))
         return stl_binary;
   }

    // Check suffix to see if it's an STL file.
   if (len > 4)
    {
        if (STRINGS_ARE_EQUAL(&filename[len-4], ".stl"))
      {
         if ((fpa = simm_fopen(filename, "r")) == NULL)
         {
            return file_not_found;
         }
         else
         {
            fscanf(fpa, "%s", fkey);
            if (STRINGS_ARE_EQUAL(fkey, stl_ascii_label))
               ft = stl_ascii;
            else
               ft = stl_binary;
            fclose(fpa);
            return ft;
         }
      }
    }

   // The file exists and it's not a Wavefront or STL, so
   // read from it to see if it's a binary SIMM bone file.
   if ((fpb = simm_open(filename, O_RDONLY | O_BINARY | O_RAW)) == -1)
   {
      return file_not_found;
   }
   else
   {
      read_binary(fpb, (char*)&fkb, sizeof(int));
      close(fpb);
   }

   if (fkb == filekey)
       ft = binary;
   else
       ft = unknown;

   if (ft == unknown)
   {
      if ((fpa = simm_fopen(filename, "r")) == NULL)
      {
          return file_not_found;
      }
      else
      {
          fscanf(fpa, "%s", fkey);
          if (STRINGS_ARE_EQUAL(fkey, new_ascii_label))
             ft = new_ascii;
          else
             ft = old_ascii;
          fclose(fpa);
      }
   }

   return ft;
}


FileReturnCode read_polyhedron(PolyhedronStruct* ph, char filename[], SBoolean run_norm)
{
   FileType input_file_type;
   ReturnCode rc;

   preread_init_polyhedron(ph);

    // Always name the polyhedron so it can be written back to JNT file.
   name_polyhedron(ph, filename);

   input_file_type = check_file_type(filename);

   if (input_file_type == file_not_found)
   {
      return file_missing;
   }
   if (input_file_type == binary)
   {
      rc = read_binary_file(ph,filename);
      if (rc == code_bad || check_polyhedron(ph) == code_bad)
      {
          sprintf(errorbuffer, "Validity check of bone %s failed.", ph->name);
          error(none,errorbuffer);
          return file_bad;
      }
      postread_init_polyhedron(ph,no);
   }
   else if (input_file_type == new_ascii)
   {
      rc = read_ascii_file(ph,filename);
      if (rc == code_bad || check_polyhedron(ph) == code_bad)
      {
          sprintf(errorbuffer, "Validity check of bone %s failed.", ph->name);
          error(none,errorbuffer);
          return file_bad;
      }
      postread_init_polyhedron(ph,no);
   }
    else if (input_file_type == wavefront)
    {
      rc = read_wavefront_file(ph, filename, run_norm);
      if (rc == code_bad || check_polyhedron(ph) == code_bad)
      {
          sprintf(errorbuffer, "Validity check of bone %s failed.", ph->name);
          error(none,errorbuffer);
          return file_bad;
      }
    }
    else if (input_file_type == stl_ascii)
    {
      rc = read_stl_ascii_file(ph, filename, run_norm);
      if (rc == code_bad || check_polyhedron(ph) == code_bad)
      {
          sprintf(errorbuffer, "Validity check of bone %s failed.", ph->name);
          error(none,errorbuffer);
          return file_bad;
      }
    }
    else if (input_file_type == stl_binary)
    {
      rc = read_stl_binary_file(ph, filename, run_norm);
      if (rc == code_bad || check_polyhedron(ph) == code_bad)
      {
          sprintf(errorbuffer, "Validity check of bone %s failed.", ph->name);
          error(none,errorbuffer);
          return file_bad;
      }
    }
   else /* input_file_type == old_ascii or unknown */
   {
      rc = read_old_ascii_file(ph,filename);
      if (rc == code_bad || check_polyhedron(ph) == code_bad)
      {
          sprintf(errorbuffer, "Validity check of bone %s failed.", ph->name);
          error(none,errorbuffer);
          return file_bad;
      }
      postread_init_polyhedron(ph,no);
      if (run_norm == yes)
      {
         int num_out;
         NormOptions opt;
         PolyhedronStruct* ph_out;

         opt.verbose_output = no;
         opt.clip_vertices = no;
         opt.output_format = new_ascii;
         opt.write_separate_polyhedra = no;
         opt.convexify_polygons = no;
         opt.remove_collinear = no;
         opt.vertex_offset = 0;
         opt.tol_box_set = no;
         opt.fill_holes = no;
         opt.triangulate = no_tri;
         opt.vertex_order = unspecified_order;
         opt.vertex_tolerance = 0.0;
         opt.max_edge_length = -50.0;
         opt.reference_normal[0] = 0.0;
         opt.reference_normal[1] = 1.0;
         opt.reference_normal[2] = 0.0;
         norm(ph, &opt, &num_out, &ph_out);
         if (num_out == 0)
            return file_bad;
         free_polyhedron(ph, no, NULL);
         copy_polyhedron(&ph_out[0], ph);
         free_polyhedron(&ph_out[0], no, NULL);
      }
   }

   return file_good;
}

/* Make a suitable name for a polyhedron, based on the name of
 * the file it was loaded from.
 */
static void name_polyhedron(PolyhedronStruct* ph, char filename[])
{
   // Form the bone name from the base of the filename.
   int len, end = strlen(filename);
   int start = end - 1;
   while (start >= 0 && filename[start--] != DIR_SEP_CHAR)
      ;

   if (start == -1)
      start = 0;
   else
      start += 2;
   len = end - start;
   ph->name = (char *)simm_malloc((len+1)*sizeof(char));

   (void)strncpy(ph->name,&filename[start],len);
   ph->name[len] = STRING_TERMINATOR;
}

ReturnCode read_binary_file(PolyhedronStruct* ph, char filename[])
{
   int i, fd, fk, not_needed, num_edges;
   long correct_num_bytes, bytes_read = 0;
   float fnormal[3];

   if ((fd = simm_open(filename,O_RDONLY | O_BINARY | O_RAW)) == -1)
   {
      (void)sprintf(errorbuffer,"Unable to open file %s", filename);
      error(none,errorbuffer);
      return code_bad;
   }

   bytes_read += read_binary(fd,(char*)&fk,sizeof(int));
   if (fk != filekey)
   {
      (void)sprintf(errorbuffer,"File %s is not a properly formatted binary file", filename);
      error(none,errorbuffer);
      return code_bad;
   }

   bytes_read += read_binary(fd,(char*)&ph->num_vertices,sizeof(int));
   bytes_read += read_binary(fd,(char*)&ph->num_polygons,sizeof(int));
   bytes_read += read_binary(fd,(char*)&not_needed,sizeof(int));
   bytes_read += read_binary(fd,(char*)&not_needed,sizeof(int));
   bytes_read += read_binary(fd,(char*)&ph->bc.x1,sizeof(double));
   bytes_read += read_binary(fd,(char*)&ph->bc.x2,sizeof(double));
   bytes_read += read_binary(fd,(char*)&ph->bc.y1,sizeof(double));
   bytes_read += read_binary(fd,(char*)&ph->bc.y2,sizeof(double));
   bytes_read += read_binary(fd,(char*)&ph->bc.z1,sizeof(double));
   bytes_read += read_binary(fd,(char*)&ph->bc.z2,sizeof(double));

   ph->vertex = (VertexStruct*)simm_malloc(ph->num_vertices*sizeof(VertexStruct));
   ph->polygon = (PolygonStruct*)simm_malloc(ph->num_polygons*sizeof(PolygonStruct));

   if (ph->vertex == NULL || ph->polygon == NULL)
      return code_bad;

   for (i=0; i<ph->num_vertices; i++)
   {
      preread_init_vertex(&ph->vertex[i],i);
      bytes_read += read_binary_array(fd, (char*) ph->vertex[i].coord,
                                      3, sizeof(double));
   }

   for (i=0; i<ph->num_vertices; i++)
   {
      bytes_read += read_binary_array(fd, (char*) fnormal, 3, sizeof(float));
      ph->vertex[i].normal[0] = fnormal[0];
      ph->vertex[i].normal[1] = fnormal[1];
      ph->vertex[i].normal[2] = fnormal[2];
   }

   for (i=0, num_edges=0; i<ph->num_polygons; i++)
   {
      preread_init_polygon(&ph->polygon[i]);
      bytes_read += read_binary(fd, (char*) &ph->polygon[i].num_vertices, sizeof(int));
      num_edges += ph->polygon[i].num_vertices;
      ph->polygon[i].vertex_index = (int*)simm_malloc(ph->polygon[i].num_vertices*
                              sizeof(int));
   }

   for (i=0; i<ph->num_polygons; i++)
   {
      bytes_read += read_binary_array(fd, (char*) ph->polygon[i].vertex_index,
                                      ph->polygon[i].num_vertices, sizeof(int));
   }
   close(fd);

   correct_num_bytes = 5*sizeof(int) + 6*sizeof(double) +
      3*ph->num_vertices*sizeof(double) + 3*ph->num_vertices*sizeof(float) +
     ph->num_polygons*sizeof(int) + num_edges*sizeof(int);

   if (bytes_read != correct_num_bytes)
   {
      ph->num_vertices = ph->num_polygons = 0;
      (void)sprintf(errorbuffer,"Error reading file %s. Only %ld of %ld bytes read.",
            filename, bytes_read, correct_num_bytes);
      error(none,errorbuffer);
      return code_bad;
   }
   return code_fine;
}


ReturnCode read_ascii_file(PolyhedronStruct* ph, char filename[])
{
   int i, j, rc;
   FILE* fp;

   fp = simm_fopen(filename,"r");
   if (fp == NULL)
   {
#if 0 /* this should never get called */
      (void)sprintf(errorbuffer,"Unable to open file %s", filename);
      error(none,errorbuffer);
#endif
      return code_bad;
   }

   /* For now, support old ASCII file type. Eventually (perhaps for SIMM/PC 1.0,
    * we'll want to remove support for this filetype (as well as for binary),
    * since the new norm will only output the new ASCII format.
    */
   rc = fscanf(fp,"%s", buffer);
   if (rc != 1 || STRINGS_ARE_NOT_EQUAL(buffer,new_ascii_label))
   {
      fclose(fp);
      (void)sprintf(errorbuffer,"Error reading header information from file %s.",
            filename);
      error(none,errorbuffer);
      return code_bad;
   }

   rc = fscanf(fp,"%d %d", &ph->num_vertices, &ph->num_polygons);

   if (rc != 2)
   {
      fclose(fp);
      (void)sprintf(errorbuffer,"Error reading header information from file %s.",
            filename);
      error(none,errorbuffer);
      return code_bad;
   }

   rc = fscanf(fp,"%lf %lf %lf %lf %lf %lf",
           &ph->bc.x1, &ph->bc.x2,
           &ph->bc.y1, &ph->bc.y2,
           &ph->bc.z1, &ph->bc.z2);

   if (rc != 6)
   {
      fclose(fp);
      (void)sprintf(errorbuffer,"Error reading bounding box from file %s.",
            filename);
      error(none,errorbuffer);
      return code_bad;
   }

   ph->vertex = (VertexStruct*)simm_malloc(ph->num_vertices*sizeof(VertexStruct));
   ph->polygon = (PolygonStruct*)simm_malloc(ph->num_polygons*sizeof(PolygonStruct));

   if (ph->vertex == NULL || ph->polygon == NULL)
   {
      fclose(fp);
      return code_bad;
   }

   /* read-in the vertices and vertex normals */

   for (i=0; i<ph->num_vertices; i++)
   {
      preread_init_vertex(&ph->vertex[i],i);
      fscanf(fp,"%lf %lf %lf", &ph->vertex[i].coord[0],
         &ph->vertex[i].coord[1], &ph->vertex[i].coord[2]);
      fscanf(fp,"%lf %lf %lf", &ph->vertex[i].normal[0],
         &ph->vertex[i].normal[1], &ph->vertex[i].normal[2]);
   }

   /* read-in the polygons (vertex indices) */

   for (i=0; i<ph->num_polygons; i++)
   {
      preread_init_polygon(&ph->polygon[i]);
      fscanf(fp,"%d", &ph->polygon[i].num_vertices);
      ph->polygon[i].vertex_index = (int*)simm_malloc(ph->polygon[i].num_vertices*
                              sizeof(int));
      for (j=0; j<ph->polygon[i].num_vertices; j++)
          fscanf(fp,"%d", &ph->polygon[i].vertex_index[j]);
   }

   fclose(fp);
   
   return code_fine;
}


ReturnCode read_old_ascii_file(PolyhedronStruct* ph, char filename[])
{
   int i, j, rc;
   FILE* fp;

   fp = simm_fopen(filename,"r");
   if (fp == NULL)
   {
      (void)sprintf(errorbuffer,"Unable to open file %s", filename);
      error(none,errorbuffer);
      return code_bad;
   }

   rc = fscanf(fp,"%d %d", &ph->num_vertices, &ph->num_polygons);

   if (rc != 2)
   {
      fclose(fp);
      (void)sprintf(errorbuffer,"Error reading header information from file %s.",
            filename);
      error(none,errorbuffer);
      return code_bad;
   }

   ph->vertex = (VertexStruct*)simm_malloc(ph->num_vertices*sizeof(VertexStruct));
   ph->polygon = (PolygonStruct*)simm_malloc(ph->num_polygons*sizeof(PolygonStruct));
   //dkb polygon malloc boolcode.seg_list here???

   if (ph->vertex == NULL || ph->polygon == NULL)
   {
      fclose(fp);
      return code_bad;
   }

   /* read-in the vertices */

   for (i=0; i<ph->num_vertices; i++)
   {
      preread_init_vertex(&ph->vertex[i],i);
      fscanf(fp,"%lf %lf %lf", &ph->vertex[i].coord[0],
         &ph->vertex[i].coord[1], &ph->vertex[i].coord[2]);
   }

   /* read-in the polygons (vertex indices) */

   for (i=0; i<ph->num_polygons; i++)
   {
      preread_init_polygon(&ph->polygon[i]);
      fscanf(fp,"%d", &ph->polygon[i].num_vertices);
      ph->polygon[i].vertex_index = (int*)simm_malloc(ph->polygon[i].num_vertices*
                              sizeof(int));
      for (j=0; j<ph->polygon[i].num_vertices; j++)
      {
          fscanf(fp,"%d", &ph->polygon[i].vertex_index[j]);
          /* Old ASCII vertex numbers start at 1 */
          ph->polygon[i].vertex_index[j]--;
      }
   }

   fclose(fp);
   
   return code_fine;
}

ReturnCode read_wavefront_file(PolyhedronStruct* ph, char filename[], SBoolean run_norm)
{
   int i, num_normals = 0, v_count = 0, n_count = 0, p_count = 0;
   double* vertex_normals;
   FILE* fp;

   fp = simm_fopen(filename, "r");
   if (fp == NULL)
   {
#if 0 /* this should never get called */
      (void)sprintf(errorbuffer,"Unable to open file %s", filename);
      error(none,errorbuffer);
#endif
      return code_bad;
   }

    /* Scan ahead to find num_vertices, num_normals, num_polygons. */
    while (fscanf(fp, "%s", buffer) > 0)
    {
        if (STRINGS_ARE_EQUAL(buffer, "v"))
            ph->num_vertices++;
        else if (STRINGS_ARE_EQUAL(buffer, "vn"))
            num_normals++;
        else if (STRINGS_ARE_EQUAL(buffer, "f"))
            ph->num_polygons++;
        else if (STRINGS_ARE_EQUAL(buffer, "#"))
            read_line(fp, buffer);
    }

   // Make a temporary array for the vertex normals, which will
   // be copied to the vertex structures as the polygons
   // are read. In a Wavefront file it is possible to use a vertex
   // in one polygon with one normal, and the same vertex in a
   // different polygon with a different normal. This is not allowed
   // in SIMM; each time a vertex is used in a polygon, the normal
   // specified for that usage will be copied into the vertex's
   // normal vector, overwriting any previous specifications.
   vertex_normals = (double*)simm_malloc(num_normals * 3 * sizeof(double));

#if 0
    if (num_normals != ph->num_vertices)
    {
      fclose(fp);
        (void)sprintf(errorbuffer, "Error: number of vertices (%d) not equal to number of vertex normals (%d) in file %s.",
            ph->num_vertices, num_normals, filename);
      error(none,errorbuffer);
      ph->num_vertices = ph->num_polygons = 0;
      return code_bad;
    }
#endif

   ph->vertex = (VertexStruct*)simm_malloc(ph->num_vertices * sizeof(VertexStruct));
   ph->polygon = (PolygonStruct*)simm_malloc(ph->num_polygons * sizeof(PolygonStruct));

   if (ph->vertex == NULL || ph->polygon == NULL || vertex_normals == NULL)
   {
      fclose(fp);
      ph->num_vertices = ph->num_polygons = 0;
      return code_bad;
   }

   for (i=0; i<ph->num_vertices; i++)
      preread_init_vertex(&ph->vertex[i], i);

   for (i=0; i<ph->num_polygons; i++)
      preread_init_polygon(&ph->polygon[i]);

    /* Rewind file to beginning. */
    rewind(fp);

    while (fscanf(fp, "%s", buffer) > 0)
    {
        if (STRINGS_ARE_EQUAL(buffer, "#"))
      {
            read_line(fp, buffer);
      }
        else if (STRINGS_ARE_EQUAL(buffer, "v"))
        {
            fscanf(fp, "%lg %lg %lg", &ph->vertex[v_count].coord[0],
                &ph->vertex[v_count].coord[1], &ph->vertex[v_count].coord[2]);
            v_count++;
        }
        else if (STRINGS_ARE_EQUAL(buffer, "vn"))
        {
            fscanf(fp, "%lg %lg %lg", &vertex_normals[n_count * 3], &vertex_normals[n_count * 3 + 1], &vertex_normals[n_count * 3 + 2]);
            n_count++;
        }
        else if (STRINGS_ARE_EQUAL(buffer, "f"))
        {
            int max_vertices, index, pv_count = 0;
            ReturnCode rc;
            char* line;

            read_line(fp, buffer);
            max_vertices = strlen(buffer);
            ph->polygon[p_count].vertex_index = (int*)simm_malloc(max_vertices * sizeof(int));
            line = buffer;
            // indices are stored in this format: 102/95/101
            // vertex coordinates: 102nd element in 'v' array
         // texture coordinates: 95th element in 'vt' array
         // normal coordinates: 101st element in 'vn' array
         // negative numbers mean to start at the end of the array and count backwards
            while (line && strlen(line) > 0)
            {
                // Read the vertex index.
            int v_index;
                line = parse_string(line, type_int, &v_index);
                if (v_index < 0)
                    v_index += ph->num_vertices; // count backwards from end
                else
                    v_index--; // make index zero-based
            ph->polygon[p_count].vertex_index[pv_count++] = v_index;

                // After each vertex index there can be normal and texture indices (each preceded by "/").
                // If they are present, skip over the texture index but use the normal index. TODO5.0: use the texure index
                if (line[0] == '/')
                {
                    // skip over the "/"
                    line++;
                    // read and ignore the texture index
                    line = parse_string(line, type_int, &index);
                    if (line[0] == '/')
                    {
                        // skip over the "/"
                        line++;
                        // read the normal index
                        line = parse_string(line, type_int, &index);
                  index--; // make index zero-based
                  // Now get that normal vector and copy it to the vertex.
                  memcpy(ph->vertex[v_index].normal, &vertex_normals[index * 3], 3 * sizeof(double));
                    }
                }
            }
            ph->polygon[p_count].num_vertices = pv_count;
            ph->polygon[p_count].vertex_index = (int*)simm_realloc(ph->polygon[p_count].vertex_index,
                ph->polygon[p_count].num_vertices * sizeof(int), &rc);
            p_count++;
        }
    }

   fclose(fp);

   postread_init_polyhedron(ph, no);

   if (run_norm == yes)
   {
      int num_out;
      NormOptions opt;
      PolyhedronStruct* ph_out;

      opt.verbose_output = no;
      opt.clip_vertices = no;
      opt.output_format = new_ascii;
      opt.write_separate_polyhedra = no;
      opt.convexify_polygons = no;
      opt.remove_collinear = no;
      opt.vertex_offset = 0;
      opt.tol_box_set = no;
      opt.fill_holes = no;
      opt.triangulate = no_tri;
      opt.vertex_order = unspecified_order;
      opt.vertex_tolerance = 0.0000002;
      opt.max_edge_length = -50.0;
      opt.reference_normal[0] = 0.0;
      opt.reference_normal[1] = 1.0;
      opt.reference_normal[2] = 0.0;

      //write_ascii_file("C:\\Products\\5.0Testing\\bones\\debug1.asc", &ph->bc, ph, 1, &opt);

      norm(ph, &opt, &num_out, &ph_out);
      if (num_out == 0)
         return file_bad;
      free_polyhedron(ph, no, NULL);
      copy_polyhedron(&ph_out[0], ph);
      free_polyhedron(&ph_out[0], no, NULL);

      //write_ascii_file("C:\\Products\\5.0Testing\\bones\\debug2.asc", &ph->bc, ph, 1, &opt);
   }

   return code_fine;
}


ReturnCode read_stl_binary_file(PolyhedronStruct* ph, char filename[], SBoolean run_norm)
{
   int i, j, fd, bytes_read = 0, index;
   float number;
   unsigned int count;
   char header[80];

   if ((fd = simm_open(filename, O_RDONLY | O_BINARY | O_RAW)) == -1)
   {
#if 0 // This should never get called.
      (void)sprintf(errorbuffer, "Unable to open file %s", filename);
      error(none, errorbuffer);
#endif
      return code_bad;
   }

   bytes_read += read(fd, (void*)header, 80);

   bytes_read += read(fd, (void*)&count, sizeof(int));
   ph->num_polygons = count;
   ph->num_vertices = 3 * ph->num_polygons;

   ph->vertex = (VertexStruct*)simm_malloc(ph->num_vertices * sizeof(VertexStruct));
   ph->polygon = (PolygonStruct*)simm_malloc(ph->num_polygons * sizeof(PolygonStruct));

   if (ph->vertex == NULL || ph->polygon == NULL)
   {
      close(fd);
      ph->num_vertices = ph->num_polygons = 0;
      return code_bad;
   }

   for (i=0; i<ph->num_vertices; i++)
      preread_init_vertex(&ph->vertex[i], i);

   for (i=0; i<ph->num_polygons; i++)
      preread_init_polygon(&ph->polygon[i]);

   for (i=0, index=0; i<ph->num_polygons; i++)
   {
      // read the polygon normal
      bytes_read += read(fd, (void*)&number, sizeof(float));
      ph->polygon[i].normal[0] = number;
      bytes_read += read(fd, (void*)&number, sizeof(float));
      ph->polygon[i].normal[1] = number;
      bytes_read += read(fd, (void*)&number, sizeof(float));
      ph->polygon[i].normal[2] = number;
      // read the 3 vertices
      for (j=0; j<3; j++)
      {
         bytes_read += read(fd, (void*)&number, sizeof(float));
         ph->vertex[index].coord[0] = number;
         bytes_read += read(fd, (void*)&number, sizeof(float));
         ph->vertex[index].coord[1] = number;
         bytes_read += read(fd, (void*)&number, sizeof(float));
         ph->vertex[index].coord[2] = number;
         index++;
      }
      bytes_read += read(fd, (void*)header, 2); // attribute byte count, not used
   }

   close(fd);

   for (i=0, index=0; i<ph->num_polygons; i++)
   {
      ph->polygon[i].num_vertices = 3;
      ph->polygon[i].vertex_index = (int*)simm_malloc(ph->polygon[i].num_vertices * sizeof(int));
      ph->polygon[i].vertex_index[0] = index++;
      ph->polygon[i].vertex_index[1] = index++;
      ph->polygon[i].vertex_index[2] = index++;
   }

   postread_init_polyhedron(ph, no);

   if (run_norm == yes)
   {
      int num_out;
      NormOptions opt;
      PolyhedronStruct* ph_out;

      opt.verbose_output = no;
      opt.clip_vertices = no;
      opt.output_format = new_ascii;
      opt.write_separate_polyhedra = no;
      opt.convexify_polygons = no;
      opt.remove_collinear = no;
      opt.vertex_offset = 0;
      opt.tol_box_set = no;
      opt.fill_holes = no;
      opt.triangulate = no_tri;
      opt.vertex_order = unspecified_order;
      opt.vertex_tolerance = 0.0000002;
      opt.max_edge_length = -50.0;
      opt.reference_normal[0] = 0.0;
      opt.reference_normal[1] = 1.0;
      opt.reference_normal[2] = 0.0;

      //write_ascii_file("C:\\Products\\5.0Testing\\bones\\debug1.asc", &ph->bc, ph, 1, &opt);

      norm(ph, &opt, &num_out, &ph_out);
      if (num_out == 0)
         return file_bad;
      free_polyhedron(ph, no, NULL);
      copy_polyhedron(&ph_out[0], ph);
      free_polyhedron(&ph_out[0], no, NULL);

      //write_ascii_file("C:\\Products\\5.0Testing\\bones\\debug2.asc", &ph->bc, ph, 1, &opt);
   }

   return code_fine;
}


ReturnCode read_stl_ascii_file(PolyhedronStruct* ph, char filename[], SBoolean run_norm)
{
   int i, j, index, count = 0;
   FILE* fp;
   float number;
   char header[80];

   fp = simm_fopen(filename, "r");
   if (fp == NULL)
   {
#if 0 /* this should never get called */
      (void)sprintf(errorbuffer, "Unable to open file %s", filename);
      error(none, errorbuffer);
#endif
      return code_bad;
   }

   fscanf(fp, "%s", header);
   if (STRINGS_ARE_NOT_EQUAL_CI(header, "solid"))
   {
      (void)sprintf(errorbuffer, "Error reading ascii STL file %s. File must start with header \"solid\".", filename);
      error(none, errorbuffer);
      return code_bad;
   }

   while (1)
   {
      if (fscanf(fp, "%s", header) != 1)
         break;
      if (STRINGS_ARE_EQUAL_CI(header, "facet"))
         count++;
   }
   rewind(fp);

   ph->num_polygons = count;
   ph->num_vertices = 3 * ph->num_polygons;

   ph->vertex = (VertexStruct*)simm_malloc(ph->num_vertices * sizeof(VertexStruct));
   ph->polygon = (PolygonStruct*)simm_malloc(ph->num_polygons * sizeof(PolygonStruct));

   if (ph->vertex == NULL || ph->polygon == NULL)
   {
      fclose(fp);
      ph->num_vertices = ph->num_polygons = 0;
      return code_bad;
   }

   for (i=0; i<ph->num_vertices; i++)
      preread_init_vertex(&ph->vertex[i], i);

   for (i=0; i<ph->num_polygons; i++)
      preread_init_polygon(&ph->polygon[i]);

   fscanf(fp, "%s", header);

   for (i=0, index=0; i<ph->num_polygons; i++)
   {
      // read the polygon normal
      fscanf(fp, "%*s %*s %lg %lg %lg", &ph->polygon[i].normal[0], &ph->polygon[i].normal[1], &ph->polygon[i].normal[2]);
      fscanf(fp, "%*s %*s"); // "outer loop"
      // read the 3 vertices
      for (j=0; j<3; j++)
      {
         fscanf(fp, "%*s %lg %lg %lg", &ph->vertex[index].coord[0], &ph->vertex[index].coord[1], &ph->vertex[index].coord[2]);
         index++;
      }
      fscanf(fp, "%*s %*s"); // "endloop endfacet"
   }

   for (i=0, index=0; i<ph->num_polygons; i++)
   {
      ph->polygon[i].num_vertices = 3;
      ph->polygon[i].vertex_index = (int*)simm_malloc(ph->polygon[i].num_vertices * sizeof(int));
      ph->polygon[i].vertex_index[0] = index++;
      ph->polygon[i].vertex_index[1] = index++;
      ph->polygon[i].vertex_index[2] = index++;
   }

   fclose(fp);

   postread_init_polyhedron(ph, no);

   if (run_norm == yes)
   {
      int num_out;
      NormOptions opt;
      PolyhedronStruct* ph_out;

      opt.verbose_output = no;
      opt.clip_vertices = no;
      opt.output_format = new_ascii;
      opt.write_separate_polyhedra = no;
      opt.convexify_polygons = no;
      opt.remove_collinear = no;
      opt.vertex_offset = 0;
      opt.tol_box_set = no;
      opt.fill_holes = no;
      opt.triangulate = no_tri;
      opt.vertex_order = unspecified_order;
      opt.vertex_tolerance = 0.0000002;
      opt.max_edge_length = -50.0;
      opt.reference_normal[0] = 0.0;
      opt.reference_normal[1] = 1.0;
      opt.reference_normal[2] = 0.0;

      //write_ascii_file("C:\\Products\\5.0Testing\\bones\\debug1.asc", &ph->bc, ph, 1, &opt);

      norm(ph, &opt, &num_out, &ph_out);
      if (num_out == 0)
         return file_bad;
      free_polyhedron(ph, no, NULL);
      copy_polyhedron(&ph_out[0], ph);
      free_polyhedron(&ph_out[0], no, NULL);

      //write_ascii_file("C:\\Products\\5.0Testing\\bones\\debug2.asc", &ph->bc, ph, 1, &opt);
   }

   return code_fine;
}


ReturnCode write_binary_file(char filename[], BoundingCube* bc,
               PolyhedronStruct polyhedron[], int num_polyhedra,
               int vertex_offset)
{

   int i, j, k, fd, vnum, num_vertices, num_polygons, num_edges;
   int num_written, max_vertex_count;
   float fnorm[3];
   SBoolean write_verbose = yes;

   if ((fd = simm_open(filename,O_CREAT | O_WRONLY | O_BINARY)) == -1)
   {
      fprintf(stderr,"Unable to open %s for output.\n", filename);
      return code_bad;
   }

   if (write_verbose == yes)
   {
      printf("Writing binary file %s ... ", filename);
      fflush(stdout);
   }

   num_vertices = 0;
   num_polygons = 0;
   num_edges = 0;
   max_vertex_count = 0;
   for (i=0; i<num_polyhedra; i++)
   {
      num_vertices += (polyhedron[i].num_vertices - polyhedron[i].num_removed_vertices);
      num_polygons += (polyhedron[i].num_polygons - polyhedron[i].num_removed_polygons);
      for (j=0; j<polyhedron[i].num_polygons; j++)
      {
          if (polyhedron[i].polygon[j].thrown_out == no)
          {
             num_edges += polyhedron[i].polygon[j].num_vertices;
             if (polyhedron[i].polygon[j].num_vertices > max_vertex_count)
                max_vertex_count = polyhedron[i].polygon[j].num_vertices;
          }
      }
   }

   /* Filekey is used to mark the file as a SIMM/norm binary file */

   write_binary(fd,(char*)&filekey,sizeof(int));
   write_binary(fd,(char*)&num_vertices,sizeof(int));
   write_binary(fd,(char*)&num_polygons,sizeof(int));
   write_binary(fd,(char*)&num_edges,sizeof(int));
   write_binary(fd,(char*)&max_vertex_count,sizeof(int));

   write_binary(fd,(char*)&bc->x1,sizeof(double));
   write_binary(fd,(char*)&bc->x2,sizeof(double));
   write_binary(fd,(char*)&bc->y1,sizeof(double));
   write_binary(fd,(char*)&bc->y2,sizeof(double));
   write_binary(fd,(char*)&bc->z1,sizeof(double));
   write_binary(fd,(char*)&bc->z2,sizeof(double));

   num_written = vertex_offset;
   for (i=0; i<num_polyhedra; i++)
   {
      for (j=0; j<polyhedron[i].num_vertices; j++)
      {
          if (polyhedron[i].vertex[j].thrown_out == no)
          {
             write_binary_array(fd,(char*)polyhedron[i].vertex[j].coord,3,sizeof(double));
          }
      }
   }

   for (i=0; i<num_polyhedra; i++)
   {
      for (j=0; j<polyhedron[i].num_vertices; j++)
      {
          if (polyhedron[i].vertex[j].thrown_out == no)
          {
             fnorm[0] = (float)polyhedron[i].vertex[j].normal[0];
             fnorm[1] = (float)polyhedron[i].vertex[j].normal[1];
             fnorm[2] = (float)polyhedron[i].vertex[j].normal[2];
             write_binary_array(fd,(char*)fnorm,3,sizeof(float));
          }
      }
   }

   for (i=0; i<num_polyhedra; i++)
      for (j=0; j<polyhedron[i].num_polygons; j++)
          if (polyhedron[i].polygon[j].thrown_out == no)
             write_binary(fd,(char*)&polyhedron[i].polygon[j].num_vertices,sizeof(int));

   for (i=0; i<num_polyhedra; i++)
   {
      for (j=0; j<polyhedron[i].num_polygons; j++)
      {
          if (polyhedron[i].polygon[j].thrown_out == no)
          {
             for (k=0; k<polyhedron[i].polygon[j].num_vertices; k++)
             {
                vnum = polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index;
                write_binary(fd,(char*)&vnum,sizeof(int));
             }
          }
      }
   }

   close(fd);

#ifdef WIN32
   _chmod(filename, _S_IREAD | _S_IWRITE );
#else
   chmod(filename,00644);
#endif

   if (write_verbose == yes)
      printf("Done.\n");

   return code_fine;

}


ReturnCode write_ascii_file(char filename[],  BoundingCube* bc,
              PolyhedronStruct polyhedron[], int num_polyhedra,
              NormOptions* opt)
{

   int i, j, k, num_vertices, num_polygons, num_written;
   SBoolean write_verbose = yes;
   FILE* fpo;

   if (polyhedron == NULL)
      return code_fine;

   if ((fpo = simm_fopen(filename,"w")) == NULL)
   {
      fprintf(stderr,"Unable to open %s for output.\n", filename);
      return code_bad;
   }

   if (write_verbose == yes)
   {
      printf("Writing ascii file %s ... ", filename);
      fflush(stdout);
   }

   fprintf(fpo, "%s\n", new_ascii_label);

   num_vertices = 0;
   num_polygons = 0;
   for (i=0; i<num_polyhedra; i++)
   {
      num_vertices += (polyhedron[i].num_vertices - polyhedron[i].num_removed_vertices);
      num_polygons += (polyhedron[i].num_polygons - polyhedron[i].num_removed_polygons);
   }

   fprintf(fpo,"%d %d\n", num_vertices, num_polygons);

   fprintf(fpo,"%lf %lf %lf %lf %lf %lf\n", bc->x1, bc->x2, bc->y1, bc->y2,
       bc->z1, bc->z2);

   num_written = opt->vertex_offset;
   for (i=0; i<num_polyhedra; i++)
   {
      for (j=0; j<polyhedron[i].num_vertices; j++)
      {
          if (polyhedron[i].vertex[j].thrown_out == no)
          {
             fprintf(fpo,"%12.7lf %12.7lf %12.7lf %12.7lf %12.7lf %12.7lf\n",
                 polyhedron[i].vertex[j].coord[0],
                 polyhedron[i].vertex[j].coord[1],
                 polyhedron[i].vertex[j].coord[2],
                 polyhedron[i].vertex[j].normal[0],
                 polyhedron[i].vertex[j].normal[1],
                 polyhedron[i].vertex[j].normal[2]);
             polyhedron[i].vertex[j].new_index = num_written++;
          }
      }
   }

   for (i=0; i<num_polyhedra; i++)
   {
      for (j=0; j<polyhedron[i].num_polygons; j++)
      {
          if (polyhedron[i].polygon[j].thrown_out == no)
          {
             fprintf(fpo,"%d ", polyhedron[i].polygon[j].num_vertices);
             for (k=0; k<polyhedron[i].polygon[j].num_vertices; k++)
             {
                fprintf(fpo,"%d ",
                    polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index);
                if (polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index >= num_written)
                    printf("writing %d\n", polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index);
             }
             fprintf(fpo,"\n");
          }
      }
   }

   fclose(fpo);

#ifdef WIN32
   _chmod(filename, _S_IREAD | _S_IWRITE );
#else
   chmod(filename,00644);
#endif

   if (write_verbose == yes)
      printf("Done.\n");

   return code_fine;

}



ReturnCode write_old_ascii_file(char filename[], PolyhedronStruct polyhedron[],
                int num_polyhedra, NormOptions* opt)
{

   int i, j, k, num_vertices, num_polygons, num_written;
   SBoolean write_verbose = yes;
   FILE* fpo;

   if (polyhedron == NULL)
      return code_fine;

   if ((fpo = simm_fopen(filename,"w")) == NULL)
   {
      fprintf(stderr,"Unable to open %s for output.\n", filename);
      return code_bad;
   }

   if (write_verbose == yes)
   {
      printf("Writing old ascii file %s ... ", filename);
      fflush(stdout);
   }

   num_vertices = 0;
   num_polygons = 0;
   for (i=0; i<num_polyhedra; i++)
   {
      num_vertices += (polyhedron[i].num_vertices - polyhedron[i].num_removed_vertices);
      num_polygons += (polyhedron[i].num_polygons - polyhedron[i].num_removed_polygons);
   }

   fprintf(fpo,"%d %d\n", num_vertices, num_polygons);

   num_written = opt->vertex_offset;
   for (i=0; i<num_polyhedra; i++)
   {
      for (j=0; j<polyhedron[i].num_vertices; j++)
      {
          if (polyhedron[i].vertex[j].thrown_out == no)
          {
             fprintf(fpo,"%12.7lf %12.7lf %12.7lf\n",
              polyhedron[i].vertex[j].coord[0],
              polyhedron[i].vertex[j].coord[1],
              polyhedron[i].vertex[j].coord[2]);
             polyhedron[i].vertex[j].new_index = num_written++;
          }
      }
   }

   for (i=0; i<num_polyhedra; i++)
   {
      for (j=0; j<polyhedron[i].num_polygons; j++)
      {
          if (polyhedron[i].polygon[j].thrown_out == no)
          {
             fprintf(fpo,"%d ", polyhedron[i].polygon[j].num_vertices);
             for (k=0; k<polyhedron[i].polygon[j].num_vertices; k++)
             {
                fprintf(fpo,"%d ",
                    polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index+1);
                if (polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index >= num_written)
                    printf("writing %d\n", polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index);
             }
             fprintf(fpo,"\n");
          }
      }
   }

   fclose(fpo);

#ifdef WIN32
   _chmod(filename, _S_IREAD | _S_IWRITE );
#else
   chmod(filename,00644);
#endif

   if (write_verbose == yes)
      printf("Done.\n");

   return code_fine;

}



ReturnCode write_binary_separates(char filename[], PolyhedronStruct polyhedron[],
                int num_polyhedra, int vertex_offset)
{
   int i, j, k, fd, vnum, num_edges, max_vertex_count;
   int num_vertices, num_polygons, num_written;
   SBoolean write_verbose = yes;
   float fnorm[3];
   char buffer[CHARBUFFER];
   BoundingCube bc;

   for (i=0; i<num_polyhedra; i++)
   {
      sprintf(buffer,"%s%d", filename, i+1);
      if (write_verbose == yes)
      {
          printf("Writing binary file %s ... ", buffer);
          fflush(stdout);
      }

      if ((fd = simm_open(buffer,O_CREAT | O_WRONLY | O_BINARY)) == -1)
      {
          fprintf(stderr,"Unable to open %s for output.\n", buffer);
          return code_bad;
      }

      num_edges = 0;
      max_vertex_count = 0;
      for (j=0; j<polyhedron[i].num_polygons; j++)
      {
          if (polyhedron[i].polygon[j].thrown_out == no)
          {
              num_edges += polyhedron[i].polygon[j].num_vertices;
             if (polyhedron[i].polygon[j].num_vertices > max_vertex_count)
                max_vertex_count = polyhedron[i].polygon[j].num_vertices;
          }
      }

      bc.x1 = bc.x2 = polyhedron[0].vertex[0].coord[0];
      bc.y1 = bc.y2 = polyhedron[0].vertex[0].coord[1];
      bc.z1 = bc.z2 = polyhedron[0].vertex[0].coord[2];

      for (j=0; j<polyhedron[i].num_vertices; j++)
      {
          if (polyhedron[i].vertex[j].thrown_out == yes)
             continue;
          if (polyhedron[i].vertex[j].coord[0] < bc.x1)
            bc.x1 = polyhedron[i].vertex[j].coord[0];
          if (polyhedron[i].vertex[j].coord[0] > bc.x2)
            bc.x2 = polyhedron[i].vertex[j].coord[0];
          if (polyhedron[i].vertex[j].coord[1] < bc.y1)
            bc.y1 = polyhedron[i].vertex[j].coord[1];
          if (polyhedron[i].vertex[j].coord[1] > bc.y2)
            bc.y2 = polyhedron[i].vertex[j].coord[1];
          if (polyhedron[i].vertex[j].coord[2] < bc.z1)
            bc.z1 = polyhedron[i].vertex[j].coord[2];
          if (polyhedron[i].vertex[j].coord[2] > bc.z2)
              bc.z2 = polyhedron[i].vertex[j].coord[2];
      }

      /* Filekey is used to mark the file as a SIMM/norm binary file */

      write(fd,(char*)&filekey,sizeof(int));

      num_vertices = polyhedron[i].num_vertices - polyhedron[i].num_removed_vertices;
      num_polygons = polyhedron[i].num_polygons - polyhedron[i].num_removed_polygons;

      write_binary(fd,(char*)&num_vertices,sizeof(int));
      write_binary(fd,(char*)&num_polygons,sizeof(int));
      write_binary(fd,(char*)&num_edges,sizeof(int));
      write_binary(fd,(char*)&max_vertex_count,sizeof(int));
      write_binary(fd,(char*)&bc.x1,sizeof(double));
      write_binary(fd,(char*)&bc.x2,sizeof(double));
      write_binary(fd,(char*)&bc.y1,sizeof(double));
      write_binary(fd,(char*)&bc.y2,sizeof(double));
      write_binary(fd,(char*)&bc.z1,sizeof(double));
      write_binary(fd,(char*)&bc.z2,sizeof(double));

      for (j=0, num_written=0; j<polyhedron[i].num_vertices; j++)
      {
          if (polyhedron[i].vertex[j].thrown_out == no)
          {
             polyhedron[i].vertex[j].new_index = num_written++;
             write_binary_array(fd,(char*)polyhedron[i].vertex[j].coord,3,sizeof(double));
          }
      }

      for (j=0; j<polyhedron[i].num_vertices; j++)
      {
          if (polyhedron[i].vertex[j].thrown_out == no)
          {
             fnorm[0] = (float)polyhedron[i].vertex[j].normal[0];
             fnorm[1] = (float)polyhedron[i].vertex[j].normal[1];
             fnorm[2] = (float)polyhedron[i].vertex[j].normal[2];
             write_binary_array(fd,(char*)fnorm,3,sizeof(float));
          }
      }

      for (j=0; j<polyhedron[i].num_polygons; j++)
          if (polyhedron[i].polygon[j].thrown_out == no)
             write_binary(fd,(char*)&polyhedron[i].polygon[j].num_vertices,sizeof(int));

      for (j=0; j<polyhedron[i].num_polygons; j++)
      {
          if (polyhedron[i].polygon[j].thrown_out == no)
          {
             for (k=0; k<polyhedron[i].polygon[j].num_vertices; k++)
             {
                vnum = polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index + vertex_offset;
                write_binary(fd,(char*)&vnum,sizeof(int));
             }
          }
      }

      close(fd);

#ifdef WIN32
      _chmod(buffer, _S_IREAD | _S_IWRITE );
#else
      chmod(buffer,00644);
#endif

      if (write_verbose == yes)
          printf("Done.\n");
   }

   return code_fine;

}




ReturnCode write_ascii_separates(char filename[], PolyhedronStruct polyhedron[],
               int num_polyhedra, NormOptions* opt)
{

   int i, j, k, num_written, num_edges, max_vertex_count;
   SBoolean write_verbose = yes;
   FILE* fpo;
   char buffer[CHARBUFFER];

   for (i=0; i<num_polyhedra; i++)
   {
      sprintf(buffer,"%s%d", filename, i+1);
      if (write_verbose == yes)
      {
          printf("Writing ascii file %s ... ", buffer);
          fflush(stdout);
      }

      if ((fpo = simm_fopen(buffer,"w")) == NULL)
      {
          fprintf(stderr,"Unable to open %s for output.\n", buffer);
          return code_bad;
      }

      fprintf(fpo,"%s\n", new_ascii_label);

      num_edges = 0;
      max_vertex_count = 0;
      for (j=0; j<polyhedron[i].num_polygons; j++)
      {
          if (polyhedron[i].polygon[j].thrown_out == no)
          {
             num_edges += polyhedron[i].polygon[j].num_vertices;
             if (polyhedron[i].polygon[j].num_vertices > max_vertex_count)
                max_vertex_count = polyhedron[i].polygon[j].num_vertices;
          }
      }

      fprintf(fpo,"%d %d\n", polyhedron[i].num_vertices - polyhedron[i].num_removed_vertices,
          polyhedron[i].num_polygons - polyhedron[i].num_removed_polygons);

      fprintf(fpo,"%lf %lf %lf %lf %lf %lf\n", polyhedron[i].bc.x1, polyhedron[i].bc.x2,
          polyhedron[i].bc.y1, polyhedron[i].bc.y2,
          polyhedron[i].bc.z1, polyhedron[i].bc.z2);

      for (j=0, num_written=0; j<polyhedron[i].num_vertices; j++)
      {
          if (polyhedron[i].vertex[j].thrown_out == yes)
             continue;
          polyhedron[i].vertex[j].new_index = num_written++;
          fprintf(fpo,"%10.6lf %10.6lf %10.6lf %10.6lf %10.6lf %10.6lf\n",
              polyhedron[i].vertex[j].coord[XX],
              polyhedron[i].vertex[j].coord[YY],
              polyhedron[i].vertex[j].coord[ZZ],
              polyhedron[i].vertex[j].normal[XX],
              polyhedron[i].vertex[j].normal[YY],
              polyhedron[i].vertex[j].normal[ZZ]);
      }

      for (j=0; j<polyhedron[i].num_polygons; j++)
      {
          if (polyhedron[i].polygon[j].thrown_out == yes)
             continue;
          fprintf(fpo,"%d ", polyhedron[i].polygon[j].num_vertices);
          for (k=0; k<polyhedron[i].polygon[j].num_vertices; k++)
            fprintf(fpo,"%d ", polyhedron[i].vertex[polyhedron[i].polygon[j].vertex_index[k]].new_index +
                 opt->vertex_offset);
          fprintf(fpo,"\n");
      }
      fclose(fpo);

#ifdef WIN32
      _chmod(buffer, _S_IREAD | _S_IWRITE );
#else
      chmod(buffer,00644);
#endif

      if (write_verbose == yes)
          printf("Done.\n");
   }

   return code_fine;

}

#if ! ENGINE
/* -------------------------------------------------------------------------
   build_file_list_from_pattern - given a file pathname with optional wildcard
      characters, this routine returns an argv-style list of file pathnames
      that match the specified pattern.

   NOTE: This routine can append to an existing list of file names.  If *list
      is non-NULL, then this routine assumes it already points to a previously
      allocated argv-style list, and appends to it.  If *list is NULL, then
      this routine allocates the list and it is up to the caller to free it.
---------------------------------------------------------------------------- */
void build_file_list_from_pattern (const char* pattern, char*** list, int* numFiles)
{
   int n;
   
   char* fileList = glutExpandFilenamePattern(pattern, &n);
   
   if (fileList)
   {
      if (n > 0)
      {
         ReturnCode rc;
         
         int i = *numFiles;
         
         *numFiles += n;
         
         if (list)
         {
            if (*list)
               *list = simm_realloc(*list, *numFiles * sizeof(char*), &rc);
            else
               *list = simm_malloc(*numFiles * sizeof(char*));
            
            if (*list)
            {
               char* p = fileList;
               
               /* build list of pointers to strings:
                */
               for ( ; i < *numFiles; i++)
               {
                  mstrcpy(&(*list)[i], p);
                  
                  p += strlen(p) + 1;
               }
            }
         }
      }
      free(fileList);
   }
}
#endif /* ENGINE */

SBoolean file_exists(const char filename[])
{
   FILE* fp;

   if (filename == NULL || strlen(filename) == 0)
      return no;

   if ((fp = simm_fopen(filename, "r")) == NULL)
      return no;

   fclose(fp);

   return yes;
}

SBoolean can_create_file(const char filename[])
{
   FILE* fp;

   if (filename == NULL || strlen(filename) == 0)
      return no;

   // If the file exists, see if it can be appended (overwritten).
   if (file_exists(filename) == yes)
   {
      if ((fp = fopen(filename, "a")) == NULL)
         return 0;

      fclose(fp);
      return 1;
   }
   else
   {
      // The file doesn't exist. See if it can be created.
      if ((fp = fopen(filename, "w")) == NULL)
         return no;

      fclose(fp);
      remove(filename);
      return yes;
   }
}
