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

#ifdef WIN32
   #ifdef __MWERKS__
      #include <unistd.h>
      #include <stat.h>
   #else
      #include <sys/stat.h>
      #include <io.h>
   #endif
   #include <time.h>
#else
   #include <values.h>
   #include <sys/stat.h>
   #include <sys/time.h>
#endif

#include "globals.h"
#include "functions.h"
#include "normio.h"
#include "normtools.h"


/*************** DEFINES (for this file only) *********************************/


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static int filekey = 133;
char new_ascii_label[] = "NORM_ASCII";
static SBoolean verbose = no;


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another .c file) *************/


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/


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

	/* Check suffix to see if it's a Wavefront (.obj) file. */
	len = strlen(filename);
	if (len > 4)
	{
		if (STRINGS_ARE_EQUAL(&filename[len-4], ".obj"))
			return wavefront;
	}

#ifdef WIN32
   if ((fpb = simm_open(filename,O_RDONLY | O_BINARY | O_RAW)) == -1)
#else
   if ((fpb = simm_open(filename,O_RDONLY)) == -1)
#endif
   {
      return file_not_found;
   }
   else
   {
      read_binary(fpb,(char*)&fkb,sizeof(int));
      if (fkb == filekey)
	      ft = binary;
      else
	      ft = unknown;
      close(fpb);
   }

   if (ft == unknown)
   {
      if ((fpa = simm_fopen(filename,"r")) == NULL)
      {
	      return file_not_found;
      }
      else
      {
	      fscanf(fpa,"%s", fkey);
	      if (STRINGS_ARE_EQUAL(fkey,new_ascii_label))
	         ft = new_ascii;
	      else
	         ft = old_ascii;
	      fclose(fpa);
      }
   }

   return ft;

}


ReturnCode read_polyhedron(PolyhedronStruct* ph, char filename[], SBoolean run_norm)
{

   int start, end, len, num_out;
   FileType input_file_type;
   NormOptions opt;
   PolyhedronStruct* ph_out;
   ReturnCode code;

   preread_init_polyhedron(ph);

   /* Form the bone name from the base of the filename */
   end = strlen(filename);
   start = end - 1;
   while (start >= 0 && filename[start--] != DIR_SEP_CHAR)
      ;

   start += 2;
   len = end - start;
   ph->name = (char *)simm_malloc((len+1)*sizeof(char));
   if (ph->name == NULL)
      return code_bad;

   (void)strncpy(ph->name,&filename[start],len);
   ph->name[len] = STRING_TERMINATOR;

   input_file_type = check_file_type(filename);

   if (input_file_type == file_not_found)
   {
      return code_bad;
   }
   if (input_file_type == binary)
   {
      code = read_binary_file(ph,filename);
      if (check_polyhedron(ph) == code_bad)
      {
	      sprintf(errorbuffer,"Validity check of bone %s failed.", ph->name);
	      error(none,errorbuffer);
	      return code_bad;
      }
      postread_init_polyhedron(ph,no);
   }
   else if (input_file_type == new_ascii)
   {
      code = read_ascii_file(ph,filename);
      if (check_polyhedron(ph) == code_bad)
      {
	      sprintf(errorbuffer,"Validity check of bone %s failed.", ph->name);
	      error(none,errorbuffer);
	      return code_bad;
      }
      postread_init_polyhedron(ph,no);
   }
	else if (input_file_type == wavefront)
	{
      code = read_wavefront_file(ph,filename);
      if (check_polyhedron(ph) == code_bad)
      {
	      sprintf(errorbuffer,"Validity check of bone %s failed.", ph->name);
	      error(none,errorbuffer);
	      return code_bad;
      }
      postread_init_polyhedron(ph,no);
	}
   else /* input_file_type == old_ascii or unknown */
   {
      code = read_old_ascii_file(ph,filename);
      if (check_polyhedron(ph) == code_bad)
      {
	      sprintf(errorbuffer,"Validity check of bone %s failed.", ph->name);
	      error(none,errorbuffer);
	      return code_bad;
      }
      postread_init_polyhedron(ph,no);
      if (run_norm == yes)
      {
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
         norm(ph,&opt,&num_out,&ph_out);
         if (num_out == 0)
            return code_bad;
         free_polyhedron(ph, no, NULL);
         copy_polyhedron(&ph_out[0],ph);
      }
   }

   return code;

}



ReturnCode read_binary_file(PolyhedronStruct* ph, char filename[])
{

   int i, fd, fk, not_needed, num_edges;
   long correct_num_bytes, bytes_read = 0;
   float fnormal[3];

#ifdef WIN32
   if ((fd = simm_open(filename,O_RDONLY | O_BINARY | O_RAW)) == -1)
#else
   if ((fd = simm_open(filename,O_RDONLY)) == -1)
#endif
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
      if (ph->polygon[i].boolcode.num_inters != 0)
	      printf("Hey, polygon[%d] has %d num_inters\n", i,
		ph->polygon[i].boolcode.num_inters);
   }

   fclose(fp);
   
   return code_fine;
}

ReturnCode read_wavefront_file(PolyhedronStruct* ph, char filename[])
{
   int num_normals = 0, v_count = 0, n_count = 0, p_count = 0;
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
			read_line(&fp, buffer);
	}

	if (num_normals != ph->num_vertices)
	{
      fclose(fp);
		(void)sprintf(errorbuffer,"Error: number of vertices (%d) not equal to number of vertex normals (%d) in file %s.",
			ph->num_vertices, num_normals, filename);
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

	/* Rewind file to beginning. */
	rewind(fp);

	while (fscanf(fp, "%s", buffer) > 0)
	{
		if (STRINGS_ARE_EQUAL(buffer, "v"))
		{
			preread_init_vertex(&ph->vertex[v_count], v_count);
			fscanf(fp, "%lg %lg %lg", &ph->vertex[v_count].coord[0],
				&ph->vertex[v_count].coord[1], &ph->vertex[v_count].coord[2]);
			v_count++;
		}
		else if (STRINGS_ARE_EQUAL(buffer, "vn"))
		{
			fscanf(fp, "%lg %lg %lg", &ph->vertex[n_count].normal[0],
				&ph->vertex[n_count].normal[1], &ph->vertex[n_count].normal[2]);
			n_count++;
		}
		else if (STRINGS_ARE_EQUAL(buffer, "f"))
		{
			int max_vertices, junk, pv_count = 0;
			ReturnCode rc;
			char* line;

			preread_init_polygon(&ph->polygon[p_count]);
			read_line(&fp, buffer);
			max_vertices = strlen(buffer);
			ph->polygon[p_count].vertex_index = (int*)simm_malloc(max_vertices * sizeof(int));
			line = buffer;
			// indices are stored in this format: -106//-106
			// for this example the index that Norm should store is 105
			while (line && strlen(line) > 0)
			{
				// Read the vertex index. Negative values mean to reference from the end of the vertex list.
				line = parse_string(line, type_int, &ph->polygon[p_count].vertex_index[pv_count]);
				if (ph->polygon[p_count].vertex_index[pv_count] < 0)
					ph->polygon[p_count].vertex_index[pv_count] = ph->num_vertices + ph->polygon[p_count].vertex_index[pv_count];
				else
					ph->polygon[p_count].vertex_index[pv_count]--;
				pv_count++;
				// After each vertex index there can be normal and texture indices (each preceded by "//").
				// If they are present, skip over them.
				if (line[0] == '/')
				{
					// skip over the "//"
					line += 2;
					// read and ignore the index
					line = parse_string(line, type_int, &junk);
					if (line[0] == '/')
					{
						// skip over the "//"
						line += 2;
						// read and ignore the index
						line = parse_string(line, type_int, &junk);
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

#ifndef ENGINE
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

/* FILE_EXISTS: There's probably a better way to check to see if a
 * file exists, but this way works well enough.
 */

SBoolean file_exists(char filename[])
{
   FILE* fp;

   if ((fp = simm_fopen(filename, "r")) == NULL)
      return no;

   fclose(fp);

   return yes;
}
