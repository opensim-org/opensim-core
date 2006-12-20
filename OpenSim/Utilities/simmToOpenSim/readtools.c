/*******************************************************************************

   READTOOLS.C

   Author: Peter Loan

   Date: 8-DEC-88

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains tools for reading ascii text from
      input files, and for extracting numbers and strings from the text.

   Routines:
      preprocess_file : runs cpp on a file, then opens it
      read_string    : reads a string from a file
      read_nonempty_line : reads a non-empty line of characters from a file
      read_line      : reads a possibly empty line from a file
      get_groups     : reads a set of muscle group names from muscle input file
      getcoords      : reads muscle points from the muscle input file
      getdoublearray : reads an array of x-y pairs from a file
      check_for_closing_paren :
      get_xy_pair_from_string : reads x-y pair, format: ( 0.00 , 0.00 )
      getintpair     : reads a pair of integers from a file, format: ( 0 , 0 )
      parse_string   : parses a string using the specified format (%s, %f, ...)
      get_stringpair : reads a pair of text strings from a file
      name_is_gencoord : checks to see if a name is a gencoord name

*******************************************************************************/

#include <ctype.h>

#include "universal.h"

#include <assert.h>
#include <stdarg.h>

#ifdef __MWERKS__
  #include <stat.h>
#else
  #include <sys/stat.h>
#endif

#include "globals.h"
#include "functions.h"
#include "defunctions.h"


/*************** DEFINES (for this file only) *********************************/


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/


/*************** GLOBAL VARIABLES (used in only a few files) ******************/
char natural_cubic_text[] = "_spl";
char gcv_text[] = "_gcv";
char linear_text[] = "_lin";

/*************** EXTERNED VARIABLES (declared in another file) ****************/


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static SBoolean string_contains_closing_paren(char str_buffer[]);
static char* get_xypair_from_string(char str_buffer[], double* x, double* y);
static char* parse_string(char str_buffer[], VariableType var_type,
			  void* dest_var);
static void acpp(char in_file[], const char out_file[]);




/* PREPROCESS_FILE: this routine runs the C preprocessor on the specified
 * input file, putting the output in the specified output file. It then
 * opens the processed file and returns a pointer to a FILE structure for it.
 */

FILE* preprocess_file(char infile[], const char outfile[])
{

   FILE* fp;

   if (file_exists(infile) == no)
      return NULL;

   acpp(infile, outfile);

#ifndef WIN32
   /* Change the protections so that anyone can overwrite this temporary file.
    * Technically this is not needed, because the file is deleted right after
    * it is closed, but I'm leaving it here anyway in case the file is not
    * deleted properly.
    */
   chmod(outfile, 438);
#endif

   fp = simm_fopen(outfile, "r");
   
   if (fp == NULL)
      perror("Unable to open acpp output");
   
   return fp;
}



/* READ_STRING: this routine reads a character string from a file. It skips over
 * any white space at the beginning, and returns EOF if it hits the end of
 * the file before completing the string.
 */

int read_string(FILE** fp, char str_buffer[])
{

   int c;

   /* Scan thru white space until you find the first character in the string */

   while (1)
   {
      c = getc(*fp);
      if (c == EOF)
         return EOF;
      *str_buffer = c;
      if (CHAR_IS_NOT_WHITE_SPACE(*str_buffer))
         break;
   }
   str_buffer++;

   /* Now read characters until you find white space or EOF */

   while (1)
   {
      c = getc(*fp);
      if (c == EOF)
         return EOF;
      *str_buffer = c;
      if (CHAR_IS_WHITE_SPACE(*str_buffer))
         break;
      str_buffer++;
   }

   /* Null-terminate the string */

   *str_buffer = STRING_TERMINATOR;

   /* You found a valid string without hitting EOF, so return a value that you
    * know will never equal EOF, no matter how EOF is defined.
    */

   return EOF + 1;

}



/* READ_LINE: reads a line (possibly empty) from a file */

int read_line(FILE** fp, char str_buffer[])
{
   int c;

   /* Read characters until you hit a carriage return or EOF */
   while (1)
   {
      c = getc(*fp);
      if (c == EOF)
         return EOF;
      *str_buffer = c;
      if (c == LINE_FEED)
         break;
      if (*str_buffer == CARRIAGE_RETURN)
         break;
      str_buffer++;
   }

   /* Null-terminate the string */
   *str_buffer = STRING_TERMINATOR;

   /* You found a valid line without hitting EOF, so return a value that you
    * know will never equal EOF, no matter how EOF is defined.
    */
   return EOF + 1;
}

/* COUNTTOKENS: returns the number of tokens in the string. */
int countTokens(char str[])
{
   int count = 0, previousWasSeparator = 1;
   char* p = str;

   while (*p != STRING_TERMINATOR)
   {
      if (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')
      {
         previousWasSeparator = 1;
      }
      else
      {
         if (previousWasSeparator)
            count++;
         previousWasSeparator = 0;
      }
      p++;
   }

   return count;
}


/* READ_NONEMPTY_LINE: Reads the first non-empty line from a file. */

int read_nonempty_line(FILE** fp, char str_buffer[])
{

   int c;

   /* Scan thru the white space until you find the first character */

   while (1)
   {
      c = getc(*fp);
      if (c == EOF)
         return (EOF);
      *str_buffer = c;
      if (*str_buffer != SPACE && *str_buffer != TAB &&
         *str_buffer != CARRIAGE_RETURN && *str_buffer != '\r' && c != LINE_FEED)
         break;
   }
   str_buffer++;

   /* Now read characters until you find a carriage return or EOF */

   while (1)
   {
      c = getc(*fp);
      if (c == EOF)
         break;
      *str_buffer = c;
      if (*str_buffer == CARRIAGE_RETURN || c == LINE_FEED)
         break;
      str_buffer++;
   }

   *str_buffer = STRING_TERMINATOR;

   /* You found a valid line without hitting EOF, so return a value that you
    * know will never equal EOF, no matter how EOF is defined.
    */

   return (EOF+1);

}



/* READ_MUSCLE_GROUPS: This routine reads names of muscle groups from a file
 * until the string "endgroups" is found. For each name it reads, it enters the
 * name into the group-name database for the model.  It then stores the
 * database-index of this name in an array of muscle-group indices for this
 * muscle. When done, this routine returns the array of group indices.
 */

int* read_muscle_groups(int mod, FILE** fp, int* num_groups, int muscle_number)
{

   int num_malloced_so_far = 50;
   int* group_indices;
   ReturnCode rc;

   *num_groups = 0;

   group_indices = (int*)simm_malloc(num_malloced_so_far*sizeof(int));
   if (group_indices == NULL)
      return (NULL);

   while (1)
   {
      if (fscanf(*fp,"%s",buffer) != 1)
         return (NULL);

      if (STRINGS_ARE_EQUAL(buffer,"endgroups"))
      {
	 /* rc should be code_fine since you're reallocing a smaller size */
	 group_indices = (int*)simm_realloc(group_indices,(*num_groups)*sizeof(int),&rc);
         return (group_indices);
      }

      if (STRINGS_ARE_EQUAL(buffer,"all"))
      {
	 error(none,"You cannot use a muscle group name of \"all.\"");
	 error(none,"SIMM automatically creates a muscle group containing all muscles.");
      }

      if (*num_groups > num_malloced_so_far)
      {
	 num_malloced_so_far += 50;
	 group_indices =
	    (int*)simm_realloc(group_indices,num_malloced_so_far*sizeof(int),&rc);
	 if (rc == code_bad)
	    return (NULL);
      }

      group_indices[*num_groups] = enter_group(mod,buffer,muscle_number);
      if (group_indices[*num_groups] == -1)
	 return (NULL);
      (*num_groups)++;
   }

}



/* READ_MUSCLE_ATTACHMENT_POINTS: this routine reads muscle attachment points
 * from a file. It keeps reading from the file until the string "endpoints"
 * is encountered. It initially mallocs space for some points, and if it
 * fills up that space and finds more points, it reallocs the muscle point
 * array so that it can add the additional points.
 */

MusclePoint* read_muscle_attachment_points(int mod, FILE** fp, int* numpoints,
					   int* mp_orig_array_size,
					   SBoolean* has_wrapping_points,
					   SBoolean* has_force_points)
{

   int i;
   MusclePoint* mp;
   double range_num_1, range_num_2;
   char com[CHARBUFFER], segment_name[CHARBUFFER], gencoord_name[CHARBUFFER];
   char* line;
   ReturnCode rc;

   *numpoints = 0;
   *mp_orig_array_size = MP_ARRAY_INCREMENT;

   mp = (MusclePoint*)simm_malloc((*mp_orig_array_size)*sizeof(MusclePoint));
   if (mp == NULL)
      return (NULL);

   while (1)
   {
      if (fscanf(*fp,"%s",buffer) != 1)
      {
         error(none,"EOF while reading muscle attachment points");
         return (NULL);
      }

      if (STRINGS_ARE_EQUAL(buffer,"endpoints"))
         return (mp);

      /* Check to see if you need to increase the size of the muscle-point
       * array before reading any more points.
       */

      if ((*numpoints) >= (*mp_orig_array_size))
      {
         (*mp_orig_array_size) += MP_ARRAY_INCREMENT;
         mp = (MusclePoint*)simm_realloc(mp,(*mp_orig_array_size)*sizeof(MusclePoint),&rc);
         if (rc == code_bad)
         {
            (*mp_orig_array_size) -= MP_ARRAY_INCREMENT;
            return (NULL);
         }
      }

      /* If the string you just read was not "endpoints" then it must
       * be the start of a new point (so it's the x-coordinate).
       */
      if (sscanf(buffer,"%lg", &mp[*numpoints].point[XX]) != 1)
         return (NULL);

      (void)read_nonempty_line(fp,buffer);

      /* Read the y coordinate */

      line = parse_string(buffer,type_double,(void*)&mp[*numpoints].point[YY]);
      if (mp[*numpoints].point[YY] == ERROR_DOUBLE)
         return (NULL);

      /* Read the z coordinate */

      line = parse_string(line,type_double,(void*)&mp[*numpoints].point[ZZ]);
      if (mp[*numpoints].point[ZZ] == ERROR_DOUBLE)
         return (NULL);

      /* read the keyword "segment" */

      line = parse_string(line,type_string,(void*)buffer);

      /* read the segment name */

      line = parse_string(line,type_string,(void*)segment_name);
      if (segment_name == NULL)
         return (NULL);

      /* read a string from the leftover part of the line. For most points,
       * the leftover will be NULL. For wrapping points, the first string
       * in the leftover should be "ranges".
       */

      line = parse_string(line,type_string,(void*)com);

      mp[*numpoints].refpt = *numpoints;
      mp[*numpoints].numranges = 0;
      mp[*numpoints].ranges = NULL;
      if (STRINGS_ARE_EQUAL(com,"ranges"))
      {
         *has_wrapping_points = yes;
         line = parse_string(line,type_int,(void*)&mp[*numpoints].numranges);
         if (mp[*numpoints].numranges <= 0)
         {
            error(none,"The number of ranges must be greater than 0.");
            return (NULL);
         }
         mp[*numpoints].ranges = (PointRange*)simm_malloc(mp[*numpoints].numranges*
            sizeof(PointRange));
         if (mp[*numpoints].ranges == NULL)
            return (NULL);

         for (i=0; i<mp[*numpoints].numranges; i++)
         {
            line = parse_string(line,type_string,(void*)gencoord_name);
            if (gencoord_name == NULL ||
               name_is_gencoord(gencoord_name, model[mod], NULL, NULL, NULL, no) < 0)
            {
               free_and_nullify((void**)&mp[*numpoints].ranges);
               return (NULL);
            }

            line = get_xypair_from_string(line,&range_num_1,&range_num_2);
            if (line == NULL)
            {
               free_and_nullify((void**)&mp[*numpoints].ranges);
               return (NULL);
            }
            else
            {
               mp[*numpoints].ranges[i].start = MIN(range_num_1,range_num_2);
               mp[*numpoints].ranges[i].end = MAX(range_num_1,range_num_2);
            }
            mp[*numpoints].ranges[i].genc = enter_gencoord(mod,gencoord_name,no);
            if (mp[*numpoints].ranges[i].genc == -1)
            {
               (void)sprintf(errorbuffer,"Gencoord %s referenced in muscle file but not defined in joints file.", gencoord_name);
               error(none,errorbuffer);
               free_and_nullify((void**)&mp[*numpoints].ranges);
               return (NULL);
            }
         }
      }
      mp[*numpoints].segment = enter_segment(mod,segment_name,no);
      if (mp[*numpoints].segment == -1)
      {
         (void)sprintf(errorbuffer,"Segment %s referenced in muscle file but not defined in joints file.", segment_name);
         error(none,errorbuffer);
         free_and_nullify((void**)&mp[*numpoints].ranges);
         return (NULL);
      }

      /* Each muscle point starts out unselected. */
      mp[*numpoints].selected = no;
      mp[*numpoints].is_auto_wrap_point = no;
      mp[*numpoints].wrap_distance = 0.0;
      mp[*numpoints].num_wrap_pts = 0;
      mp[*numpoints].wrap_pts = NULL;

      (*numpoints)++;
   }
}

/* READ_LIGAMENT_ATTACHMENT_POINTS: this routine reads ligament attachment points
 * from a file. It keeps reading from the file until the string "endpoints"
 * is encountered. It initially mallocs space for some points, and if it
 * fills up that space and finds more points, it reallocs the muscle point
 * array so that it can add the additional points.
 */

LigamentPoint* read_ligament_attachment_points(int mod, FILE** fp, int* numpoints,
					   int* lp_orig_array_size,
					   SBoolean* has_wrapping_points,
					   SBoolean* has_force_points)
{

   int i;
   LigamentPoint* lp;
   double range_num_1, range_num_2;
   char com[CHARBUFFER], segment_name[CHARBUFFER], gencoord_name[CHARBUFFER];
   char* line;
   ReturnCode rc;

   *numpoints = 0;
   *lp_orig_array_size = MP_ARRAY_INCREMENT;

   lp = (LigamentPoint*)simm_malloc((*lp_orig_array_size)*sizeof(LigamentPoint));
   if (lp == NULL)
      return (NULL);

   while (1)
   {
      if (fscanf(*fp,"%s",buffer) != 1)
      {
         error(none,"EOF while reading ligament attachment points");
         return (NULL);
      }

      if (STRINGS_ARE_EQUAL(buffer,"endpoints"))
         return (lp);

      /* Check to see if you need to increase the size of the muscle-point
       * array before reading any more points.
       */

      if ((*numpoints) >= (*lp_orig_array_size))
      {
         (*lp_orig_array_size) += MP_ARRAY_INCREMENT;
         lp = (LigamentPoint*)simm_realloc(lp,(*lp_orig_array_size)*sizeof(MusclePoint),&rc);
         if (rc == code_bad)
         {
            (*lp_orig_array_size) -= MP_ARRAY_INCREMENT;
            return (NULL);
         }
      }

      /* If the string you just read was not "endpoints" then it must
       * be the start of a new point (so it's the x-coordinate), either
       * a number (e.g. "0.0346"), or a function (e.g. "f2").
       */
      if (sscanf(buffer,"%lg", &lp[*numpoints].point[XX]) != 1)
         return (NULL);

      (void)read_nonempty_line(fp,buffer);

      /* Read the y coordinate */

      line = parse_string(buffer,type_double,(void*)&lp[*numpoints].point[YY]);
      if (lp[*numpoints].point[YY] == ERROR_DOUBLE)
         return (NULL);

      /* Read the z coordinate */

      line = parse_string(line,type_double,(void*)&lp[*numpoints].point[ZZ]);
      if (lp[*numpoints].point[ZZ] == ERROR_DOUBLE)
         return (NULL);

      /* read the keyword "segment" */

      line = parse_string(line,type_string,(void*)buffer);

      /* read the segment name */

      line = parse_string(line,type_string,(void*)segment_name);
      if (segment_name == NULL)
         return (NULL);

      lp[*numpoints].state = on;

      /* read a string from the leftover part of the line. For most points,
       * the leftover will be NULL. For wrapping points, the first string
       * in the leftover should be "ranges".
       */

      line = parse_string(line,type_string,(void*)com);

      lp[*numpoints].numranges = 0;
      lp[*numpoints].ranges = NULL;
      if (STRINGS_ARE_EQUAL(com,"ranges"))
      {
         *has_wrapping_points = yes;
         line = parse_string(line,type_int,(void*)&lp[*numpoints].numranges);
         if (lp[*numpoints].numranges <= 0)
         {
            error(none,"The number of ranges must be greater than 0.");
            return (NULL);
         }
         lp[*numpoints].ranges = (PointRange*)simm_malloc(lp[*numpoints].numranges*
            sizeof(PointRange));
         if (lp[*numpoints].ranges == NULL)
            return (NULL);

         for (i=0; i<lp[*numpoints].numranges; i++)
         {
            line = parse_string(line,type_string,(void*)gencoord_name);
            if (gencoord_name == NULL ||
               name_is_gencoord(gencoord_name, model[mod], NULL, NULL, NULL, no) < 0)
            {
               free_and_nullify((void**)&lp[*numpoints].ranges);
               return (NULL);
            }

            line = get_xypair_from_string(line,&range_num_1,&range_num_2);
            if (line == NULL)
            {
               free_and_nullify((void**)&lp[*numpoints].ranges);
               return (NULL);
            }
            else
            {
               lp[*numpoints].ranges[i].start = MIN(range_num_1,range_num_2);
               lp[*numpoints].ranges[i].end = MAX(range_num_1,range_num_2);
            }
            lp[*numpoints].ranges[i].genc = enter_gencoord(mod,gencoord_name,no);
            if (lp[*numpoints].ranges[i].genc == -1)
            {
               (void)sprintf(errorbuffer,"Gencoord %s referenced in muscle file but not defined in joints file.", gencoord_name);
               error(none,errorbuffer);
               free_and_nullify((void**)&lp[*numpoints].ranges);
               return (NULL);
            }
         }
      }
      lp[*numpoints].segment = enter_segment(mod,segment_name,no);
      if (lp[*numpoints].segment == -1)
      {
         (void)sprintf(errorbuffer,"Segment %s referenced in muscle file but not defined in joints file.", segment_name);
         error(none,errorbuffer);
         free_and_nullify((void**)&lp[*numpoints].ranges);
         return (NULL);
      }

      (*numpoints)++;
   }
}

/* ---------------------------------------------------------------------------
   count_remaining_lines - scan to the end of the file, counting the number
     of lines remaining.
------------------------------------------------------------------------------ */
public int count_remaining_lines (FILE* file, SBoolean countEmptyLines)
{
   SBoolean lineHasContent = no;
   
   long fpos = ftell(file);
   
   int c, n = 0;
   
   for (c = fgetc(file); c != EOF; c = fgetc(file))
   {
      if (isgraph(c))
         lineHasContent = yes;
      
      if (c == '\n')
      {
         if (lineHasContent || countEmptyLines)
            n++;
         
         lineHasContent = no;
      }
   }
   if (lineHasContent)
      n++;
   
   fseek(file, fpos, SEEK_SET);
   
   return n;
}



/* READ_DOUBLE_ARRAY: this routine reads an array of pairs-of-doubles
 * (e.g. "(2.30, -0.05)"), as in the definition of a function. It keeps
 * reading until it encounters the ending string (e.g. "endfunction")
 * which is passed in. This routine is not trivial because it allows
 * spaces to be placed liberally in the string that it extracts the
 * doubles from (e.g. "( 2.30 , -0.05 )").
 */

ReturnCode read_double_array(FILE **fp, char ending[], char name[],
			     SplineFunction* func)
{

   int new_size;

   func->numpoints = 0;

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
      {
         (void)sprintf(errorbuffer,"Unexpected EOF reading function %s.", name);
         error(abort_action,errorbuffer);
         return (code_bad);
      }

      if (STRINGS_ARE_EQUAL(buffer,ending))
         break;

      /* If the string just read is not the ending string (e.g. "endfunction"),
       * then it must be part or all of an x-y pair "(x,y)". It cannot be more
       * than one x-y pair because there must be a space between each x-y pair.
       * Since there can be spaces within an x-y pair, you now want to continue
       * reading strings from the file until you encounter a closing parenthesis.
       */

      /* Before parsing this next string, first make sure that there is enough
       * space in the function structure for the next x-y pair.
       */

      if (func->numpoints == func->coefficient_array_size)
      {
         new_size = func->coefficient_array_size + SPLINE_ARRAY_INCREMENT;
         if (realloc_function(func,new_size) == code_bad)
            return (code_bad);
      }

      while (1)
      {
         if (string_contains_closing_paren(buffer) == yes)
            break;

         if (read_string(fp,&buffer[strlen(buffer)]) == EOF)
         {
            (void)sprintf(errorbuffer,"Unexpected EOF reading function %s.", name);
            error(abort_action,errorbuffer);
            return (code_bad);
         }
      }

      if (get_xypair_from_string(buffer,&func->x[func->numpoints],
         &func->y[func->numpoints]) == NULL)
      {
         (void)sprintf(errorbuffer,"Error reading x-y pair in function %s.", name);
         error(abort_action,errorbuffer);
         return (code_bad);
      }
      
      (func->numpoints)++;
   }

   if (func->numpoints < 2)
   {
      (void)sprintf(errorbuffer,"Function %s contains fewer than 2 points.", name);
      error(abort_action,errorbuffer);
      return (code_bad);
   }

   return (code_fine);

}



/* STRING_CONTAINS_CLOSING_PAREN: this routine scans a string to see if
 * it contains a closing parenthesis.
 */

static SBoolean string_contains_closing_paren(char str_buffer[])
{

   while (*str_buffer != STRING_TERMINATOR)
   {
      if (*(str_buffer++) == ')')
	 return (yes);
   }

   return (no);

}



/* GET_XYPAIR_FROM_STRING: this routine parses a string to extract a pair
 * of doubles from it. The string should be of the form: "(x, y)".
 */

static char* get_xypair_from_string(char str_buffer[], double* x, double* y)
{

   char junk;
   char* buffer_ptr;

   *x = *y = ERROR_DOUBLE;

   buffer_ptr = parse_string(str_buffer,type_char,(void*)&junk); /* open paren */
   buffer_ptr = parse_string(buffer_ptr,type_double,(void*)x);   /* x coord */
   buffer_ptr = parse_string(buffer_ptr,type_char,(void*)&junk); /* comma */
   buffer_ptr = parse_string(buffer_ptr,type_double,(void*)y);   /* y coord */

   buffer_ptr = parse_string(buffer_ptr,type_char,(void*)&junk); /* close paren*/

   if (*x == ERROR_DOUBLE || *y == ERROR_DOUBLE)
      return (NULL);

   return (buffer_ptr);

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
      return (NULL);

   buffer_ptr = tmp_buffer;

   while (CHAR_IS_WHITE_SPACE(*str_buffer))
      str_buffer++;

   if (var_type == type_char)
   {
      *((char*)dest_var) = *str_buffer;
      return (str_buffer+1);
   }

   if (var_type == type_string)
   {
      if (STRING_IS_NULL(str_buffer))
         *((char*)dest_var) = STRING_TERMINATOR;
      else
         (void)sscanf(str_buffer,"%s", (char*)dest_var);
      return (str_buffer + strlen((char*)dest_var));
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
      return (str_buffer);
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
      return (str_buffer);
   }

   (void)sprintf(errorbuffer,"Unknown variable type (%d) passed to parse_string().",
	   (int)var_type);
   error(none,errorbuffer);

   return (NULL);

}



/* GET_STRING_PAIR: this routine takes a string of the form "(str1,str2)",
 * which can have spaces between any of the 5 components, and returns the
 * two internal strings, str1 and str2.
 */

ReturnCode get_string_pair(char str_buffer[], char str1[], char str2[])
{

   /* scan past the opening parenthesis */

   while (*str_buffer != '(' && *str_buffer != STRING_TERMINATOR)
      str_buffer++;
   str_buffer++;

   if (STRING_IS_NULL(str_buffer))
      return (code_bad);

   /* scan upto the start of str1 */

   while (*str_buffer == SPACE || *str_buffer == TAB)
      str_buffer++;

   if (STRING_IS_NULL(str_buffer))
      return (code_bad);

   /* copy the first string to the str1 array */

   while (*str_buffer != ',' && *str_buffer != SPACE && *str_buffer != TAB)
      *str1++ = *str_buffer++;
   *str1 = STRING_TERMINATOR;

   /* scan upto the start of str2 */

   while (*str_buffer == ',' || *str_buffer == SPACE || *str_buffer == TAB)
      str_buffer++;

   if (STRING_IS_NULL(str_buffer))
      return (code_bad);

   /* copy the second string to the str2 array */

   while (*str_buffer != ')' && *str_buffer != SPACE && *str_buffer != TAB)
      *str2++ = *str_buffer++;
   *str2 = STRING_TERMINATOR;

   return (code_fine);

}


/* NAME_IS_GENCOORD: this function checks to see if a string is the name
 * of one of the model's gencoords, with an optional suffix and also
 * possibly the extra suffix "_spl" or "_gcv" (possibly followed by a cutoff
 * frequency). It returns the index of the gencoord if there is a match.
 * If stripEnd is yes, the optional splineType and cutoff frequency are
 * removed from the name if a match is made.
 */
int name_is_gencoord(char name[], ModelStruct* ms, char suffix[],
                     SplineType* splineType, double* cutoffFrequency, SBoolean stripEnd)
{
   int i, len, lenQ, Qnum, index, maxLen;
   char *ptr, *newEnd;

   len = strlen(name);

   /* First check to see if the string begins with the name of a gencoord.
    * To handle models which have overlapping gencoord names, like
    * "q1" and "q10", check for a match with all gencoords, and take
    * the longest match.
    */
   for (i = 0, index = -1, maxLen = -1; i < ms->numgencoords; i++)
   {
      lenQ = strlen(ms->gencoord[i].name);
      if (len >= lenQ)
      {
         if (!strncmp(name, ms->gencoord[i].name, lenQ))
         {
            if (lenQ > maxLen)
            {
               index = i;
               maxLen = lenQ;
            }
         }
      }
   }

   if (index == -1)
      return -1;

   /* You've found a matching gencoord name, so move ptr past the name and
    * get ready to check the suffixes.
    */
   Qnum = index;
   ptr = &name[maxLen];
   len -= maxLen;

   /* If a suffix was passed in, check to see if the name ends in that suffix.
    * If it does, remove the suffix and continue on. If it does not, return -1
    * because the passed-in suffix must be in the name.
    */
   if (suffix)
   {
      int suffix_len = strlen(suffix);

      if (len >= suffix_len)
      {
         if (!strncmp(ptr, suffix, suffix_len))
         {
            ptr += suffix_len;
            len -= suffix_len;
         }
         else
         {
            return -1;
         }
      }
      else
      {
         return -1;
      }
   }

   /* Store a pointer to the character right after the name (plus suffix, if specified).
    * This will be the new end of the string if stripEnd == yes.
    */
   newEnd = ptr;

   /* If splineType and cutoffFrequency are not NULL, check to see if the name ends in
    * natural_cubic_text or gcv_text (followed by an optional cutoff frequency). If it
    * does, set *splineType to the appropriate type. If no spline label is found, set
    * the type to step_func.
    */
   if (splineType && cutoffFrequency)
   {
      int matched_spl = 0, matched_lin = 0;
		int lin_len = strlen(linear_text);
      int spl_len = strlen(natural_cubic_text);
      int gcv_len = strlen(gcv_text);

      *splineType = step_function;
      *cutoffFrequency = -1.0; // by default there is no smoothing

		if (len >= lin_len)
		{
         if (!strncmp(ptr, linear_text, lin_len))
         {
            *splineType = linear;
            ptr += lin_len;
            len -= lin_len;
            matched_lin = 1;
         }
		}

		if (!matched_lin && len >= spl_len)
      {
         if (!strncmp(ptr, natural_cubic_text, spl_len))
         {
            *splineType = natural_cubic;
            ptr += spl_len;
            len -= spl_len;
            matched_spl = 1;
         }
      }

      if (!matched_lin && !matched_spl && len >= gcv_len)
      {
         if (!strncmp(ptr, gcv_text, gcv_len))
         {
            ptr += gcv_len;
            len -= gcv_len;
            *splineType = gcv_spline;
            if (len > 0)
            {
               char* intPtr = buffer;

               /* Move over the underscore and look for an integer. */
               if (*(ptr++) != '_')
               {
                  return -1;
               }
               else
               {
                  len--; /* for the underscore character */
                  while (*ptr >= '0' && *ptr <= '9')
                  {
                     *(intPtr++) = *(ptr++);
                     len--;
                  }
                  *intPtr = STRING_TERMINATOR;
                  *cutoffFrequency = atof(buffer);
               }
            }
         }
      }
   }

   /* If there are extra characters after the suffixes, return an error. */
   if (len > 0)
      return -1;

   /* Strip off the text for the spline type and cutoff frequency. */
   if (stripEnd == yes)
      *newEnd = STRING_TERMINATOR;

   return Qnum;
}


/* NAME_IS_BODY_SEGMENT: this function checks to see if a string is the name
 * of one of the model's body segments. The name can be followed by the name of
 * any of the model's motion objects, plus an animation component (e.g., "_px" or "_vy").
 * Also, the string can contain a suffix of "_gcv" or "_spl", possibly followed
 * by a cutoff frequency. It returns the index of the body segment if there is
 * a match. If stripEnd is yes, the optional splineType and cutoff frequency are
 * removed from the name if a match is made.
 */
int name_is_body_segment(ModelStruct* ms, char name[], int* motion_object, int* component,
                         SplineType* splineType, double* cutoffFrequency, SBoolean stripEnd)
{
   int i, index, len, lenS, maxLen, Snum;
   char *ptr, *newEnd;

   len = strlen(name);

   /* First check to see if the string begins with the name of a segment.
    * To handle models which have overlapping segment names, like
    * "femur" and "femur1", check for a match with all segments, and take
    * the longest match.
    */
   for (i = 0, index = -1, maxLen = -1; i < ms->numsegments; i++)
   {
      lenS = strlen(ms->segment[i].name);
      if (len >= lenS)
      {
         if (!strncmp(name, ms->segment[i].name, lenS))
         {
            if (lenS > maxLen)
            {
               index = i;
               maxLen = lenS;
            }
         }
      }
   }

   if (index == -1)
      return -1;

   /* You've found a matching segment name, so move ptr past the name and
    * get ready to check the suffixes.
    */
   Snum = index;
   ptr = &name[maxLen];
   len -= maxLen;

   if (motion_object)
   {
      int lenM;
      *motion_object = -1;

      /* The motion object name must be preceded by an underscore. */
      if (*ptr == '_')
      {
         /* Temporarily move past the underscore. */
         ptr++;
         len--;

         /* Find the first motion object which matches the name.
          * This suffix is optional, so do not return if there is no match.
          */
         for (i = 0; i < ms->num_motion_objects; i++)
         {
            lenM = strlen(ms->motion_objects[i].name);
            if (len >= lenM)
            {
               if (!strncmp(ptr, ms->motion_objects[i].name, lenM))
                  break;
            }
         }
         if (i < ms->num_motion_objects)
         {
            *motion_object = i;
            ptr += lenM;
            len -= lenM;
         }
         else
         {
            /* Move backwards over the underscore. */
            ptr--;
            len++;
         }
      }
   }

   if (component)
   {
      *component = -1;

      /* Check to see if a component (e.g., "_px") is next in the string.
       * This suffix is optional, and independent of the motion object name suffix.
       */
      if (!strncmp(ptr, "_tx", 3) || !strncmp(ptr, "_px", 3))
         *component = MO_TX;
      else if (!strncmp(ptr, "_ty", 3) || !strncmp(ptr, "_py", 3))
         *component = MO_TY;
      else if (!strncmp(ptr, "_tz", 3) || !strncmp(ptr, "_pz", 3))
         *component = MO_TZ;
      else if (!strncmp(ptr, "_vx", 3))
         *component = MO_VX;
      else if (!strncmp(ptr, "_vy", 3))
         *component = MO_VY;
      else if (!strncmp(ptr, "_vz", 3))
         *component = MO_VZ;
      else if (!strncmp(ptr, "_sx", 3))
         *component = MO_SX;
      else if (!strncmp(ptr, "_sy", 3))
         *component = MO_SY;
      else if (!strncmp(ptr, "_sz", 3))
         *component = MO_SZ;
      else if (!strncmp(ptr, "_cr", 3))
         *component = MO_CR;
      else if (!strncmp(ptr, "_cg", 3))
         *component = MO_CG;
      else if (!strncmp(ptr, "_cb", 3))
         *component = MO_CB;
      else if (!strncmp(ptr, "_x", 2))
         *component = MO_X;
      else if (!strncmp(ptr, "_y", 2))
         *component = MO_Y;
      else if (!strncmp(ptr, "_z", 2))
         *component = MO_Z;

      if (*component >= MO_TX && *component <= MO_CB)
      {
         ptr += 3;
         len -= 3;
      }
      else if (*component >= MO_X && *component <= MO_Z)
      {
         ptr += 2;
         len -= 2;
      }
   }

   /* Store a pointer to the character right after the part of the name processed
    * so far. This will be the new end of the string if stripEnd == yes.
    */
   newEnd = ptr;

   /* If splineType and cutoffFrequency are not NULL, check to see if the name ends in
    * natural_cubic_text or gcv_text (followed by an optional cutoff frequency). If it
    * does, set *splineType to the appropriate type. If no spline label is found, set
    * the type to step_func.
    */
   if (splineType && cutoffFrequency)
   {
      int matched_spl = 0, matched_lin = 0;
		int lin_len = strlen(linear_text);
      int spl_len = strlen(natural_cubic_text);
      int gcv_len = strlen(gcv_text);

      *splineType = step_function;
      *cutoffFrequency = -1.0; // by default there is no smoothing

      if (len >= lin_len)
      {
         if (!strncmp(ptr, linear_text, lin_len))
         {
            *splineType = linear;
            ptr += lin_len;
            len -= lin_len;
            matched_lin = 1;
         }
      }

      if (!matched_lin && len >= spl_len)
      {
         if (!strncmp(ptr, natural_cubic_text, spl_len))
         {
            *splineType = natural_cubic;
            ptr += spl_len;
            len -= spl_len;
            matched_spl = 1;
         }
      }

      if (!matched_lin && !matched_spl && len >= gcv_len)
      {
         if (!strncmp(ptr, gcv_text, gcv_len))
         {
            ptr += gcv_len;
            len -= gcv_len;
            *splineType = gcv_spline;
            if (len > 0)
            {
               char* intPtr = buffer;

               /* Move over the underscore and look for an integer. */
               if (*(ptr++) != '_')
               {
                  return -1;
               }
               else
               {
                  len--; /* for the underscore character */
                  while (*ptr >= '0' && *ptr <= '9')
                  {
                     *(intPtr++) = *(ptr++);
                     len--;
                  }
                  *intPtr = STRING_TERMINATOR;
                  *cutoffFrequency = atof(buffer);
               }
            }
         }
      }
   }

   /* If there are extra characters after the suffixes, return an error. */
   if (len > 0)
      return -1;

   /* Strip off the text for the spline type and cutoff frequency. */
   if (stripEnd == yes)
      *newEnd = STRING_TERMINATOR;

   return Snum;
}


/* NAME_IS_MUSCLE: this function checks to see if a string is the name
 * of one of the model's muscles, with an optional suffix and also
 * possibly the extra suffix "_spl" or "_gcv" (possibly followed by a cutoff
 * frequency). It returns the index of the muscle if there is a match.
 * If stripEnd is yes, the optional splineType and cutoff frequency are
 * removed from the name if a match is made.
 */
int name_is_muscle(ModelStruct* ms, char name[], char suffix[],
                   SplineType* splineType, double* cutoffFrequency, SBoolean stripEnd)
{
   int i, len, lenM, Mnum, index, maxLen;
   char *ptr, *newEnd;

   len = strlen(name);

   /* First check to see if the string begins with the name of a muscle.
    * To handle models which have overlapping muscle names, like
    * "trap1" and "trap10", check for a match with all muscles, and take
    * the longest match.
    */
   for (i = 0, index = -1, maxLen = -1; i < ms->nummuscles; i++)
   {
      lenM = strlen(ms->muscle[i].name);
      if (len >= lenM)
      {
         if (!strncmp(name, ms->muscle[i].name, lenM))
         {
            if (lenM > maxLen)
            {
               index = i;
               maxLen = lenM;
            }
         }
      }
   }

   if (index == -1)
      return -1;

   /* You've found a matching muscle name, so move ptr past the name and
    * get ready to check the suffixes.
    */
   Mnum = index;
   ptr = &name[maxLen];
   len -= maxLen;

   /* If a suffix was passed in, check to see if the name ends in that suffix.
    * If it does, remove the suffix and continue on. If it does not, return -1
    * because the suffix is not optional.
    */
   if (suffix)
   {
      int suffix_len = strlen(suffix);

      if (len >= suffix_len)
      {
         if (!strncmp(ptr, suffix, suffix_len))
         {
            ptr += suffix_len;
            len -= suffix_len;
         }
         else
         {
            return -1;
         }
      }
      else
      {
         return -1;
      }
   }

   /* Store a pointer to the character right after the name (plus suffix, if specified).
    * This will be the new end of the string if stripEnd == yes.
    */
   newEnd = ptr;

   /* If splineType and cutoffFrequency are not NULL, check to see if the name ends in
    * natural_cubic_text or gcv_text (followed by an optional cutoff frequency). If it
    * does, set *splineType to the appropriate type. If no spline label is found, set
    * the type to step_func.
    */
   if (splineType && cutoffFrequency)
   {
      int matched_spl = 0, matched_lin = 0;
		int lin_len = strlen(linear_text);
      int spl_len = strlen(natural_cubic_text);
      int gcv_len = strlen(gcv_text);

      *splineType = step_function;
      *cutoffFrequency = -1.0; // by default there is no smoothing

      if (len >= lin_len)
      {
         if (!strncmp(ptr, linear_text, lin_len))
         {
            *splineType = linear;
            ptr += lin_len;
            len -= lin_len;
            matched_lin = 1;
         }
      }

      if (!matched_lin && len >= spl_len)
      {
         if (!strncmp(ptr, natural_cubic_text, spl_len))
         {
            *splineType = natural_cubic;
            ptr += spl_len;
            len -= spl_len;
            matched_spl = 1;
         }
      }

      if (!matched_lin && !matched_spl && len >= gcv_len)
      {
         if (!strncmp(ptr, gcv_text, gcv_len))
         {
            ptr += gcv_len;
            len -= gcv_len;
            *splineType = gcv_spline;
            if (len > 0)
            {
               char* intPtr = buffer;

               /* Move over the underscore and look for an integer. */
               if (*(ptr++) != '_')
               {
                  return -1;
               }
               else
               {
                  len--; /* for the underscore character */
                  while (*ptr >= '0' && *ptr <= '9')
                  {
                     *(intPtr++) = *(ptr++);
                     len--;
                  }
                  *intPtr = STRING_TERMINATOR;
                  *cutoffFrequency = atof(buffer);
               }
            }
         }
      }
   }

   /* If there are extra characters after the suffixes, return an error. */
   if (len > 0)
      return -1;

   /* Strip off the text for the spline type and cutoff frequency. */
   if (stripEnd == yes)
      *newEnd = STRING_TERMINATOR;

   return Mnum;
}


SBoolean muscle_has_force_params(MuscleStruct* ms)
{

   if (ms->optimal_fiber_length == NULL || ms->resting_tendon_length == NULL ||
       ms->pennation_angle == NULL || ms->max_isometric_force == NULL ||
       ms->tendon_force_len_curve == NULL || ms->active_force_len_curve == NULL ||
       ms->passive_force_len_curve == NULL)
      return (no);

   return (yes);

}


int divide_string(char string[], char* word_array[], int max_words)
{

   int i, len, num_chars, word_start=0, count=0, last_one_white=1;

   if (max_words <= 0)
      return (0);

   len = strlen(string) + 1;

   /* When you hit " x", set word_start to point to the 'x'
    * When you hit "x ", copy the word ending in 'x' to the next
    * slot in word_array.
    */

   for (i=0; i<len; i++)
   {
      if (CHAR_IS_WHITE_SPACE(string[i]) || (string[i] == STRING_TERMINATOR))
      {
	 /* Copy the last word to word_array[] */
	 if (last_one_white == 0)
	 {
	    num_chars = i - word_start;
	    word_array[count] = (char*)simm_malloc((num_chars+1)*sizeof(char));
	    strncpy(word_array[count],&string[word_start],num_chars);
	    word_array[count][num_chars] = STRING_TERMINATOR;
	    if (++count == max_words)
	       return (count);
	 }
	 last_one_white = 1;
      }
      else
      {
	 if (last_one_white == 1)
	    word_start = i;
	 last_one_white = 0;
      }
   }

   return (count);

}

/* Returns the index of the last occurrence of a character in a string
 * that belongs to a set of characters.
 */

int strrcspn(const char* string, const char* strCharSet)
{
   int i, j, len = strlen(string);
   int setLen = strlen(strCharSet);

   for (i = len - 1; i >= 0; i--)
   {
      for (j = 0; j < setLen; j++)
      {
         if (string[i] == strCharSet[j])
            return i;
      }
   }

   return -1;
}


/* Extracts a name and a value (double) from a specified character string.
 * The name is assumed to be at the beginning of the string, and can have
 * any number of tokens in it, followed by a colon (which may or may not
 * be attached to the name or the value). This function modifies the string
 * by inserting a NULL character after the name.
 */
int get_name_and_value_from_string(char* string, double* value)
{
   int len, pos;
   char* ptr;

   /* The string may have a colon in it between the name and the value,
    * so search for the last colon and replace it with a space.
    */
   ptr = strrchr(string, ':');
   if (ptr)
      *ptr = ' ';

   /* Strip trailing white space, then find the position of the
    * last character of white space. The value is contained in the
    * characters after this last white space.
    */
   strip_trailing_white_space(string);
   len = strlen(string);
   pos = strrcspn(string, "\r\t ");

   if (pos > 0 && pos < len - 1)
   {
      if (sscanf(&string[pos+1], "%lg", value) != 1)
         return 0;
      string[pos] = STRING_TERMINATOR;
      strip_trailing_white_space(string);
   }
   else
   {
      return 0;
   }

   return 1;
}


void strip_trailing_white_space(char string[])
{

   int i;

   for (i=strlen(string)-1; i>=0; i--)
   {
      if (CHAR_IS_WHITE_SPACE(string[i]))
         string[i] = STRING_TERMINATOR;
      else
         break;
   }

}

/* -------------------------------------------------------------------------
   STATIC DATA
---------------------------------------------------------------------------- */
static int   sNumDefaultAcppOptions = 2;
static int   sNumTotalAcppOptions   = 2;
static char* sAcppOptions[128]      = { "acpp", "-P" };

/* -------------------------------------------------------------------------
   clear_preprocessor_options - reset the options that are passed to acpp()
      their defaults.
---------------------------------------------------------------------------- */
public void clear_preprocessor_options ()
{
   int i;
   
   for (i = sNumDefaultAcppOptions; i < sNumTotalAcppOptions; i++)
      free(sAcppOptions[i]);
   
   sNumTotalAcppOptions = sNumDefaultAcppOptions;
}

/* -------------------------------------------------------------------------
   add_preprocessor_option - format and append the specified string to the
      list of command-line options passed to acpp().
   
   NOTE: "default" options must precede "non-default" option in the list,
      therefore be certain to call clear_preprocessor_options() before
      calling this routine with 'isDefaultOption' equal to 'yes'.
---------------------------------------------------------------------------- */
public void add_preprocessor_option (SBoolean isDefaultOption, const char* format, ...)
{
   va_list ap;
   
   va_start(ap, format);
     vsprintf(buffer, format, ap);
   va_end(ap);
   
   mstrcpy(&sAcppOptions[sNumTotalAcppOptions++], buffer);
   
   if (isDefaultOption)
      sNumDefaultAcppOptions++;
}

/* -------------------------------------------------------------------------
   acpp_message_proc - 
---------------------------------------------------------------------------- */
static int acpp_message_proc (const char* format, ...)
{
   va_list ap;
   int n;
   
   va_start(ap, format);
   n = vsprintf(buffer, format, ap);
   va_end(ap);
   
   simm_printf(yes, buffer);
   
   return n;
}

typedef int (*_msg_proc)(const char* format, ...);

int acpp_main(int argc, char** argv, _msg_proc);

/* -------------------------------------------------------------------------
   acpp - 
---------------------------------------------------------------------------- */
static void acpp (char in_file[], const char out_file[])
{
   char* pure_path = NULL;
   int argc;

   /* Add the folder containing the input file to the search path
    * for acpp. This is so that if the file includes other files,
    * its folder is searched for them. By default, acpp will search
    * only SIMM\resources and the current directory (which may not
    * be the directory the input file is in).
    */
   get_pure_path_from_path(in_file, &pure_path);
   if (pure_path)
   {
      add_preprocessor_option(no, "-I%s", pure_path);
      free(pure_path);
   }

   argc = sNumTotalAcppOptions;
   sAcppOptions[argc++] = (char*) in_file;
   sAcppOptions[argc++] = (char*) out_file;

#if 0
   {
       int i;

       fprintf(stderr, "\n");
       for (i = 0; i < argc; i++)
           fprintf(stderr, "%s ", sAcppOptions[i]);
       fprintf(stderr, "\n\n");
   }
#endif

   remove(out_file); /* required to work-around CodeWarrior bug */

#ifndef __linux__
   if (acpp_main(argc, sAcppOptions, acpp_message_proc) != 0)
   {
      const char* infile = strrchr(in_file, DIR_SEP_CHAR);
      
      if (infile)
         infile++;
      else
         infile = in_file;
      
      sprintf(errorbuffer, "Error running preproccesor on %s.", infile);
      
      error(none, errorbuffer);
   }
#else
	char cmdline[1024];
	sprintf(cmdline,"cp %s %s",in_file,out_file);
	printf("Executing '%s' in place of acpp_main", cmdline);
	system(cmdline);
#endif

#if 0
   /* copy the preprocessed file so we can get a look at it:
    */
   {
	static int sCounter = 1;
	sprintf(buffer, "cp %s /suture/usr/people/simm/ACPP-OUT-%d.txt", out_file, sCounter++);
	glutSystem(buffer);
   }
#endif

} /* acpp */

/* -------------------------------------------------------------------------
   read_deform - 
---------------------------------------------------------------------------- */
ReturnCode read_deform (FILE** fp, SegmentStruct* seg, int segmentnum)
{
   DeformObject* dfm = NULL;
   double xyz[3];
   int i = seg->num_deforms;
   DMatrix m;
   
   if (seg->num_deforms >= seg->deform_obj_array_size)
   {
      ReturnCode rc = code_fine;
      
      /* expand deform object array if necessary */
      seg->deform_obj_array_size += 2;
      
      if (seg->deform == NULL)
      {
         seg->deform = (DeformObject*) simm_malloc(
               seg->deform_obj_array_size * sizeof(DeformObject));
      }
      else
      {
         seg->deform = (DeformObject*) simm_realloc(seg->deform,
               seg->deform_obj_array_size * sizeof(DeformObject), &rc);
      }
      
      if (rc == code_bad || seg->deform == NULL)
      {
         seg->deform_obj_array_size -= 2;
         return code_bad;
      }
   }

   seg->num_deforms++;
   
   /* initialize the new deform */
   dfm = &seg->deform[i];
   
   init_deform(dfm);
   
   dfm->segment = segmentnum;

   if (fscanf(*fp,"%s", buffer) != 1)
   {
      error(abort_action,"Error reading name in deform object definition");
      return code_bad;
   }
   else
      mstrcpy(&dfm->name,buffer);
   
   while (1)
   {
      if (read_string(fp,buffer) == EOF)
         break;

      if (buffer[0] == '#')
      {
	 read_nonempty_line(fp,buffer);
	 continue;
      }

      if (STRINGS_ARE_EQUAL(buffer,"active"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"false"))
            dfm->active = no;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"visible"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"false"))
            dfm->visible = no;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"autoreset"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"yes") || STRINGS_ARE_EQUAL(buffer,"true"))
            dfm->autoReset = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"translationonly"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"yes") || STRINGS_ARE_EQUAL(buffer,"true"))
            dfm->translationOnly = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"innermin"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &dfm->innerMin.xyz[0],
                    &dfm->innerMin.xyz[1], &dfm->innerMin.xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading \'innermin\' for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"innermax"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &dfm->innerMax.xyz[0],
                    &dfm->innerMax.xyz[1], &dfm->innerMax.xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading \'innermax\' for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"outermin"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &dfm->outerMin.xyz[0],
                    &dfm->outerMin.xyz[1], &dfm->outerMin.xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading \'outermin\' for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"outermax"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &dfm->outerMax.xyz[0],
                    &dfm->outerMax.xyz[1], &dfm->outerMax.xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading \'outermax\' for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"xyz_body_rotation_POSITION"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &xyz[0], &xyz[1], &xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading rotation for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         identity_matrix(m);
         x_rotate_matrix_bodyfixed(m, xyz[0] * DTOR);
         y_rotate_matrix_bodyfixed(m, xyz[1] * DTOR);
         z_rotate_matrix_bodyfixed(m, xyz[2] * DTOR);
         extract_rotation(m, &dfm->position.rotation_axis, &dfm->position.rotation_angle);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"translation_POSITION"))
      {
         if (fscanf(*fp,"%lg %lg %lg",
                    &dfm->position.translation.xyz[0],
                    &dfm->position.translation.xyz[1],
                    &dfm->position.translation.xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading translation for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"xyz_body_rotation_DEFORM_START"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &xyz[0], &xyz[1], &xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading rotation for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         identity_matrix(m);
         x_rotate_matrix_bodyfixed(m, xyz[0] * DTOR);
         y_rotate_matrix_bodyfixed(m, xyz[1] * DTOR);
         z_rotate_matrix_bodyfixed(m, xyz[2] * DTOR);
         extract_rotation(m, &dfm->deform_start.rotation_axis, &dfm->deform_start.rotation_angle);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"translation_DEFORM_START"))
      {
         if (fscanf(*fp,"%lg %lg %lg",
                    &dfm->deform_start.translation.xyz[0],
                    &dfm->deform_start.translation.xyz[1],
                    &dfm->deform_start.translation.xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading translation for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"xyz_body_rotation_DEFORM_END"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &xyz[0], &xyz[1], &xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading rotation for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         identity_matrix(m);
         x_rotate_matrix_bodyfixed(m, xyz[0] * DTOR);
         y_rotate_matrix_bodyfixed(m, xyz[1] * DTOR);
         z_rotate_matrix_bodyfixed(m, xyz[2] * DTOR);
         extract_rotation(m, &dfm->deform_end.rotation_axis, &dfm->deform_end.rotation_angle);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"translation_DEFORM_END"))
      {
         if (fscanf(*fp,"%lg %lg %lg",
                    &dfm->deform_end.translation.xyz[0],
                    &dfm->deform_end.translation.xyz[1],
                    &dfm->deform_end.translation.xyz[2]) != 3)
         {
            sprintf(errorbuffer, "Error reading translation for deform object %s", dfm->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"enddeform"))
         break;
      else
      {
         sprintf(errorbuffer, "Unrecognized text in deform object %s: %s", dfm->name, buffer);
         error(abort_action,errorbuffer);
         return code_bad;
      }
   }

   /* Check the inner and outer boxes to see if they are well defined. */
   if (dfm->innerMin.xyz[0] > dfm->innerMax.xyz[0] || dfm->innerMin.xyz[0] < dfm->outerMin.xyz[0])
   {
      sprintf(errorbuffer,"Error setting up deform object %s", dfm->name);
      error(none, errorbuffer);
      error(recover,"Inner min X must be greater than outer min X and less than inner max X");
      return code_bad;
   }
   if (dfm->innerMin.xyz[1] > dfm->innerMax.xyz[1] || dfm->innerMin.xyz[1] < dfm->outerMin.xyz[1])
   {
      sprintf(errorbuffer,"Error setting up deform object %s", dfm->name);
      error(none, errorbuffer);
      error(recover,"Inner min Y must be greater than outer min Y and less than inner max Y");
      return code_bad;
   }
   if (dfm->innerMin.xyz[2] > dfm->innerMax.xyz[2] || dfm->innerMin.xyz[2] < dfm->outerMin.xyz[2])
   {
      sprintf(errorbuffer,"Error setting up deform object %s", dfm->name);
      error(none, errorbuffer);
      error(recover,"Inner min Z must be greater than outer min Z and less than inner max Z");
      return code_bad;
   }
   if (dfm->innerMax.xyz[0] > dfm->outerMax.xyz[0])
   {
      sprintf(errorbuffer,"Error setting up deform object %s", dfm->name);
      error(none, errorbuffer);
      error(recover,"Inner max X must be less than outer max X");
      return code_bad;
   }
   if (dfm->innerMax.xyz[1] > dfm->outerMax.xyz[1])
   {
      sprintf(errorbuffer,"Error setting up deform object %s", dfm->name);
      error(none, errorbuffer);
      error(recover,"Inner max Y must be less than outer max Y");
      return code_bad;
   }
   if (dfm->innerMax.xyz[2] > dfm->outerMax.xyz[2])
   {
      sprintf(errorbuffer,"Error setting up deform object %s", dfm->name);
      error(none, errorbuffer);
      error(recover,"Inner max Z must be less than outer max Z");
      return code_bad;
   }

   /* NOTE: 'xyz_body_rotation_DEFORM' and 'translation_DEFORM' specify the
    * *delta* transformation from the "position" frame to the "deform_start" 
    * frame.  The code below converts this into the DeformObject's true
    * "deform_start" transform which is measured relative to the parent segment's
    * frame.
    */
   dfm->position.xforms_valid = no;
   dfm->deform_start.xforms_valid = no;
   dfm->deform_end.xforms_valid = no;
   recalc_deform_xforms(seg, dfm);
   
   /* deform start */
   copy_4x4matrix(dfm->deform_start.from_local_xform, m);
   append_matrix(m, dfm->position.from_local_xform);
   
   extract_rotation(m, &dfm->deform_start.rotation_axis, &dfm->deform_start.rotation_angle);
   dfm->deform_start.translation.xyz[0] = m[3][0];
   dfm->deform_start.translation.xyz[1] = m[3][1];
   dfm->deform_start.translation.xyz[2] = m[3][2];
   dfm->deform_start.xforms_valid = no;
   
   /* deform end */
   copy_4x4matrix(dfm->deform_end.from_local_xform, m);
   append_matrix(m, dfm->position.from_local_xform);
   
   extract_rotation(m, &dfm->deform_end.rotation_axis, &dfm->deform_end.rotation_angle);
   dfm->deform_end.translation.xyz[0] = m[3][0];
   dfm->deform_end.translation.xyz[1] = m[3][1];
   dfm->deform_end.translation.xyz[2] = m[3][2];
   dfm->deform_end.xforms_valid = no;
   
   recalc_deform_xforms(seg, dfm);

#ifndef ENGINE
   init_deform_box_verts(dfm);
#endif
   
   return code_fine;

} /* read_deform */


/* -------------------------------------------------------------------------
   read_deformity - 
---------------------------------------------------------------------------- */
ReturnCode read_deformity (ModelStruct* ms, FILE** fp)
{
   ReturnCode  rc;
   Deformity* dty;

   if (ms->num_deformities == ms->deformity_array_size)
   {
      ms->deformity_array_size += DEFORMITY_ARRAY_INCREMENT;

      ms->deformity = (Deformity*) simm_realloc(ms->deformity,
			    ms->deformity_array_size * sizeof(Deformity), &rc);
      if (rc == code_bad)
      {
         ms->deformity_array_size -= DEFORMITY_ARRAY_INCREMENT;
	         return code_bad;
      }
   }

   dty = &ms->deformity[ms->num_deformities];

   init_deformity(dty);

   if (fscanf(*fp,"%s", buffer) != 1)
   {
      error(abort_action,"Error reading name in deformity definition");
      return code_bad;
   }
   else
      mstrcpy(&dty->name, buffer);

   while (1)
   {
      if (read_string(fp, buffer) == EOF)
         break;

      if (buffer[0] == '#')
      {
	      read_nonempty_line(fp,buffer);
	      continue;
      }

      if (STRINGS_ARE_EQUAL(buffer,"default_value") || STRINGS_ARE_EQUAL(buffer,"value"))
      {
         if (fscanf(*fp, "%lg", &dty->default_value) != 1)
         {
            sprintf(errorbuffer, "Error reading value for deformity: %s.", dty->name);
	         error(none, errorbuffer);
	         return code_bad;
         }
         else
         {
            dty->value = dty->default_value;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"range"))
      {
         if (fscanf(*fp, "%lg %lg", &dty->range.start, &dty->range.end) != 2)
         {
            sprintf(errorbuffer, "Error reading range for deformity: %s.", dty->name);
	         error(none, errorbuffer);
	         return code_bad;
         }

         if (dty->range.start > dty->range.end)
         {
            double tmp = dty->range.start;
            dty->range.start = dty->range.end;
            dty->range.end = tmp;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"keys"))
      {
	      int nk;
	      char key1[64], key2[64];
	 
	      read_line(fp, buffer);
	 
	      nk = sscanf(buffer,"%s %s", key1, key2);
	 
	      if (nk == 1)
	         dty->keys[0] = dty->keys[1] = lookup_simm_key(key1);
	      else if (nk == 2)
	      {
	         dty->keys[0] = lookup_simm_key(key1);
	         dty->keys[1] = lookup_simm_key(key2);
	      }
	      else
	      {
	         sprintf(errorbuffer, "Error reading keys for deformity: %s.", dty->name);
	         error(recover,errorbuffer);
	      }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"deform"))
      {
         char* dfmName = buffer;

         read_line(fp, dfmName);

         _strip_outer_whitespace(dfmName);  /* remove any leading or trailing whitespace */

         if (*dfmName)
         {
            rc = code_fine;

            if (dty->deform == NULL)
            {
               dty->deform = (DeformObject**) simm_malloc(1 * sizeof(DeformObject*));
               dty->deform_name = (char**) simm_malloc(1 * sizeof(char*));
            }
            else
            {
               dty->deform = (DeformObject**) simm_realloc(dty->deform,
                                (dty->num_deforms + 1) * sizeof(DeformObject*), &rc);

               dty->deform_name = (char**) simm_realloc(dty->deform_name,
                                (dty->num_deforms + 1) * sizeof(char*), &rc);
            }

            if (dty->deform && dty->deform_name && rc == code_fine)
            {
               dty->deform[dty->num_deforms] = lookup_deform(ms, dfmName);
               
               if (dty->deform[dty->num_deforms])
               {
                  mstrcpy(&dty->deform_name[dty->num_deforms], dfmName);
                  dty->num_deforms++;
               }
               else
                  rc = code_bad;
            }

            if (rc != code_fine)
            {
	            if (dty->deform)
	               free(dty->deform);

	            if (dty->deform_name)
	               free(dty->deform_name);

	            dty->deform = NULL;
	            dty->num_deforms = 0;

	            sprintf(errorbuffer, "Error reading deform \'%s\' for deformity: %s.",
	               dfmName, dty->name);
	            error(recover,errorbuffer);

	            return code_bad;
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"enddeformity"))
         break;
   }

   /* Check the value and default_value to make sure they're in range.
    * This code will also initialize them to range.start if they were not
    * specified in the deformity definition.
    */
   if (dty->value < dty->range.start)
      dty->value = dty->range.start;
   else if (dty->value > dty->range.end)
      dty->value = dty->range.end;

   if (dty->default_value < dty->range.start)
      dty->default_value = dty->range.start;
   else if (dty->default_value > dty->range.end)
      dty->default_value = dty->range.end;

   ms->num_deformities++;

   return code_fine;

} /* read_deformity */


/* -------------------------------------------------------------------------
   _read_til - read (and ignore) characters from the specified file until
      the specified character 'c' is found.
---------------------------------------------------------------------------- */
SBoolean _read_til (FILE* file, int c)
{
   int t = fgetc(file);
   
   while (t != c && t != EOF)
       t = fgetc(file);
   
   return (SBoolean) (t == EOF);
}


/* -------------------------------------------------------------------------
   _read_til_tokens - Read characters from 'file' into 'buf' until one of the
     characters specified in 'delimiters' is encountered.  Put the delimiting
     character back into 'file'.
---------------------------------------------------------------------------- */
int _read_til_tokens (FILE* file, char* buf, const char* delimiters)
{
   char* p = buf;

   while (1)
   {
      char c = fgetc(file);

      if (feof(file) || ferror(file))
         break;
      else
      {
         if (strchr(delimiters, c))
         {
            ungetc(c, file);
            break;
         }
         else
            *p++ = c;
      }
   }
   *p = '\0';

   /* remove trailing whitespace if necessary */
   while (p > buf && isspace(p[-1]))
       *(--p) = '\0';

   /* Return 1 if the string is not empty. */
   if (p > buf)
      return 1;
   else
      return 0;
}

/* -------------------------------------------------------------------------
   _strip_outer_whitespace - 
---------------------------------------------------------------------------- */
void _strip_outer_whitespace (char* str)
{
   /* remove exterior (but not interior) whitespace from a string.
    */
   
   /* remove trailing whitespace */
   char* p = str + strlen(str) - 1;
   
   for ( ; p >= str && isspace(*p); p--)
      *p = '\0';
   
   /* remove leading whitespace */
   for (p = str; *p; p++)
      if ( ! isspace(*p))
         break;
   
   if (*p && p != str)
      memmove(str, p, strlen(p) + 1);
}

