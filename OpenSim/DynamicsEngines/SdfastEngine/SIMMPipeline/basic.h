/*******************************************************************************

   BASIC.H

   Author: Peter Loan

   Copyright (c) 1996-2004 MusculoGraphics, a division of Motion Analysis Corp.
   All rights reserved.

   Description: This file contains #defines and enums that are used by most
      of the Pipeline source files.

*******************************************************************************/

#ifndef BASIC_H
#define BASIC_H

/* integration parameters */
#define INTEGRATION_TOL 1e-5        /* integration tolerance, size of allowable error */
#define MIN_INTEGRATION_STEP 1e-12  /* minimum step size in integration, should generally */
                                    /* be at least 10 times smaller than INTEGRATION_TOL */
#define MIN_IMPACT_STEP 1e-6        /* min step size when searching for impacts */
#define TINY_TIME MIN_INTEGRATION_STEP * 0.5  /* round-off error for comparing times */

/* Return codes from integrate() */
#define INT_NO_ERROR        0
#define INT_OVER_BUMP       1
#define INT_CANT_CONTINUE   2
#define INT_USER_FLAG       3

/* Return codes from calc_derivatives() */
#define NO_FLAG         0
#define IMPACT_FLAG     1
#define BAD_STATE       2
#define CONS_VIOL       3

#define NO 0
#define YES 1
#if !defined M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
#define RAD2DEG(x) ((x) * 180.0/M_PI)
#define DEG2RAD(x) ((x) * M_PI/180.0)

#define STRUCT typedef struct
#define UNION typedef union
#define ENUM typedef enum

#define GROUND -1
#define TRUE 1
#define FALSE 0

#define XX 0
#define YY 1
#define ZZ 2
#define WW 3

#define PERR 0
#define VERR 1
#define AERR 2

#define SPACE ' '
#define TAB '\t'
#define CARRIAGE_RETURN '\n'
#define STRING_TERMINATOR '\0'

#define TIME -1

#define END_OF_ARRAY -1
#define TINY_NUMBER 0.0000001

#define MAX_USER_FUNCTIONS 10
#define SPLINE_ARRAY_INCREMENT 20
#define ARRAY_INCREMENT 10

#define MUSCLEBUFFER 96
#define SEGMENTBUFFER 32
#define GENBUFFER 64
#define GROUPBUFFER 96
#define CHARBUFFER 2048

#define MAXMDOUBLE 99999.99
#define MINMDOUBLE -99999.99
#define ERROR_DOUBLE -999999.3
#define DONT_CHECK_DOUBLE -999999.5
#define ERROR_INT -32760
#define ROUNDOFF_ERROR 0.0000000000002
#define MINIMUM_DIVISOR 0.0000001
#define DEG_TO_RAD 0.017453292519943
#define RAD_TO_DEG 57.295779513082323
#define DTOR DEG_TO_RAD
#define RTOD RAD_TO_DEG
#define GRAV_CONSTANT 9.80665
#define MAX_MATRIX_SIZE 500

#define ON_RAY 0
#define ABOVE_RAY 1
#define BELOW_RAY 2

#define E_INSIDE_RADIUS	  -1
#define E_NO_WRAP 0
#define E_WRAPPED 1
#define E_MANDATORY_WRAP 2

#ifdef WIN32
  #define DIR_SEP_CHAR   '\\'
  #define DIR_SEP_STRING "\\"
  
  #define COPY_COMMAND   "copy"
#else
  #define DIR_SEP_CHAR   '/'
  #define DIR_SEP_STRING "/"
  
  #define COPY_COMMAND   "cp"
#endif

/* wrapping algorithms
 */
#define WE_HYBRID_ALGORITHM    0   /* Frans + fan algorithm */
#define WE_MIDPOINT_ALGORITHM  1   /* midpoint algorithm    */
#define WE_AXIAL_ALGORITHM     2   /* Frans only algorithm  */
#define WE_NUM_WRAP_ALGORITHMS 3

#define WE_FAN_ALGORITHM       4   /* fan only algorithm    */

typedef double Quat[4];
typedef double DMatrix[4][4];

ENUM
{
   code_fine,                        /* fine, no error was encountered */
   code_bad                          /* bad, an error was encountered */
} ReturnCode;                        /* error condition values */


ENUM
{
   defining_element,                 /* defining an element's properties */
   declaring_element,                /* using the element name, not defining it */
   just_checking_element             /* just checking to see if already defined */
} EnterMode;                         /* modes when entering/defining model elements */


ENUM
{
   recover,                          /* recover from error */
   abort_action,                     /* abort action; critical error */
   exit_program,                     /* exit program; fatal error */
   none                              /* no action; no error */
} ErrorAction;                       /* error recovery actions */


ENUM
{
   zeroth = 0,                       /* zeroth derivative */
   first,                            /* first derivative */
   second                            /* second derivative */
} Derivative;                        /* function derivative values */


ENUM
{
   off,                              /* off */
   on                                /* on */
} OnOffSwitch;                       /* conventional on/off switch */


ENUM
{
   type_int,                         /* integer */
   type_double,                      /* double */
   type_char,                        /* single character */
   type_string                       /* string */
} VariableType;                      /* variable types that can be read from string */


ENUM
{
   old_ascii,
   new_ascii,
   binary,
   unknown,
   file_not_found
} FileType;


#endif /* BASIC_H */
