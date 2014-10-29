/* C Compatible Compiler Preprocessor (CCCP)
Copyright (C) 1986, 1987, 1989 Free Software Foundation, Inc.
                    Written by Paul Rubin, June 1986
            Adapted to ANSI C, Richard Stallman, Jan 1987

This program is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 1, or (at your option) any
later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 In other words, you are welcome to use, share and improve this program.
 You are forbidden to forbid anyone else to use, share and improve
 what you give them.   Help stamp out software-hoarding!  */


/*JPL*/
#define GCC_INCLUDE_DIR "/usr/include"
#define GPLUSPLUS_INCLUDE_DIR "/usr/include"

#ifdef sgi
/* do not compile with -signed!! Only use default unsigned char */
typedef char U_CHAR;
#else
typedef unsigned char U_CHAR;
#endif

#ifdef DWARF_DM
#include "dm.h"
#endif

#ifdef EMACS
#define NO_SHORTNAMES
#include "../src/config.h"
#ifdef open
#undef open
#undef read
#undef write
#endif /* open */
#endif /* EMACS */

#ifndef EMACS
#include "config.h"
#endif /* not EMACS */

/* In case config.h defines these.  */
#undef bcopy
#undef bzero
#undef bcmp

#include <sys/types.h>

#ifdef __MWERKS__
  /*#include <unistd.h>*/
  #include <stat.h>

  #define _OPEN(_FD, _MODE, _PERM) open(_FD, _MODE)
#else
  #include <sys/stat.h>

  #define _OPEN(_FD, _MODE, _PERM) open(_FD, _MODE, _PERM)
#endif

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <signal.h>

#ifndef WIN32
#include <stdarg.h>
#endif

#ifndef VMS
#ifdef WIN32
#include <fcntl.h>
#else
#include <sys/file.h>
#endif

#ifndef USG
#ifdef WIN32
#include <time.h>       /* for __DATE__ and __TIME__ */
#define index strchr
#define rindex strrchr
#else
#include <sys/time.h>       /* for __DATE__ and __TIME__ */
#include <sys/resource.h>
#endif
#else
#define index strchr
#define rindex strrchr
#include <time.h>
#include <fcntl.h>
#endif /* USG */
#endif /* not VMS */
  
/* VMS-specific definitions */
#ifdef VMS
#include <time.h>
#include <errno.h>      /* This defines "errno" properly */
#include <perror.h>     /* This defines sys_errlist/sys_nerr properly */
#define O_RDONLY    0   /* Open arg for Read/Only  */
#define O_WRONLY    1   /* Open arg for Write/Only */
#define read(fd,buf,size)   VAX11_C_read(fd,buf,size)
#define write(fd,buf,size)  VAX11_C_write(fd,buf,size)
#ifdef __GNUC__
#define BSTRING         /* VMS/GCC supplies the bstring routines */
#endif /* __GNUC__ */
#endif /* VMS */

#ifdef __linux__
#include <time.h>
#ifdef __GNUC__
#define BSTRING         /* linux/GCC supplies the bstring routines */
#endif /* __GNUC__ */
#endif /* linux */

#ifdef WIN32
  #include <errno.h>
  #include <assert.h>


  #ifdef __MWERKS__
    #define fileno(F) ((F)->handle)
  #endif
  
  #ifndef F_DUPFD
    #define F_DUPFD 0x0
  #endif
#endif

#ifdef max
#undef max
#endif

#define max(a,b) ((a) > (b) ? (a) : (b))

/* External declarations.  */

void bcopy (), bzero ();
int bcmp ();
extern char *getenv ();
extern char *version_string;
extern int inside_math;

/* Forward declarations.  */

struct directive;
struct file_buf;
struct arglist;
struct argdata;

int do_define (), do_undef (), do_error (),
  do_pragma (), do_if (), do_xifdef (),
  do_endif (), do_sccs (), do_ident(), do_once (),
  do_include(), finclude(), do_line(), do_elif(), do_else();

struct hashnode *install ();
struct hashnode *lookup ();

char *xmalloc (), *xrealloc (), *xcalloc (), *savestring ();
void fatal (), pfatal_with_name (), perror_with_name ();

void macroexpand ();
void dump_all_macros ();
void conditional_skip ();
void skip_if_group ();
void output_line_command ();
/* Last arg to output_line_command.  */
enum file_change_code {same_file, enter_file, leave_file};

int grow_outbuf ();
int handle_directive ();
int handle_c_expression ();
double parse_c_math_expression ();
void memory_full ();

U_CHAR *macarg1 ();
char *macarg ();

U_CHAR *skip_to_end_of_comment ();
U_CHAR *skip_quoted_string ();

#ifndef FATAL_EXIT_CODE
#define FATAL_EXIT_CODE 33  /* gnu cc command understands this */
#endif

#ifndef SUCCESS_EXIT_CODE
#define SUCCESS_EXIT_CODE 0 /* 0 means success on Unix.  */
#endif

/* Name under which this program was invoked.  */

char *acpp_progname;

/* Nonzero means handle C++ comment syntax and use
   extra default include directories for C++.  */

int cplusplus;

/* Current maximum length of directory names in the search path
   for include files.  (Altered as we get more of them.)  */

int max_include_len;

/* Nonzero means copy comments into the output file.  */

int put_out_comments = 0;

/* Nonzero means turn backslash-newline into newline when outputting
   defines, but don't consider them to end the macro definition. */
int no_newline_splicing = 0;

/* Nonzero means don't process the ANSI trigraph sequences.  */

int no_trigraphs = 0;

/* Nonzero means print the names of included files rather than
   the preprocessed output.  1 means just the #include "...",
   2 means #include <...> as well.  */

int print_deps = 0;
#ifdef sgi
/* deps_prefix is non-null if we wish to output dependencies in traditional
   cpp format. It is a pointer to the prefix string.
   deps_file is a handle for -MD's argument, which names the make dependency
   file to update.  */
#include <string.h>
#include "make_depend.h"
char *deps_prefix;
MDhandle deps_file;
#endif

/* Nonzero means don't output line number information.  */

int no_line_commands;

/* Nonzero means inhibit output of the preprocessed text
   and instead output the definitions of all user-defined macros
   in a form suitable for use as input to cccp.  */

int dump_macros;

/* Nonzero means give all the error messages the ANSI standard requires.  */
#ifdef sgi
  /* value > 1 means add warnings about #ident, #sccs, which are in
     our headers, so we do not want to warn too much....
  */

#endif

int pedantic;
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
/* value of > 0 means turn warnings off */
int warnings_off;
/*
    If showdefines is non-zero, put out extra stuff to show where
        the define is coming from.
    For -showdefines option for debugging include files.
*/
int showdefines;
#endif

/* Nonzero means warn if slash-star appears in a comment.  */

int warn_comments;

/* Nonzero means warn if there are any trigraphs.  */

int warn_trigraphs;

/* Nonzero means try to imitate old fashioned non-ANSI preprocessor.  */

int traditional;

/* Nonzero causes output not to be done,
   but directives such as #define that have side effects
   are still obeyed.  */

int no_output;

/* I/O buffer structure.
   The `fname' field is nonzero for source files and #include files
   and for the dummy text used for -D and -U.
   It is zero for rescanning results of macro expansion
   and for expanding macro arguments.  */
#define INPUT_STACK_MAX 200
struct file_buf {
  char *fname;
  int lineno;
  int length;
#ifdef sgi
  int endif_cruft_message_issued;
#endif
  U_CHAR *buf;
  U_CHAR *bufp;
  /* Macro that this level is the expansion of.
     Included so that we can reenable the macro
     at the end of this level.  */
  struct hashnode *macro;
  /* Value of if_stack at start of this file.
     Used to prohibit unmatched #endif (etc) in an include file.  */
  struct if_stack *if_stack;
  /* Object to be freed at end of input at this level.  */
  U_CHAR *free_ptr;
} instack[INPUT_STACK_MAX];

#ifdef sgi
#define ONE_TIME_ENDIF(s) {\
    if(instack[indepth].endif_cruft_message_issued == 0) {\
        s;\
    } instack[indepth].endif_cruft_message_issued += 1; }
    
#endif

/* Current nesting level of input sources.
   `instack[indepth]' is the level currently being read.  */
int indepth = -1;
#define CHECK_DEPTH(code) \
  if (indepth >= (INPUT_STACK_MAX - 1))                 \
    {                                   \
      error_with_line (line_for_error (instack[indepth].lineno),    \
               "macro or #include recursion too deep");     \
      code;                             \
    }

/* Current depth in #include directives that use <...>.  */
int system_include_depth = 0;

typedef struct file_buf FILE_BUF;

/* The output buffer.  Its LENGTH field is the amount of room allocated
   for the buffer, not the number of chars actually present.  To get
   that, subtract outbuf.buf from outbuf.bufp. */

#define OUTBUF_SIZE 10  /* initial size of output buffer */
FILE_BUF outbuf;

/* Grow output buffer OBUF points at
   so it can hold at least NEEDED more chars.  */

#define check_expand(OBUF, NEEDED)  \
  (((OBUF)->length - ((OBUF)->bufp - (OBUF)->buf) <= (NEEDED))   \
   ? grow_outbuf ((OBUF), (NEEDED)) : 0)

struct file_name_list
  {
    struct file_name_list *next;
    char *fname;
  };

/* #include "file" looks in source file dir, then stack. */
/* #include <file> just looks in the stack. */
/* -I directories are added to the end, then the defaults are added. */
struct file_name_list include_defaults[] =
  {
#ifndef VMS
    { &include_defaults[1], GCC_INCLUDE_DIR },
    { &include_defaults[2], "/usr/include" },
    { 0, "/usr/local/include" }
#else
    { &include_defaults[1], "GNU_CC_INCLUDE:" },       /* GNU includes */
    { &include_defaults[2], "SYS$SYSROOT:[SYSLIB.]" }, /* VAX-11 "C" includes */
    { 0, "" },  /* This makes normal VMS filespecs work OK */
#endif /* VMS */
  };

/* These are used instead of the above, for C++.  */
struct file_name_list cplusplus_include_defaults[] =
  {
#ifndef VMS
    /* Pick up GNU C++ specific include files.  */
    { &cplusplus_include_defaults[1], GPLUSPLUS_INCLUDE_DIR },
    /* Use GNU CC specific header files.  */
    { &cplusplus_include_defaults[2], GCC_INCLUDE_DIR },
    { 0, "/usr/include" }
#else
    { &cplusplus_include_defaults[1], "GNU_CC_INCLUDE:" },
    /* VAX-11 C includes */
    { &cplusplus_include_defaults[2], "SYS$SYSROOT:[SYSLIB.]" },
    { 0, "" },  /* This makes normal VMS filespecs work OK */
#endif /* VMS */
  };

struct file_name_list *include = 0; /* First dir to search */
    /* First dir to search for <file> */
struct file_name_list *first_bracket_include = 0;
struct file_name_list *last_include = 0;    /* Last in chain */

/* List of included files that contained #once.  */
struct file_name_list *dont_repeat_files = 0;

/* List of other included files.  */
struct file_name_list *all_include_files = 0;

/* Structure allocated for every #define.  For a simple replacement
   such as
    #define foo bar ,
   nargs = -1, the `pattern' list is null, and the expansion is just
   the replacement text.  Nargs = 0 means a functionlike macro with no args,
   e.g.,
       #define getchar() getc (stdin) .
   When there are args, the expansion is the replacement text with the
   args squashed out, and the reflist is a list describing how to
   build the output from the input: e.g., "3 chars, then the 1st arg,
   then 9 chars, then the 3rd arg, then 0 chars, then the 2nd arg".
   The chars here come from the expansion.  Whatever is left of the
   expansion after the last arg-occurrence is copied after that arg.
   Note that the reflist can be arbitrarily long---
   its length depends on the number of times the arguments appear in
   the replacement text, not how many args there are.  Example:
   #define f(x) x+x+x+x+x+x+x would have replacement text "++++++" and
   pattern list
     { (0, 1), (1, 1), (1, 1), ..., (1, 1), NULL }
   where (x, y) means (nchars, argno). */

typedef struct definition DEFINITION;
struct definition {
  int nargs;
  int length;           /* length of expansion string */
  U_CHAR *expansion;
  struct reflist {
    struct reflist *next;
    char stringify;     /* nonzero if this arg was preceded by a
                   # operator. */
    char raw_before;        /* Nonzero if a ## operator before arg. */
    char raw_after;     /* Nonzero if a ## operator after arg. */
    int nchars;         /* Number of literal chars to copy before
                   this arg occurrence.  */
    int argno;          /* Number of arg to substitute (origin-0) */
  } *pattern;
  /* Names of macro args, concatenated in reverse order
     with comma-space between them.
     The only use of this is that we warn on redefinition
     if this differs between the old and new definitions.  */
  U_CHAR *argnames;
};

/* different kinds of things that can appear in the value field
   of a hash node.  Actually, this may be useless now. */
union hashval {
  int ival;
  char *cpval;
  DEFINITION *defn;
};


/* The structure of a node in the hash table.  The hash table
   has entries for all tokens defined by #define commands (type T_MACRO),
   plus some special tokens like __LINE__ (these each have their own
   type, and the appropriate code is run when that type of node is seen.
   It does not contain control words like "#define", which are recognized
   by a separate piece of code. */

/* different flavors of hash nodes --- also used in keyword table */
enum node_type {
 T_DEFINE = 1,  /* the `#define' keyword */
 T_INCLUDE, /* the `#include' keyword */
 T_IFDEF,   /* the `#ifdef' keyword */
 T_IFNDEF,  /* the `#ifndef' keyword */
 T_IF,      /* the `#if' keyword */
 T_ELSE,    /* `#else' */
 T_PRAGMA,  /* `#pragma' */
 T_ELIF,    /* `#else' */
 T_UNDEF,   /* `#undef' */
 T_LINE,    /* `#line' */
 T_ERROR,   /* `#error' */
 T_ENDIF,   /* `#endif' */
 T_SCCS,    /* `#sccs', used on system V.  */
 T_IDENT,   /* `#ident', used on system V.  */
 T_SPECLINE,    /* special symbol `__LINE__' */
 T_DATE,    /* `__DATE__' */
 T_FILE,    /* `__FILE__' */
 T_BASE_FILE,   /* `__BASE_FILE__' */
 T_INCLUDE_LEVEL, /* `__INCLUDE_LEVEL__' */
 T_VERSION, /* `__VERSION__' */
 T_TIME,    /* `__TIME__' */
 T_CONST,   /* Constant value, used by `__STDC__' */
 T_MACRO,   /* macro defined by `#define' */
 T_DISABLED,    /* macro temporarily turned off for rescan */
 T_SPEC_DEFINED, /* special `defined' macro for use in #if statements */
 T_UNUSED   /* Used for something not defined.  */
 };

struct hashnode {
  struct hashnode *next;    /* double links for easy deletion */
  struct hashnode *prev;
  struct hashnode **bucket_hdr; /* also, a back pointer to this node's hash
                   chain is kept, in case the node is the head
                   of the chain and gets deleted. */
  enum node_type type;      /* type of special token */
  int length;           /* length of token, for quick comparison */
  U_CHAR *name;         /* the actual name */
  union hashval value;      /* pointer to expansion, or whatever */
};

typedef struct hashnode HASHNODE;

/* Some definitions for the hash table.  The hash function MUST be
   computed as shown in hashf () below.  That is because the rescan
   loop computes the hash value `on the fly' for most tokens,
   in order to avoid the overhead of a lot of procedure calls to
   the hashf () function.  Hashf () only exists for the sake of
   politeness, for use when speed isn't so important. */

#define HASHSIZE 1403
HASHNODE *hashtab[HASHSIZE];
#define HASHSTEP(old, c) ((old << 2) + c)
#define MAKE_POS(v) (v & ~0x80000000) /* make number positive */

/* Symbols to predefine.  */

#ifdef CPP_PREDEFINES
char *predefs = CPP_PREDEFINES;
#else
char *predefs = "";
#endif

/* `struct directive' defines one #-directive, including how to handle it.  */

struct directive {
  int length;           /* Length of name */
  int (*func)();        /* Function to handle directive */
  char *name;           /* Name of directive */
  enum node_type type;      /* Code which describes which directive. */
  char angle_brackets;      /* Nonzero => <...> is special.  */
  char traditional_comments;    /* Nonzero: keep comments if -traditional.  */
  char pass_thru;       /* Copy preprocessed directive to output file.  */
};

/* Here is the actual list of #-directives, most-often-used first.  */

struct directive directive_table[] = {
  {  6, do_define, "define", T_DEFINE, 0, 1},
  {  2, do_if, "if", T_IF},
  {  5, do_xifdef, "ifdef", T_IFDEF},
  {  6, do_xifdef, "ifndef", T_IFNDEF},
  {  5, do_endif, "endif", T_ENDIF},
  {  4, do_else, "else", T_ELSE},
  {  4, do_elif, "elif", T_ELIF},
  {  4, do_line, "line", T_LINE},
  {  7, do_include, "include", T_INCLUDE, 1},
  {  5, do_undef, "undef", T_UNDEF},
  {  5, do_error, "error", T_ERROR},
#ifdef SCCS_DIRECTIVE
  {  4, do_sccs, "sccs", T_SCCS},
  {  5, do_ident, "ident", T_IDENT},
#endif
  {  6, do_pragma, "pragma", T_PRAGMA, 0, 0, 1},
  {  -1, 0, "", T_UNUSED},
};

/* table to tell if char can be part of a C identifier. */
U_CHAR is_idchar[256];
/* table to tell if char can be first char of a c identifier. */
U_CHAR is_idstart[256];
/* table to tell if c is horizontal space.  */
extern U_CHAR is_hor_space[256];
/* table to tell if c is horizontal or vertical space.  */
U_CHAR is_space[256];

#define SKIP_WHITE_SPACE(p) do { while (is_hor_space[*p]) p++; } while (0)
#define SKIP_ALL_WHITE_SPACE(p) do { while (is_space[*p]) p++; } while (0)
  
int errors = 0;         /* Error counter for exit code */

/* Zero means dollar signs are punctuation.
   -$ stores 0; -traditional, stores 1.  Default is 1 for VMS, 0 otherwise.
   This must be 0 for correct processing of this ANSI C program:
    #define foo(a) #a
    #define lose(b) foo(b)
    #define test$
    lose(test)  */
#ifndef DOLLARS_IN_IDENTIFIERS
#define DOLLARS_IN_IDENTIFIERS 0
#endif
int dollars_in_ident = DOLLARS_IN_IDENTIFIERS;

FILE_BUF expand_to_temp_buffer ();

DEFINITION *collect_expansion ();

/* Stack of conditionals currently in progress
   (including both successful and failing conditionals).  */

struct if_stack {
  struct if_stack *next;    /* for chaining to the next stack frame */
  char *fname;      /* copied from input when frame is made */
  int lineno;           /* similarly */
  int if_succeeded;     /* true if a leg of this if-group
                    has been passed through rescan */
  enum node_type type;      /* type of last directive seen in this group */
};
typedef struct if_stack IF_STACK_FRAME;
IF_STACK_FRAME *if_stack = NULL;

/* Buffer of -M output.  */

char *deps_buffer;

/* Number of bytes allocated in above.  */
int deps_allocated_size;

/* Number of bytes used.  */
int deps_size;

/* Number of bytes since the last newline.  */
int deps_column;

/* Nonzero means -I- has been seen,
   so don't look for #include "foo" the source-file directory.  */
int ignore_srcdir;

/* Handler for SIGPIPE.  */

static void
pipe_closed ()
{
  fatal ("output pipe has been closed");
}

#ifdef sgi
/* This is for output on names like b.p or a.c++ etc etc 
   Generates the same sort of output as cpp does 
   for the dependency (.o) name.  16 is too much extra, but
   we do need room for the .o (sometimes) and the :space ...
*/
char *
make_deps_target(char *p)
{
  char *q, *target, *dot, *slash;
  int len;

  /* discard all directory prefixes from P */
  for (q = p; *q; q++) {
    if (*q == '/' && q[1])
      p = q + 1;
  }
  len = q - p;
  target = (char *)xmalloc(len + 16);
  strcpy(target,p);
  dot = strrchr(target,'.');
  slash = strrchr(target,'/');
  if (dot && ((slash == 0) || (dot > slash)) && dot <= target+len-2) {
    dot[1] = 'o';
    dot[2] = '\0';
  } else {
    /* cpp says -missing component name- rather than doing this */
    strcat(target,".o");
  }
  return target;
}
#endif


/* -------------------------------------------------------------------------
   PARAMETERIZED USER MESSAGES - allows SIMM to display acpp message in the
      SIMM message window.
---------------------------------------------------------------------------- */
static int default_message_proc (const char* format, ...)
{

   va_list ap;
   int n;
   
   va_start(ap, format);
   n = vfprintf(stderr, format, ap);
   va_end(ap);

   return n;

}

typedef int (*_msg_proc)(const char* format, ...);

static _msg_proc sMessageProc = default_message_proc;

#define USER_MESSAGE sMessageProc

/* -------------------------------------------------------------------------
   reset_acpp - this routine was added to allow accp_main() to be called
      multiple times during a program's execution without residual effect
      from one call to the next.  This routine resets acpp's state.  -- KMS 8/24/99
---------------------------------------------------------------------------- */
static void reset_acpp ()
{
   int i, n = 0;
   
   for (i = 0; i < HASHSIZE; i++)
   {
      if (hashtab[i])
      {
         HASHNODE* node = hashtab[i];
         
         while (node)
         {
            HASHNODE* next = node->next;
            
            free(node);
            
            /* NOTE:  If we want to be ambitious here we can also determine
             *  when to free the hash node's "value" (i.e. cpval, or defn).
             *  This seemed tricky however, and I decided that a small
             *  memory leak was better than a possible invalid free().
             *  -- KMS 8/24/99
             */
            
            node = next;
            
            n++;
         }
         hashtab[i] = NULL;
      }
   }
#if 0
   if (n > 0)
      USER_MESSAGE("Freed %d hash nodes in acpp.\n", n);
#endif
}


FILE* fp_out = NULL;


int acpp_main(int argc, char** argv, _msg_proc msg)
{
  int st_mode;
  long st_size;
  char *in_fname, *out_fname;
  int f, i;
  FILE_BUF *fp;
  char **pend_files = (char **) xmalloc (argc * sizeof (char *));
  char **pend_defs = (char **) xmalloc (argc * sizeof (char *));
  char **pend_undefs = (char **) xmalloc (argc * sizeof (char *));
  int inhibit_predefs = 0;
  int no_standard_includes = 0;

  /* Non-0 means don't output the preprocessed program.  */
  int inhibit_output = 0;

  /* Stream on which to print the dependency information.  */
  FILE *deps_stream = 0;
  /* Target-name to write with the dependency information.  */
  char *deps_target = 0;
#ifdef sgi
  /* name of the make-dependencies file to update */
  char *deps_update = 0;
#endif

  /* KMS: allow caller to specify message procedure */
  
  sMessageProc = msg ? msg : default_message_proc;
  
  /* JPL: re-initialize global variables */

  include = 0;
  first_bracket_include = 0;
  last_include = 0;
  errors = 0;
  indepth = -1;
  system_include_depth = 0;
  if_stack = NULL;
  
  /* KMS: empty residual hash-table entries from previous call to acpp_main() */
  
  reset_acpp();

#ifdef RLIMIT_STACK
  /* Get rid of any avoidable limit on stack size.  */
  {
    struct rlimit rlim;

    /* Set the stack limit huge so that alloca (particularly stringtab
     * in dbxread.c) does not fail. */
    getrlimit (RLIMIT_STACK, &rlim);
    rlim.rlim_cur = rlim.rlim_max;
    setrlimit (RLIMIT_STACK, &rlim);
  }
#endif /* RLIMIT_STACK defined */

  acpp_progname = argv[0];
  in_fname = NULL;
  out_fname = NULL;

  /* Initialize is_idchar to allow $.  */
  dollars_in_ident = 1;
  initialize_char_syntax ();
  dollars_in_ident = DOLLARS_IN_IDENTIFIERS;

  no_line_commands = 0;
  no_trigraphs = 1;
  dump_macros = 0;
  no_output = 0;
  cplusplus = 0;
#ifdef CPLUSPLUS
  cplusplus = 1;
#endif

#ifndef WIN32
  signal (SIGPIPE, pipe_closed);
#endif

#ifndef VMS
  max_include_len
    = max (max (sizeof (GCC_INCLUDE_DIR),
        sizeof (GPLUSPLUS_INCLUDE_DIR)),
       sizeof ("/usr/include/CC"));
#else /* VMS */
  max_include_len
    = sizeof("SYS$SYSROOT:[SYSLIB.]");
#endif /* VMS */

  bzero (pend_files, argc * sizeof (char *));
  bzero (pend_defs, argc * sizeof (char *));
  bzero (pend_undefs, argc * sizeof (char *));

  /* Process switches and find input file name.  */

  for (i = 1; i < argc; i++) {
    if (argv[i][0] != '-') {
      if (out_fname != NULL)
    fatal ("Usage: %s [switches] input output\n", argv[0]);
      else if (in_fname != NULL)
    out_fname = argv[i];
      else
    in_fname = argv[i];
    } else {
      switch (argv[i][1]) {

      case 'i':
    if (argv[i][2] != 0)
      pend_files[i] = argv[i] + 2;
    else
      pend_files[i] = argv[i+1], i++;
    break;

      case 'o':
    if (out_fname != NULL)
      fatal ("Output filename specified twice\n");
    out_fname = argv[++i];
    if (!strcmp (out_fname, "-"))
      out_fname = "";
    break;

      case 'p':
#ifdef sgi
    pedantic += 1;
#else
    pedantic = 1;
#endif
    break;

      case 't':
    if (!strcmp (argv[i], "-traditional")) {
      traditional = 1;
      dollars_in_ident = 1;
    } else if (!strcmp (argv[i], "-trigraphs")) {
      no_trigraphs = 0;
    }
    break;

#ifdef sgi
      case 'B':
#endif
      case '+':
    cplusplus = 1;
    break;

      case 'W':
    if (!strcmp (argv[i], "-Wtrigraphs")) {
      warn_trigraphs = 1;
    }
    if (!strcmp (argv[i], "-Wcomments"))
      warn_comments = 1;
    if (!strcmp (argv[i], "-Wcomment"))
      warn_comments = 1;
    if (!strcmp (argv[i], "-Wall")) {
      warn_trigraphs = 1;
      warn_comments = 1;
    }
    break;

      case 'M':
#ifdef sgi
    if (argv[i][2] == 'D') {
      char **cpp;
      if (!strcmp(&argv[i][3], "target"))
        cpp = &deps_target;
      else if (!strcmp(&argv[i][3], "update"))
        cpp = &deps_update;
      else
        goto default_illegal_switch;
      if (++i == argc)
        fatal("%s option requires an argument", argv[i-1]);
      *cpp = argv[i];
      break;
    }
#endif
    if (!strcmp (argv[i], "-M"))
      print_deps = 2;
    else if (!strcmp (argv[i], "-MM"))
      print_deps = 1;
#ifdef sgi
    else
      goto default_illegal_switch;
#endif
    inhibit_output = 1;
    break;

      case 'd':
    dump_macros = 1;
    no_output = 1;
    break;

      case 'v':
    USER_MESSAGE("GNU CPP version %s\n", version_string);
    break;

      case 'D':
    {
      char *p, *p1;

      if (argv[i][2] != 0)
        p = argv[i] + 2;
      else
        p = argv[++i];

      if ((p1 = (char *) index (p, '=')) != NULL)
        *p1 = ' ';
      pend_defs[i] = p;
    }
    break;

      case 'U':     /* JF #undef something */
    if (argv[i][2] != 0)
      pend_undefs[i] = argv[i] + 2;
    else
      pend_undefs[i] = argv[i+1], i++;
    break;

      case 'C':
    put_out_comments = 1;
    break;

      case 'E':         /* -E comes from cc -E; ignore it.  */
    break;

      case 'P':
    no_line_commands = 1;
    break;

      case '$':         /* Don't include $ in identifiers.  */
#ifdef sgi
    /* we disallow by default, but allow with -$ */
    dollars_in_ident = 1;
#else
    dollars_in_ident = 0;
#endif
    break;

      case 'I':         /* Add directory to path for includes.  */
    {
      struct file_name_list *dirtmp;

      if (argv[i][2] == 0 && argv[i+1][0] == '-') {
        /* handle SGI brain-damaged -I hackery; -I with
         * no arg means -nostdinc. YUCK */
        no_standard_includes = 1;
        break;
      }

      if (! ignore_srcdir && !strcmp (argv[i] + 2, "-"))
      {
         ignore_srcdir = 1;
      }
      else
      {
         dirtmp = (struct file_name_list *)
        xmalloc (sizeof (struct file_name_list));
         dirtmp->next = 0;      /* New one goes on the end */
         if (include == 0)
         {
        include = dirtmp;
        last_include = dirtmp;
         }
         else
         {
        last_include->next = dirtmp;
         }
         last_include = dirtmp; /* Tail follows the last one */
         if (argv[i][2] != 0)
        dirtmp->fname = argv[i] + 2;
         else
        dirtmp->fname = argv[++i];
         if (strlen (dirtmp->fname) > max_include_len)
        max_include_len = strlen (dirtmp->fname);
         if (ignore_srcdir && first_bracket_include == 0)
        first_bracket_include = dirtmp;
      }
       }
    break;
#ifdef sgi
      case 'w':
    warnings_off = 1;
    break;
      case 's':
    if(strcmp(argv[i],"-showdefines") == 0) {
        showdefines++;
    } else {
        goto default_illegal_switch;
    }
    break;
#endif

      case 'n':
#if sgi
    if (!strcmp (argv[i], "-nosplice")) {
      no_newline_splicing = 1;
      break;
    }
#endif
    /* -nostdinc causes no default include directories.
       You must specify all include-file directories with -I.  */
    no_standard_includes = 1;
    break;

      case 'u':
    /* Sun compiler passes undocumented switch "-undef".
       Let's assume it means to inhibit the predefined symbols.  */
    inhibit_predefs = 1;
    break;

      case '\0': /* JF handle '-' as file name meaning stdin or stdout */
    if (in_fname == NULL) {
      in_fname = "";
      break;
    } else if (out_fname == NULL) {
      out_fname = "";
      break;
    }   /* else fall through into error */

      default_illegal_switch: ; /* fall thru */
      default:
    fatal ("Invalid option `%s'\n", argv[i]);
      }
    }
  }

  /* Now that dollars_in_ident is known, initialize is_idchar.  */
  initialize_char_syntax ();

#ifdef sgi
  /* Initialize output buffer */
  outbuf.buf = (U_CHAR *) xmalloc (OUTBUF_SIZE);
  outbuf.bufp = outbuf.buf;
  outbuf.length = OUTBUF_SIZE;
#endif
  /* Install __LINE__, etc.  Must follow initialize_char_syntax
     and option processing.  */
#ifdef DWARF_DM
    dm_push_new(in_fname,0);
#endif
  initialize_builtins ();

  /* Do standard #defines that identify processor type.  */

#ifdef DWARF_DM
  dm_push_new(DWARF_INIT_FNAME,1);
#endif
  if (!inhibit_predefs) {
    char *p = (char *) alloca (strlen (predefs) + 1);
    strcpy (p, predefs);
    while (*p) {
      char *q;
      if (p[0] != '-' || p[1] != 'D')
    abort ();
      q = &p[2];
      while (*p && *p != ' ') p++;
      if (*p != 0)
    *p++= 0;
      make_definition (q);
    }
  }
  /* Do defines specified with -D.  */
  for (i = 1; i < argc; i++)
    if (pend_defs[i])
      make_definition (pend_defs[i]);

  /* Do undefines specified with -U.  */
  for (i = 1; i < argc; i++)
    if (pend_undefs[i])
      make_undef (pend_undefs[i]);
#ifdef DWARF_DM
  dm_finish_current();
#endif

  /* Unless -fnostdinc,
     tack on the standard include file dirs to the specified list */
  if (!no_standard_includes)
  {
     if (include == 0)
     {
    include = (cplusplus ? cplusplus_include_defaults : include_defaults);
     }
     else
     {
    last_include->next
       = (cplusplus ? cplusplus_include_defaults : include_defaults);
     }
     /* Make sure the list for #include <...> also has the standard dirs.  */
     if (ignore_srcdir && first_bracket_include == 0)
     {
    first_bracket_include
       = (cplusplus ? cplusplus_include_defaults : include_defaults);
     }
  }

#ifndef sgi
  /* Initialize output buffer */

  outbuf.buf = (U_CHAR *) xmalloc (OUTBUF_SIZE);
  outbuf.bufp = outbuf.buf;
  outbuf.length = OUTBUF_SIZE;
#endif

  /* Scan the -i files before the main input.
     Much like #including them, but with no_output set
     so that only their macro definitions matter.  */

  no_output++;
  for (i = 1; i < argc; i++)
    if (pend_files[i]) {
      int fd = _OPEN (pend_files[i], O_RDONLY, 0666);
      if (fd < 0) {
    perror_with_name (pend_files[i]);
    reset_acpp();
    return FATAL_EXIT_CODE;
      }
      finclude (fd, pend_files[i], &outbuf);
    }
  no_output--;

  /* Create an input stack level for the main input file
     and copy the entire contents of the file into it.  */

  fp = &instack[++indepth];

  /* JF check for stdin */
  if (in_fname == NULL || *in_fname == 0) {
    in_fname = "";
    f = 0;
  } else if ((f = _OPEN (in_fname, O_RDONLY, 0666)) < 0)
    goto perror;

  /* Either of two environment variables can specify output of deps.
     Its value is either "OUTPUT_FILE" or "OUTPUT_FILE DEPS_TARGET",
     where OUTPUT_FILE is the file to write deps info to
     and DEPS_TARGET is the target to mention in the deps.  */

  if (print_deps == 0
      && (getenv ("SUNPRO_DEPENDENCIES") != 0
      || getenv ("DEPENDENCIES_OUTPUT") != 0))
    {
      char *spec = getenv ("DEPENDENCIES_OUTPUT");
      char *s;
      char *output_file;

      if (spec == 0)
    {
      spec = getenv ("SUNPRO_DEPENDENCIES");
      print_deps = 2;
    }
      else
    print_deps = 1;

      s = spec;
      /* Find the space before the DEPS_TARGET, if there is one.  */
      /* Don't use `index'; that causes trouble on USG.  */
      while (*s != 0 && *s != ' ') s++;
      if (*s != 0)
    {
      deps_target = s + 1;
      output_file = (char *) xmalloc (s - spec + 1);
      bcopy (spec, output_file, s - spec);
      output_file[s - spec] = 0;
    }
      else
    {
      deps_target = 0;
      output_file = spec;
    }
      
      deps_stream = fopen (output_file, "a");
      if (deps_stream == 0)
    pfatal_with_name (output_file);
    }
  /* If the -M option was used, output the deps to standard output.  */
  else if (print_deps)
    deps_stream = stdout;

  /* For -M, print the expected object file name
     as the target of this Make-rule.  */
  if (print_deps) {
    deps_allocated_size = 200;
    deps_buffer = (char *) xmalloc (deps_allocated_size);
    deps_buffer[0] = 0;
    deps_size = 0;
    deps_column = 0;

    if (deps_target) {
      deps_output (deps_target, 0);
      deps_output (":", 0);
    } else if (*in_fname == 0)
      deps_output ("-: ", 0);
    else {
#ifdef sgi
      deps_prefix = make_deps_target(in_fname);
      strcat(deps_prefix,": ");
      deps_output (deps_prefix, 0);
#else
      int len;
      char *p = in_fname;
      char *p1 = p;
      /* Discard all directory prefixes from P.  */
      while (*p1) {
    if (*p1 == '/')
      p = p1 + 1;
    p1++;
      }
      /* Output P, but remove known suffixes.  */
      len = strlen (p);
      if (p[len - 2] == '.' && p[len - 1] == 'c') {
    deps_output (p, len - 2);
      } else if (p[len - 3] == '.'
           && p[len - 2] == 'c'
           && p[len - 1] == 'c') {
    deps_output (p, len - 3);
      } else {
    deps_output (p, 0);
      }
      /* Supply our own suffix.  */
      deps_output (".o : ", 0);
#endif
      deps_output (in_fname, 0);
      deps_output (" ", 0);
    }
  }
#ifdef sgi
  if (deps_update) {
    if (*in_fname == '\0')
      fatal("No input filename specified with -MDupdate option");
    if (deps_target == NULL)
      deps_target = make_deps_target(in_fname);
    printf("*************No MDopen************\n");
/*    deps_file = MDopen("cpp", deps_update, deps_target, fatal);*/
  }
#endif

  file_size_and_mode (f, &st_mode, &st_size);
  fp->fname = in_fname;
  fp->lineno = 1;
  /* JF all this is mine about reading pipes and ttys */
  if ((st_mode & S_IFMT) != S_IFREG) {
    /* Read input from a file that is not a normal disk file.
       We cannot preallocate a buffer with the correct size,
       so we must read in the file a piece at the time and make it bigger.  */
    int size;
    int bsize;
    int cnt;
    U_CHAR *bufp;

    bsize = 2000;
    size = 0;
    fp->buf = (U_CHAR *) xmalloc (bsize + 2);
    bufp = fp->buf;
    for (;;) {
      cnt = read (f, bufp, bsize - size);
      if (cnt < 0) goto perror; /* error! */
      if (cnt == 0) break;  /* End of file */
      size += cnt;
      bufp += cnt;
      if (bsize == size) {  /* Buffer is full! */
        bsize *= 2;
        fp->buf = (U_CHAR *) xrealloc (fp->buf, bsize + 2);
    bufp = fp->buf + size;  /* May have moved */
      }
    }
    fp->length = size;
  } else {
    /* Read a file whose size we can determine in advance.
       For the sake of VMS, st_size is just an upper bound.  */
    long i;
    fp->length = 0;
    fp->buf = (U_CHAR *) xmalloc (st_size + 2);

    while (st_size > 0) {
      i = read (f, fp->buf + fp->length, st_size);
      if (i <= 0) {
        if (i == 0) break;
    goto perror;
      }
      fp->length += i;
      st_size -= i;
    }
  }
  fp->bufp = fp->buf;
  fp->if_stack = if_stack;
  
  /* Unless inhibited, convert trigraphs in the input.  */

  if (!no_trigraphs)
    trigraph_pcp (fp);

  /* Make sure data ends with a newline.  And put a null after it.  */

  if (fp->length > 0 && fp->buf[fp->length-1] != '\n')
    fp->buf[fp->length++] = '\n';
  fp->buf[fp->length] = '\0';

  /* Now that we know the input file is valid, open the output.  */
  if ( ! out_fname || ! strcmp (out_fname, ""))
    out_fname = "stdout";
  else if ( ! (fp_out = fopen (out_fname, "w")))
    pfatal_with_name (out_fname);

  output_line_command (fp, &outbuf, 0, same_file);

  /* Scan the input, processing macros and directives.  */

  rescan (&outbuf, 0);

  /* Now we have processed the entire input
     Write whichever kind of output has been requested.  */

  if (dump_macros)
    dump_all_macros ();
  else if (! inhibit_output && deps_stream != stdout) {
    if (write (fileno (fp_out), outbuf.buf, outbuf.bufp - outbuf.buf) < 0)
      fatal ("I/O error on output");
  }

  if (print_deps) {
    fputs (deps_buffer, deps_stream);
    putc ('\n', deps_stream);
    if (deps_stream != stdout) {
      fclose (deps_stream);
      if (ferror (deps_stream))
    fatal ("I/O error on output");
    }
  }
#ifdef sgi
  if (deps_file)
  {
    printf("**************No MDclose***************\n");
/*    MDclose(deps_file, deps_target);*/
  }
#endif
#ifdef DWARF_DM
    dm_finish_current();
    dm_write_file();
#endif

  fclose(fp_out);
  
  if (f != 0)
     close(f);

/*JPL: is this still needed?
  if (ferror (stdout))
    fatal ("I/O error on output");
*/
  reset_acpp();
  
  if (errors)
     return (FATAL_EXIT_CODE);
  return (SUCCESS_EXIT_CODE);

 perror:
  reset_acpp();
  pfatal_with_name (in_fname);
}
#ifdef sgi
static void
print_def_undef(FILE_BUF *op,U_CHAR *type, U_CHAR *name,int namelen)
{
    U_CHAR *lbuf;
    U_CHAR *lname;
    int fnamelen;
    U_CHAR *fname;
    FILE_BUF *fp;
    int len;
    
    fp = &instack[indepth];
    fname = fp->fname;
    fnamelen = strlen(fname);
    lbuf = xmalloc(namelen + 100 + fnamelen);
    lname = xmalloc(namelen + 100 + fnamelen);
    strncpy(lname,name,namelen);
    lname[namelen] = 0;
    sprintf(lbuf,"#%s %s #line %d %s\n",type,lname,fp->lineno,fname);
    len = strlen(lbuf);
    if(op == 0)
        op = &outbuf;
    check_expand(op,len+1);
    if (op->buf > op->buf && op->bufp[-1] != '\n')
        *op->bufp++ = '\n';
    bcopy(lbuf,op->bufp,len);
    op->bufp+= len;
    free(lbuf);
    free(lname);
}
#endif

/* Pre-C-Preprocessor to translate ANSI trigraph idiocy in BUF
   before main CCCP processing.  Name `pcp' is also in honor of the
   drugs the trigraph designers must have been on.

   Using an extra pass through the buffer takes a little extra time,
   but is infinitely less hairy than trying to handle ??/" inside
   strings, etc. everywhere, and also makes sure that trigraphs are
   only translated in the top level of processing. */

trigraph_pcp (buf)
     FILE_BUF *buf;
{
  register U_CHAR c, *fptr, *bptr, *sptr;
  int len;

  fptr = bptr = sptr = buf->buf;
  while ((sptr = (U_CHAR *) index (sptr, '?')) != NULL) {
    if (*++sptr != '?')
      continue;
    switch (*++sptr) {
      case '=':
      c = '#';
      break;
    case '(':
      c = '[';
      break;
    case '/':
      c = '\\';
      break;
    case ')':
      c = ']';
      break;
    case '\'':
      c = '^';
      break;
    case '<':
      c = '{';
      break;
    case '!':
      c = '|';
      break;
    case '>':
      c = '}';
      break;
    case '-':
      c  = '~';
      break;
    case '?':
      sptr--;
      continue;
    default:
      continue;
    }
    len = sptr - fptr - 2;
    if (bptr != fptr && len > 0)
      bcopy (fptr, bptr, len);  /* BSD doc says bcopy () works right
                   for overlapping strings.  In ANSI
                   C, this will be memmove (). */
    bptr += len;
    *bptr++ = c;
    fptr = ++sptr;
  }
  len = buf->length - (fptr - buf->buf);
  if (bptr != fptr && len > 0)
    bcopy (fptr, bptr, len);
  buf->length -= fptr - bptr;
  buf->buf[buf->length] = '\0';
  if (warn_trigraphs && fptr != bptr
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
                                     && warnings_off == 0
#endif
                                                         )
    warning ("%d trigraph(s) encountered", (fptr - bptr) / 2);
}

/* Move all backslash-newline pairs out of embarrassing places.
   Exchange all such pairs following BP
   with any potentially-embarrasing characters that follow them.
   Potentially-embarrassing characters are / and *
   (because a backslash-newline inside a comment delimiter
   would cause it not to be recognized).  */

void newline_fix (bp)
     U_CHAR *bp;
{
  register U_CHAR *p = bp;
  register int count = 0;

  /* First count the backslash-newline pairs here.  */

  while (*p++ == '\\' && *p++ == '\n')
    count++;

  p = bp + count * 2;

  /* What follows the backslash-newlines is not embarrassing.  */

  if (count == 0 || (*p != '/' && *p != '*'))
    return;

  /* Copy all potentially embarrassing characters
     that follow the backslash-newline pairs
     down to where the pairs originally started.  */

  while (*p == '*' || *p == '/')
    *bp++ = *p++;

  /* Now write the same number of pairs after the embarrassing chars.  */
  while (count-- > 0) {
    *bp++ = '\\';
    *bp++ = '\n';
  }
}

/* Like newline_fix but for use within a directive-name.
   Move any backslash-newlines up past any following symbol constituents.  */

void name_newline_fix (bp)
     U_CHAR *bp;
{
  register U_CHAR *p = bp;
  register int count = 0;

  /* First count the backslash-newline pairs here.  */

  while (*p++ == '\\' && *p++ == '\n')
    count++;

  p = bp + count * 2;

  /* What follows the backslash-newlines is not embarrassing.  */

  if (count == 0 || !is_idchar[*p])
    return;

  /* Copy all potentially embarrassing characters
     that follow the backslash-newline pairs
     down to where the pairs originally started.  */

  while (is_idchar[*p])
    *bp++ = *p++;

  /* Now write the same number of pairs after the embarrassing chars.  */
  while (count-- > 0) {
    *bp++ = '\\';
    *bp++ = '\n';
  }
}

/*
 * The main loop of the program.
 *
 * Read characters from the input stack, transferring them to the
 * output buffer OP.
 *
 * Macros are expanded and push levels on the input stack.
 * At the end of such a level it is popped off and we keep reading.
 * At the end of any other kind of level, we return.
 * #-directives are handled, except within macros.
 *
 * If OUTPUT_MARKS is nonzero, keep Newline markers found in the input
 * and insert them when appropriate.  This is set while scanning macro
 * arguments before substitution.  It is zero when scanning for final output.
 *   There are three types of Newline markers:
 *   * Newline -  follows a macro name that was not expanded
 *     because it appeared inside an expansion of the same macro.
 *     This marker prevents future expansion of that identifier.
 *     When the input is rescanned into the final output, these are deleted.
 *     These are also deleted by ## concatenation.
 *   * Newline Space (or Newline and any other whitespace character)
 *     stands for a place that tokens must be separated or whitespace
 *     is otherwise desirable, but where the ANSI standard specifies there
 *     is no whitespace.  This marker turns into a Space (or whichever other
 *     whitespace char appears in the marker) in the final output,
 *     but it turns into nothing in an argument that is stringified with #.
 *     Such stringified arguments are the only place where the ANSI standard
 *     specifies with precision that whitespace may not appear.
 *
 * During this function, IP->bufp is kept cached in IBP for speed of access.
 * Likewise, OP->bufp is kept in OBP.  Before calling a subroutine
 * IBP, IP and OBP must be copied back to memory.  IP and IBP are
 * copied back with the RECACHE macro.  OBP must be copied back from OP->bufp
 * explicitly, and before RECACHE, since RECACHE uses OBP.
 */

rescan (op, output_marks)
     FILE_BUF *op;
     int output_marks;
{
  /* Character being scanned in main loop.  */
  register U_CHAR c;

  /* Length of pending accumulated identifier.  */
  register int ident_length = 0;

  /* Hash code of pending accumulated identifier.  */
  register int hash = 0;

  /* Current input level (&instack[indepth]).  */
  FILE_BUF *ip;

  /* Pointer for scanning input.  */
  register U_CHAR *ibp;

  /* Pointer to end of input.  End of scan is controlled by LIMIT.  */
  register U_CHAR *limit;

  /* Pointer for storing output.  */
  register U_CHAR *obp;

  /* REDO_CHAR is nonzero if we are processing an identifier
     after backing up over the terminating character.
     Sometimes we process an identifier without backing up over
     the terminating character, if the terminating character
     is not special.  Backing up is done so that the terminating character
     will be dispatched on again once the identifier is dealt with.  */
  int redo_char = 0;

  /* 1 if within an identifier inside of which a concatenation
     marker (Newline -) has been seen.  */
  int concatenated = 0;

  /* While scanning a comment or a string constant,
     this records the line it started on, for error messages.  */
  int start_line;

  /* Record position of last `real' newline.  */
  U_CHAR *beg_of_line;

/* Pop the innermost input stack level, assuming it is a macro expansion.  */

#define POPMACRO \
do { ip->macro->type = T_MACRO;     \
     if (ip->free_ptr) free (ip->free_ptr); \
     --indepth; } while (0)

/* Reload `rescan's local variables that describe the current
   level of the input stack.  */

#define RECACHE  \
do { ip = &instack[indepth];        \
     ibp = ip->bufp;            \
     limit = ip->buf + ip->length;  \
     op->bufp = obp;            \
     check_expand (op, limit - ibp);    \
     beg_of_line = 0;           \
     obp = op->bufp; } while (0)

  if (no_output && instack[indepth].fname != 0)
    skip_if_group (&instack[indepth], 1);

  obp = op->bufp;
  RECACHE;
  beg_of_line = ibp;

  /* Our caller must always put a null after the end of
     the input at each input stack level.  */
  if (*limit != 0)
    abort ();

  while (1) {
    c = *ibp++;
    *obp++ = c;

    switch (c)
    {
    case '\\':
       if (ibp >= limit)
          break;
       if (*ibp == '\n') {
       /* Always merge lines ending with backslash-newline,
          even in middle of identifier.  */
          ++ibp;
          ++ip->lineno;
          --obp;        /* remove backslash from obuf */
          break;
       }
       /* Otherwise, backslash suppresses specialness of following char,
       so copy it here to prevent the switch from seeing it.
       But first get any pending identifier processed.  */
       if (ident_length > 0)
          goto specialchar;
       *obp++ = *ibp++;
       break;

    case '#':
    /* If this is expanding a macro definition, don't recognize
       preprocessor directives.  */
       if (ip->macro != 0)
          goto randomchar;
       if (ident_length)
          goto specialchar;

       /* # keyword: a # must be first nonblank char on the line */
       if (beg_of_line == 0)
          goto randomchar;
       {
          U_CHAR *bp;

          /* Scan from start of line, skipping whitespace, comments
          and backslash-newlines, and see if we reach this #.
          If not, this # is not special.  */
          bp = beg_of_line;
          while (1) {
             if (is_hor_space[*bp])
                bp++;
             else if (*bp == '\\' && bp[1] == '\n')
                bp += 2;
             else if (*bp == '/' && bp[1] == '*') {
                bp += 2;
                while (!(*bp == '*' && bp[1] == '/'))
                   bp++;
                bp += 2;
             }
             else if (cplusplus && *bp == '/' && bp[1] == '/') {
                bp += 2;
                while (*bp++ != '\n') ;
             }
             else break;
          }
          if (bp + 1 != ibp)
             goto randomchar;
       }

       /* This # can start a directive.  */

       --obp;       /* Don't copy the '#' */

       ip->bufp = ibp;
       op->bufp = obp;
       if (! handle_directive (ip, op)) {
#ifdef USE_C_ALLOCA
          alloca (0);
#endif
          /* Not a known directive: treat it as ordinary text.
          IP, OP, IBP, etc. have not been changed.  */
          if (no_output && instack[indepth].fname) {
          /* If not generating expanded output,
          what we do with ordinary text is skip it.
             Discard everything until next # directive.  */
             skip_if_group (&instack[indepth], 1);
             RECACHE;
             beg_of_line = ibp;
             break;
          }
          ++obp;        /* Copy the '#' after all */
          goto randomchar;
       }
#ifdef USE_C_ALLOCA
       alloca (0);
#endif
       /* A # directive has been successfully processed.  */
       /* If not generating expanded output, ignore everything until
       next # directive.  */
       if (no_output && instack[indepth].fname)
          skip_if_group (&instack[indepth], 1);
       obp = op->bufp;
       RECACHE;
       beg_of_line = ibp;
       break;

    case '{':
       //if (beg_of_line == 0)
          //goto randomchar;
       {
          int len;
          U_CHAR *bp;
#if 0
          /* Scan from start of line, skipping whitespace, comments
          and backslash-newlines, and see if we reach this {.
          If not, this { is not special.  */
          bp = beg_of_line;
          while (1)
          {
             if (is_hor_space[*bp])
                bp++;
             else if (*bp == '\\' && bp[1] == '\n')
                bp += 2;
             else if (*bp == '/' && bp[1] == '*')
             {
                bp += 2;
                while (!(*bp == '*' && bp[1] == '/'))
                   bp++;
                bp += 2;
             }
             else if (cplusplus && *bp == '/' && bp[1] == '/')
             {
                bp += 2;
                while (*bp++ != '\n') ;
             }
             else if (*bp == '{')
                break;
             else
                bp++;
          }
          if (bp + 1 != ibp)
             goto randomchar;
#endif
          /* This { can start an expression.  */

          --obp;        /* Don't copy the '{' */

          ip->bufp = ibp;
          op->bufp = obp;
          inside_math++;
          len = handle_c_expression(ip, op);
          inside_math--;
          ibp = ++ip->bufp;
          obp = op->bufp;
       }
       break;

    case '\"':          /* skip quoted string */
    case '\'':
    /* A single quoted string is treated like a double -- some
       programs (e.g., troff) are perverse this way */

       if (ident_length)
          goto specialchar;

       start_line = ip->lineno;

       /* Skip ahead to a matching quote.  */

       while (1) {
          if (ibp >= limit) {
             if (traditional) {
                if (ip->macro != 0) {
                   /* try harder: this string crosses a macro expansion boundary */
                   POPMACRO;
                   RECACHE;
                   continue;
                }
             } else
                error_with_line (line_for_error (start_line),
                "unterminated string or character constant");
             break;
          }
          *obp++ = *ibp;
          switch (*ibp++) {
          case '\n':
             ++ip->lineno;
             ++op->lineno;
             if (traditional)
                goto while2end;
             if (pedantic || c == '\'') {
                error_with_line (line_for_error (start_line),
                   "unterminated string or character constant");
                goto while2end;
             }
             break;

          case '\\':
             if (ibp >= limit)
                break;
             if (*ibp == '\n') {
             /* Backslash newline is replaced by nothing at all,
                but keep the line counts correct.  */
                --obp;
                ++ibp;
                ++ip->lineno;
             } else {
             /* ANSI stupidly requires that in \\ the second \
                is *not* prevented from combining with a newline.  */
                while (*ibp == '\\' && ibp[1] == '\n') {
                   ibp += 2;
                   ++ip->lineno;
                }
                *obp++ = *ibp++;
             }
             break;

          case '\"':
          case '\'':
             if (ibp[-1] == c)
                goto while2end;
             break;
          }
       }
while2end:
       break;

          case '/':
             if (*ibp == '\\' && ibp[1] == '\n')
                newline_fix (ibp);
             if (cplusplus && *ibp == '/') {
                /* C++ style comment... */
                start_line = ip->lineno;

                --ibp;          /* Back over the slash */
                --obp;

                /* Comments are equivalent to spaces. */
                if (! put_out_comments)
                   *obp++ = ' ';
                else {
                   /* must fake up a comment here */
                   *obp++ = '/';
                   *obp++ = '/';
                }
                {
                   U_CHAR *before_bp = ibp+2;

                   while (ibp < limit) {
                      if (*ibp++ == '\n') {
                         ibp--;
                         if (put_out_comments) {
                            bcopy (before_bp, obp, ibp - before_bp);
                            obp += ibp - before_bp;
                         }
                         break;
                      }
                   }
                   break;
                }
             }
             if (*ibp != '*')
                goto randomchar;
             if (ip->macro != 0)
                goto randomchar;
             if (ident_length)
                goto specialchar;

             /* We have a comment.  Skip it, optionally copying it to output.  */

             start_line = ip->lineno;

             ++ibp;         /* Skip the star. */

                           /* Comments are equivalent to spaces.
                           Note that we already output the slash; we might not want it.
             For -traditional, a comment is equivalent to nothing.  */
             if (! put_out_comments) {
                if (traditional)
                   obp--;
                else
                   obp[-1] = ' ';
             }
             else
                *obp++ = '*';

             {
                U_CHAR *before_bp = ibp;

                while (ibp < limit) {
                   switch (*ibp++) {
                   case '/':
                      if (warn_comments && ibp < limit && *ibp == '*'
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
                                                                      && warnings_off == 0
#endif
                                                                                          )
                         warning("`/*' within comment");
                      break;
                   case '*':
                      if (*ibp == '\\' && ibp[1] == '\n')
                         newline_fix (ibp);
                      if (ibp >= limit || *ibp == '/')
                         goto comment_end;
                      break;
                   case '\n':
                      ++ip->lineno;
                      /* Copy the newline into the output buffer, in order to
                      avoid the pain of a #line every time a multiline comment
                      is seen.  */
                      if (!put_out_comments)
                         *obp++ = '\n';
                      ++op->lineno;
                   }
                }
comment_end:

                if (ibp >= limit)
                   error_with_line (line_for_error (start_line),
                   "unterminated comment");
                else {
                   ibp++;
                   if (put_out_comments) {
                      bcopy (before_bp, obp, ibp - before_bp);
                      obp += ibp - before_bp;
                   }
                }
             }
             break;

          case '$':
             if (!dollars_in_ident)
                goto randomchar;
             goto letter;

          case '0': case '1': case '2': case '3': case '4':
          case '5': case '6': case '7': case '8': case '9':
          /* If digit is not part of identifier, it starts a number,
          which means that following letters are not an identifier.
          "0x5" does not refer to an identifier "x5".
          So copy all alphanumerics that follow without accumulating
             as an identifier.  Periods also, for sake of "3.e7".  */

             if (ident_length == 0) {
                while (ibp < limit) {
                   c = *ibp++;
                   if (!isalnum (c) && c != '.' && c != '_') {
                      --ibp;
                      break;
                   }
                   *obp++ = c;
                   /* A sign can be part of a preprocessing number
                   if it follows an e.  */
                   if (c == 'e' || c == 'E') {
                      if (ibp < limit && (*ibp == '+' || *ibp == '-'))
                         *obp++ = *ibp++;
                   }
                }
                break;
             }
             /* fall through */

    case '_':
    case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
    case 'g': case 'h': case 'i': case 'j': case 'k': case 'l':
    case 'm': case 'n': case 'o': case 'p': case 'q': case 'r':
    case 's': case 't': case 'u': case 'v': case 'w': case 'x':
    case 'y': case 'z':
    case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
    case 'G': case 'H': case 'I': case 'J': case 'K': case 'L':
    case 'M': case 'N': case 'O': case 'P': case 'Q': case 'R':
    case 'S': case 'T': case 'U': case 'V': case 'W': case 'X':
    case 'Y': case 'Z':
    letter:
      ident_length++;
      /* Compute step of hash function, to avoid a proc call on every token */
      hash = HASHSTEP (hash, c);
      break;

    case '\n':
      /* If reprocessing a macro expansion, newline is a special marker.  */
      if (ip->macro != 0) {
    /* Newline White is a "funny space" to separate tokens that are
       supposed to be separate but without space between.
       Here White means any horizontal whitespace character.
       Newline - marks a recursive macro use that is not
       supposed to be expandable.  */

    if (*ibp == '-') {
      /* Newline - inhibits expansion of preceding token.
         If expanding a macro arg, we keep the newline -.
         In final output, it is deleted.  */
      if (! concatenated) {
        ident_length = 0;
        hash = 0;
      }
      ibp++;
      if (!output_marks) {
        obp--;
      } else {
        /* If expanding a macro arg, keep the newline -.  */
        *obp++ = '-';
      }
    } else if (is_space[*ibp]) {
      /* Newline Space does not prevent expansion of preceding token
         so expand the preceding token and then come back.  */
      if (ident_length > 0)
        goto specialchar;

      /* If generating final output, newline space makes a space.  */
      if (!output_marks) {
        obp[-1] = *ibp++;
        /* And Newline Newline makes a newline, so count it.  */
        if (obp[-1] == '\n')
          op->lineno++;
      } else {
        /* If expanding a macro arg, keep the newline space.
           If the arg gets stringified, newline space makes nothing.  */
        *obp++ = *ibp++;
      }
    } else abort ();    /* Newline followed by something random?  */
    break;
      }

      /* If there is a pending identifier, handle it and come back here.  */
      if (ident_length > 0)
    goto specialchar;

      beg_of_line = ibp;

      /* Update the line counts and output a #line if necessary.  */
      ++ip->lineno;
      ++op->lineno;
      if (ip->lineno != op->lineno) {
    op->bufp = obp;
    output_line_command (ip, op, 1, same_file);
    check_expand (op, ip->length - (ip->bufp - ip->buf));
    obp = op->bufp;
      }
      break;

      /* Come here either after (1) a null character that is part of the input
     or (2) at the end of the input, because there is a null there.  */
    case 0:
      if (ibp <= limit)
    /* Our input really contains a null character.  */
    goto randomchar;

      /* At end of a macro-expansion level, pop it and read next level.  */
      if (ip->macro != 0) {
    obp--;
    ibp--;
    /* If traditional, and we have an identifier that ends here,
       process it now, so we get the right error for recursion.  */
    if (traditional && ident_length
        && ! is_idchar[*instack[indepth - 1].bufp]) {
      redo_char = 1;
      goto randomchar;
    }
    POPMACRO;
    RECACHE;
    break;
      }

      /* If we don't have a pending identifier,
     return at end of input.  */
      if (ident_length == 0) {
    obp--;
    ibp--;
    op->bufp = obp;
    ip->bufp = ibp;
    goto ending;
      }

      /* If we do have a pending identifier, just consider this null
     a special character and arrange to dispatch on it again.
     The second time, IDENT_LENGTH will be zero so we will return.  */

      /* Fall through */

specialchar:

      /* Handle the case of a character such as /, ', " or null
     seen following an identifier.  Back over it so that
     after the identifier is processed the special char
     will be dispatched on again.  */

      ibp--;
      obp--;
      redo_char = 1;

    default:

randomchar:

      if (ident_length > 0) {
    register HASHNODE *hp;

    /* We have just seen an identifier end.  If it's a macro, expand it.

       IDENT_LENGTH is the length of the identifier
       and HASH is its hash code.

       The identifier has already been copied to the output,
       so if it is a macro we must remove it.

       If REDO_CHAR is 0, the char that terminated the identifier
       has been skipped in the output and the input.
       OBP-IDENT_LENGTH-1 points to the identifier.
       If the identifier is a macro, we must back over the terminator.

       If REDO_CHAR is 1, the terminating char has already been
       backed over.  OBP-IDENT_LENGTH points to the identifier.  */

    for (hp = hashtab[MAKE_POS (hash) % HASHSIZE]; hp != NULL;
         hp = hp->next) {

      if (hp->length == ident_length) {
        U_CHAR *obufp_before_macroname;
        int op_lineno_before_macroname;
        register int i = ident_length;
        register U_CHAR *p = hp->name;
        register U_CHAR *q = obp - i;
        int disabled;

        if (! redo_char)
          q--;

        do {        /* All this to avoid a strncmp () */
          if (*p++ != *q++)
        goto hashcollision;
        } while (--i);

        /* We found a use of a macro name.
           see if the context shows it is a macro call.  */

        /* Back up over terminating character if not already done.  */
        if (! redo_char) {
          ibp--;
          obp--;
        }

        obufp_before_macroname = obp - ident_length;
        op_lineno_before_macroname = op->lineno;

        /* Record whether the macro is disabled.  */
        disabled = hp->type == T_DISABLED;

        /* This looks like a macro ref, but if the macro was disabled,
           just copy its name and put in a marker if requested.  */

        if (disabled) {
#if 0
          /* This error check caught useful cases such as
         #define foo(x,y) bar(x(y,0), y)
         foo(foo, baz)  */
          if (traditional)
        error ("recursive use of macro `%s'", hp->name);
#endif

          if (output_marks) {
        check_expand (op, limit - ibp + 2);
        *obp++ = '\n';
        *obp++ = '-';
          }
          break;
        }

        /* If macro wants an arglist, verify that a '(' follows.
           first skip all whitespace, copying it to the output
           after the macro name.  Then, if there is no '(',
           decide this is not a macro call and leave things that way.  */
        if ((hp->type == T_MACRO || hp->type == T_DISABLED)
        && hp->value.defn->nargs >= 0)
          {
        while (1) {
          /* Scan forward over whitespace, copying it to the output.  */
          if (ibp == limit && ip->macro != 0) {
            POPMACRO;
            RECACHE;
          }
          /* A comment: copy it to the output unchanged.  */
          else if (*ibp == '/' && ibp+1 != limit && ibp[1] == '*') {
            *obp++ = '/';
            *obp++ = '*';
            ibp += 2;
            while (ibp + 1 != limit
               && !(ibp[0] == '*' && ibp[1] == '/')) {
              /* We need not worry about newline-marks,
             since they are never found in comments.  */
              if (*ibp == '\n') {
            /* Newline in a file.  Count it.  */
            ++ip->lineno;
            ++op->lineno;
              }
              *obp++ = *ibp++;
            }
            ibp += 2;
            *obp++ = '*';
            *obp++ = '/';
          }
          else if (is_space[*ibp]) {
            *obp++ = *ibp++;
            if (ibp[-1] == '\n') {
              if (ip->macro == 0) {
            /* Newline in a file.  Count it.  */
            ++ip->lineno;
            ++op->lineno;
              } else if (!output_marks) {
            /* A newline mark, and we don't want marks
               in the output.  If it is newline-hyphen,
               discard it entirely.  Otherwise, it is
               newline-whitechar, so keep the whitechar.  */
            obp--;
            if (*ibp == '-')
              ibp++;
            else {
              if (*ibp == '\n')
                ++op->lineno;
              *obp++ = *ibp++;
            }
              } else {
            /* A newline mark; copy both chars to the output.  */
            *obp++ = *ibp++;
              }
            }
          }
          else break;
        }
        if (*ibp != '(')
          break;
          }

        /* This is now known to be a macro call.
           Discard the macro name from the output,
           along with any following whitespace just copied.  */
        obp = obufp_before_macroname;
        op->lineno = op_lineno_before_macroname;

        /* Expand the macro, reading arguments as needed,
           and push the expansion on the input stack.  */
        ip->bufp = ibp;
        op->bufp = obp;
        macroexpand (hp, op);

        /* Reexamine input stack, since macroexpand has pushed
           a new level on it.  */
        obp = op->bufp;
        RECACHE;
        break;
      }
hashcollision:
           ;
    }           /* End hash-table-search loop */
    ident_length = hash = 0; /* Stop collecting identifier */
    redo_char = 0;
    concatenated = 0;
      }             /* End if (ident_length > 0) */
    }               /* End switch */
  }             /* End per-char loop */

  /* Come here to return -- but first give an error message
     if there was an unterminated successful conditional.  */
 ending:
  if (if_stack != ip->if_stack) {
    char *str;
    switch (if_stack->type) {
    case T_IF:
      str = "if";
      break;
    case T_IFDEF:
      str = "ifdef";
      break;
    case T_IFNDEF:
      str = "ifndef";
      break;
    case T_ELSE:
      str = "else";
      break;
    case T_ELIF:
      str = "elif";
      break;
    }
    error_with_line (line_for_error (if_stack->lineno),
             "unterminated #%s conditional", str);
  }
  if_stack = ip->if_stack;
}

/*
 * Rescan a string into a temporary buffer and return the result
 * as a FILE_BUF.  Note this function returns a struct, not a pointer.
 *
 * OUTPUT_MARKS nonzero means keep Newline markers found in the input
 * and insert such markers when appropriate.  See `rescan' for details.
 * OUTPUT_MARKS is 1 for macroexpanding a macro argument separately
 * before substitution; it is 0 for other uses.
 */
FILE_BUF
expand_to_temp_buffer (buf, limit, output_marks)
     U_CHAR *buf, *limit;
     int output_marks;
{
  register FILE_BUF *ip;
  FILE_BUF obuf;
  int length = limit - buf;
  U_CHAR *buf1;
  int odepth = indepth;

  if (length < 0)
    abort ();

  /* Set up the input on the input stack.  */

  buf1 = (U_CHAR *) alloca (length + 1);
  {
    register U_CHAR *p1 = buf;
    register U_CHAR *p2 = buf1;

    while (p1 != limit)
      *p2++ = *p1++;
  }
  buf1[length] = 0;

  /* Set up to receive the output.  */

  obuf.length = length * 2 + 100; /* Usually enough.  Why be stingy?  */
  obuf.bufp = obuf.buf = (U_CHAR *) xmalloc (obuf.length);
  obuf.fname = 0;
  obuf.macro = 0;
  obuf.free_ptr = 0;

  CHECK_DEPTH ({return obuf;});

  ++indepth;

  ip = &instack[indepth];
#ifdef sgi
    ip->endif_cruft_message_issued = 0;
#endif
  ip->fname = 0;
  ip->macro = 0;
  ip->free_ptr = 0;
  ip->length = length;
  ip->buf = ip->bufp = buf1;
  ip->if_stack = if_stack;

  ip->lineno = obuf.lineno = 1;

  /* Scan the input, create the output.  */

  rescan (&obuf, output_marks);

  /* Pop input stack to original state.  */
  --indepth;

  if (indepth != odepth)
    abort ();

  /* Record the output.  */
  obuf.length = obuf.bufp - obuf.buf;

  return obuf;
}

/*
 * Process a # directive.  Expects IP->bufp to point to the '#', as in
 * `#define foo bar'.  Passes to the command handler
 * (do_define, do_include, etc.): the addresses of the 1st and
 * last chars of the command (starting immediately after the #
 * keyword), plus op and the keyword table pointer.  If the command
 * contains comments it is copied into a temporary buffer sans comments
 * and the temporary buffer is passed to the command handler instead.
 * Likewise for backslash-newlines.
 *
 * Returns nonzero if this was a known # directive.
 * Otherwise, returns zero, without advancing the input pointer.
 */

int
handle_directive (ip, op)
     FILE_BUF *ip, *op;
{
  register U_CHAR *bp, *cp;
  register struct directive *kt;
  register int ident_length;
  U_CHAR *resume_p;

  /* Nonzero means we must copy the entire command
     to get rid of comments or backslash-newlines.  */
  int copy_command = 0;

  U_CHAR *ident, *after_ident;

  bp = ip->bufp;
  /* Skip whitespace and \-newline.  */
  while (1) {
    if (is_hor_space[*bp])
      bp++;
    else if (*bp == '/' && bp[1] == '*') {
      ip->bufp = bp;
      skip_to_end_of_comment (ip, &ip->lineno);
      bp = ip->bufp;
    } else if (*bp == '\\' && bp[1] == '\n') {
      bp += 2; ip->lineno++;
    } else break;
  }

  /* Now find end of directive name.
     If we encounter a backslash-newline, exchange it with any following
     symbol-constituents so that we end up with a contiguous name.  */

  cp = bp;
  while (1) {
    if (is_idchar[*cp])
      cp++;
    else {
      if (*cp == '\\' && cp[1] == '\n')
    name_newline_fix (cp);
      if (is_idchar[*cp])
    cp++;
      else break;
    }
  }
  ident_length = cp - bp;
  ident = bp;
  after_ident = cp;

  /* A line of just `#' becomes blank.  */

  if (traditional && ident_length == 0 && *after_ident == '\n') {
    ip->bufp = after_ident;
    return 1;
  }
#ifdef sgi
  /* A line of just `#' becomes blank.  */
  if (ident_length == 0 && *after_ident == '\n') {
    ip->bufp = after_ident;
    return 1;
  }
#endif

  /*
   * Decode the keyword and call the appropriate expansion
   * routine, after moving the input pointer up to the next line.
   */

  for (kt = directive_table; kt->length > 0; kt++) {
    if (kt->length == ident_length && !strncmp (kt->name, ident, ident_length)) {
      register U_CHAR *buf;
      register U_CHAR *limit = ip->buf + ip->length;
      int unterminated = 0;

      /* Nonzero means do not delete comments within the directive.
     #define needs this when -traditional.  */
      int keep_comments = traditional && kt->traditional_comments;

      /* Find the end of this command (first newline not backslashed
     and not in a string or comment).
     Set COPY_COMMAND if the command must be copied
     (it contains a backslash-newline or a comment).  */

      buf = bp = after_ident;
      while (bp < limit)
      {
         register U_CHAR c = *bp++;
         switch (c) {
         case '\\':
            if (bp < limit) {
               if (*bp == '\n') {
                  ip->lineno++;
                  copy_command = 1;
               }
               bp++;
            }
            break;

         case '\'':
         case '\"':
            bp = skip_quoted_string (bp - 1, limit, ip->lineno, &ip->lineno, &copy_command, &unterminated);
            /* Don't bother calling the directive if we already got an error
            message due to unterminated string.  Skip everything and pretend
            we called the directive.  */
            if (unterminated) {
               if (traditional) {
                  /* Traditional preprocessing permits unterminated strings.  */
                  --bp;
                  ip->bufp = bp;
                  goto endloop1;
               }
               ip->bufp = bp;
               return 1;
            }
            break;

            /* <...> is special for #include.  */
         case '<':
            if (!kt->angle_brackets)
               break;
            while (*bp && *bp != '>') bp++;
            break;

         case '/':
            if (*bp == '\\' && bp[1] == '\n')
               newline_fix (bp);
            if (*bp == '*'
               || (cplusplus && *bp == '/')) {
               U_CHAR *obp = bp - 1;
               ip->bufp = bp + 1;
               skip_to_end_of_comment (ip, &ip->lineno);
               bp = ip->bufp;
               /* No need to copy the command because of a comment at the end;
               just don't include the comment in the directive.  */
               if (bp == limit || *bp == '\n') {
                  bp = obp;
                  goto endloop1;
               }
               /* Don't remove the comments if -traditional.  */
               if (! keep_comments)
                  copy_command++;
            }
            break;

         case '\n':
            --bp;       /* Point to the newline */
            ip->bufp = bp;
            goto endloop1;
         }
      }
      ip->bufp = bp;

    endloop1:
      resume_p = ip->bufp;
      /* BP is the end of the directive.
     RESUME_P is the next interesting data after the directive.
     A comment may come between.  */

      if (copy_command) {
    register U_CHAR *xp = buf;
    /* Need to copy entire command into temp buffer before dispatching */

    cp = (U_CHAR *) alloca (bp - buf + 5); /* room for cmd plus
                          some slop */
    buf = cp;

    /* Copy to the new buffer, deleting comments
       and backslash-newlines (and whitespace surrounding the latter).  */

    while (xp < bp) {
      register U_CHAR c = *xp++;
      *cp++ = c;

      switch (c) {
      case '\n':
        break;

        /* <...> is special for #include.  */
      case '<':
        if (!kt->angle_brackets)
          break;
        while (xp < bp && c != '>') {
          c = *xp++;
          *cp++ = c;
        }
        break;

      case '\\':
        if (*xp == '\n') {
          xp++;
          cp--;
#if sgi
          if (no_newline_splicing) {
        *cp++ = '\n';
        *cp++ = '\n';
        break;
          }
#endif
          if (cp != buf && is_space[cp[-1]]) {
        while (cp != buf && is_space[cp[-1]]) cp--;
        cp++;
        SKIP_WHITE_SPACE (xp);
          } else if (is_space[*xp]) {
        *cp++ = *xp++;
        SKIP_WHITE_SPACE (xp);
          }
        }
        break;

      case '\'':
      case '\"':
        if (!traditional) {
          register U_CHAR *bp1
        = skip_quoted_string (xp - 1, limit, ip->lineno, 0, 0, 0);
          while (xp != bp1)
        *cp++ = *xp++;
        }
        break;

      case '/':
        if (keep_comments)
          break;
        if (*xp == '*'
        || (cplusplus && *xp == '/')) {
          if (traditional)
        cp--;
          else
        cp[-1] = ' ';
          ip->bufp = xp + 1;
          skip_to_end_of_comment (ip, 0);
          xp = ip->bufp;
        }
      }
    }

    /* Null-terminate the copy.  */

    *cp = 0;
      }
      else
    cp = bp;

      ip->bufp = resume_p;

      /* Some directives should be written out for cc1 to process,
     just as if they were not defined.  */

      if (kt->pass_thru) {
        int len;

    /* Output directive name.  */
        check_expand (op, kt->length+1);
        *op->bufp++ = '#';
        bcopy (kt->name, op->bufp, kt->length);
        op->bufp += kt->length;

    /* Output arguments.  */
        len = (cp - buf);
        check_expand (op, len);
        bcopy (buf, op->bufp, len);
        op->bufp += len;
      }

      /* Call the appropriate command handler.  buf now points to
     either the appropriate place in the input buffer, or to
     the temp buffer if it was necessary to make one.  cp
     points to the first char after the contents of the (possibly
     copied) command, in either case. */
      (*kt->func) (buf, cp, op, kt);
      check_expand (op, ip->length - (ip->bufp - ip->buf));

      return 1;
    }
  }
#ifdef sgi
    if(pedantic && warnings_off == 0)
      warning("Not a legal preprocessing directive; inappropriate non-whitespace characters are present (ANSI 3.8)");
#endif
  return 0;
}

/*
 * Process a # directive.  Expects IP->bufp to point to the '#', as in
 * `#define foo bar'.  Passes to the command handler
 * (do_define, do_include, etc.): the addresses of the 1st and
 * last chars of the command (starting immediately after the #
 * keyword), plus op and the keyword table pointer.  If the command
 * contains comments it is copied into a temporary buffer sans comments
 * and the temporary buffer is passed to the command handler instead.
 * Likewise for backslash-newlines.
 *
 * Returns nonzero if this was a known # directive.
 * Otherwise, returns zero, without advancing the input pointer.
 */

int
handle_c_expression (ip, op)
     FILE_BUF *ip, *op;
{
  register U_CHAR *bp, *cp;
  register int ident_length;
  U_CHAR *resume_p;

  /* Nonzero means we must copy the entire command
     to get rid of comments or backslash-newlines.  */
  int copy_command = 1;

  U_CHAR *ident, *after_ident;

  bp = ip->bufp;
  /* Skip whitespace and \-newline.  */
  while (1) {
    if (is_hor_space[*bp])
      bp++;
    else if (*bp == '/' && bp[1] == '*') {
      ip->bufp = bp;
      skip_to_end_of_comment (ip, &ip->lineno);
      bp = ip->bufp;
    } else if (*bp == '\\' && bp[1] == '\n') {
      bp += 2; ip->lineno++;
    } else break;
  }

  /* Now find end of directive name.
     If we encounter a backslash-newline, exchange it with any following
     symbol-constituents so that we end up with a contiguous name.  */

  cp = bp;
#if 0
  while (1) {
    if (is_idchar[*cp])
      cp++;
    else {
      if (*cp == '\\' && cp[1] == '\n')
    name_newline_fix (cp);
      if (is_idchar[*cp])
    cp++;
      else break;
    }
  }
#endif

  ident_length = cp - bp;
  ident = bp;
  after_ident = cp;

#if 0
  /* A line of just `#' becomes blank.  */

  if (traditional && ident_length == 0 && *after_ident == '\n') {
    ip->bufp = after_ident;
    return 1;
  }
#ifdef sgi
  /* A line of just `#' becomes blank.  */
  if (ident_length == 0 && *after_ident == '\n') {
    ip->bufp = after_ident;
    return 1;
  }
#endif

#endif

  /*
   * Decode the keyword and call the appropriate expansion
   * routine, after moving the input pointer up to the next line.
   */

  {
     register U_CHAR *buf;
     register U_CHAR *limit = ip->buf + ip->length;
     int unterminated = 0;

     /* Nonzero means do not delete comments within the directive.
        #define needs this when -traditional.  */
     int keep_comments = traditional;

     /* Find the end of this command (first newline not backslashed
     and not in a string or comment).
     Set COPY_COMMAND if the command must be copied
     (it contains a backslash-newline or a comment).  */

     buf = bp = after_ident;
     while (bp < limit)
     {
        register U_CHAR c = *bp++;
        switch (c)
        {
        case '\\':
           if (bp < limit) {
              if (*bp == '\n') {
                 ip->lineno++;
                 copy_command = 1;
              }
              bp++;
           }
           break;

        case '\'':
        case '\"':
           bp = skip_quoted_string (bp - 1, limit, ip->lineno, &ip->lineno, &copy_command, &unterminated);
           /* Don't bother calling the directive if we already got an error
           message due to unterminated string.  Skip everything and pretend
           we called the directive.  */
           if (unterminated) {
              if (traditional) {
                 /* Traditional preprocessing permits unterminated strings.  */
                 --bp;
                 ip->bufp = bp;
                 goto endloop1;
              }
              ip->bufp = bp;
              return 1;
           }
           break;

        case '/':
           if (*bp == '\\' && bp[1] == '\n')
              newline_fix (bp);
           if (*bp == '*'
              || (cplusplus && *bp == '/')) {
              U_CHAR *obp = bp - 1;
              ip->bufp = bp + 1;
              skip_to_end_of_comment (ip, &ip->lineno);
              bp = ip->bufp;
              /* No need to copy the command because of a comment at the end;
              just don't include the comment in the directive.  */
              if (bp == limit || *bp == '\n') {
                 bp = obp;
                 goto endloop1;
              }
              /* Don't remove the comments if -traditional.  */
              if (! keep_comments)
                 copy_command++;
           }
           break;

        case '}':
           --bp;        /* Point to the newline */
           ip->bufp = bp;
           goto endloop1;
        }
     }
     ip->bufp = bp;

endloop1:
     resume_p = ip->bufp;
     /* BP is the end of the directive.
     RESUME_P is the next interesting data after the directive.
     A comment may come between.  */

     if (copy_command)
     {
        register U_CHAR *xp = buf;
        /* Need to copy entire command into temp buffer before dispatching */

        cp = (U_CHAR *) alloca (bp - buf + 5); /* room for cmd plus
                          some slop */
        buf = cp;

        /* Copy to the new buffer, deleting comments
        and backslash-newlines (and whitespace surrounding the latter).  */

        while (xp < bp)
        {
           register U_CHAR c = *xp++;
           *cp++ = c;

           switch (c) {
           case '\n':
              break;

           case '\\':
              if (*xp == '\n') {
                 xp++;
                 cp--;
#if sgi
                 if (no_newline_splicing) {
                    *cp++ = '\n';
                    *cp++ = '\n';
                    break;
                 }
#endif
                 if (cp != buf && is_space[cp[-1]]) {
                    while (cp != buf && is_space[cp[-1]]) cp--;
                    cp++;
                    SKIP_WHITE_SPACE (xp);
                 } else if (is_space[*xp]) {
                    *cp++ = *xp++;
                    SKIP_WHITE_SPACE (xp);
                 }
              }
              break;

           case '\'':
           case '\"':
              if (!traditional) {
                 register U_CHAR *bp1
                    = skip_quoted_string (xp - 1, limit, ip->lineno, 0, 0, 0);
                 while (xp != bp1)
                    *cp++ = *xp++;
              }
              break;

           case '/':
              if (keep_comments)
                 break;
              if (*xp == '*'
                 || (cplusplus && *xp == '/')) {
                 if (traditional)
                    cp--;
                 else
                    cp[-1] = ' ';
                 ip->bufp = xp + 1;
                 skip_to_end_of_comment (ip, 0);
                 xp = ip->bufp;
              }
           }
        }

        /* Null-terminate the copy.  */

        *cp = 0;
     }
     else
     {
        cp = bp;
     }

     ip->bufp = resume_p;

     /* Some directives should be written out for cc1 to process,
     just as if they were not defined.  */

     /* buf now points to the beginning of the expression,
      * and is null terminated after the end of the expression.
      */
     {
        int len;
        double val;
        char obuf[512];
        FILE_BUF temp_obuf;

        len = strlen(buf);
        temp_obuf = expand_to_temp_buffer (buf, buf + len, 0);
        val = parse_c_math_expression(temp_obuf.buf);
        sprintf(obuf, "%lf", val);
        len = strlen(obuf);
        bcopy(obuf, op->bufp, len);
        op->bufp += len;
     }
     check_expand (op, ip->length - (ip->bufp - ip->buf));

     return 1;
  }

#ifdef sgi
    if(pedantic && warnings_off == 0)
      warning("Not a legal preprocessing directive; inappropriate non-whitespace characters are present (ANSI 3.8)");
#endif

  return 0;
}


static char *monthnames[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                 "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
                };

static const char* _get_filename_without_path (const char* filename)
{
#if defined WIN32 || defined _WIN32 || defined __WIN32 || defined __WIN32__
  #define DIR_SEP_CHAR   '\\'
#else
  #define DIR_SEP_CHAR   '/'
#endif

   const char* p = strrchr(filename, DIR_SEP_CHAR);
   
   return p ? ++p : filename;
}

/*
 * error - print error message and increment count of errors.
 */
static error (msg, arg1, arg2, arg3)
     char *msg;
{
  int i;
  FILE_BUF *ip = NULL;

  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != NULL) {
      ip = &instack[i];
      break;
    }

  if (ip != NULL)
    USER_MESSAGE("%s:%d: ", _get_filename_without_path(ip->fname), ip->lineno);
  USER_MESSAGE(msg, arg1, arg2, arg3);
  USER_MESSAGE("\n");
  errors++;
  return 0;
}

/*
 * expand things like __FILE__.  Place the expansion into the output
 * buffer *without* rescanning.
 */
void special_symbol (hp, op)
     HASHNODE *hp;
     FILE_BUF *op;
{
  char *buf;
  long t;
  int i, len;
  int true_indepth;
  FILE_BUF *ip = NULL;
  static struct tm *timebuf = NULL;
#ifndef sgi
  struct tm *localtime ();
#endif

  int paren = 0;        /* For special `defined' keyword */

  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != NULL) {
      ip = &instack[i];
      break;
    }
  if (ip == NULL) {
    error ("cccp error: not in any file?!");
    return;         /* the show must go on */
  }

  switch (hp->type) {
  case T_FILE:
  case T_BASE_FILE:
    {
      char *string;
      if (hp->type == T_FILE)
    string = ip->fname;
      else
    string = instack[0].fname;

      if (string)
    {
      buf = (char *) alloca (3 + strlen (string));
      sprintf (buf, "\"%s\"", string);
    }
      else
    buf = "\"\"";

      break;
    }

  case T_INCLUDE_LEVEL:
    true_indepth = 0;
    for (i = indepth; i >= 0; i--)
      if (instack[i].fname != NULL)
        true_indepth++;

    buf = (char *) alloca (8);  /* Eigth bytes ought to be more than enough */
    sprintf (buf, "%d", true_indepth - 1);
    break;

  case T_VERSION:
    buf = (char *) alloca (3 + strlen (version_string));
    sprintf (buf, "\"%s\"", version_string);
    break;

  case T_CONST:
    buf = (char *) alloca (4 * sizeof (int));
    sprintf (buf, "%d", hp->value.ival);
    break;

  case T_SPECLINE:
    buf = (char *) alloca (10);
    sprintf (buf, "%d", ip->lineno);
    break;

  case T_DATE:
  case T_TIME:
    if (timebuf == NULL) {
      t = time (0);
      timebuf = localtime (&t);
    }
    buf = (char *) alloca (20);
    if (hp->type == T_DATE)
      sprintf (buf, "\"%s %2d %4d\"", monthnames[timebuf->tm_mon],
          timebuf->tm_mday, timebuf->tm_year + 1900);
    else
      sprintf (buf, "\"%02d:%02d:%02d\"", timebuf->tm_hour, timebuf->tm_min,
          timebuf->tm_sec);
    break;

  case T_SPEC_DEFINED:
    buf = " 0 ";        /* Assume symbol is not defined */
    ip = &instack[indepth];
    SKIP_WHITE_SPACE (ip->bufp);
    if (*ip->bufp == '(') {
      paren++;
      ip->bufp++;           /* Skip over the paren */
      SKIP_WHITE_SPACE (ip->bufp);
    }

    if (!is_idstart[*ip->bufp])
      goto oops;
    if (lookup (ip->bufp, -1, -1))
      buf = " 1 ";
    while (is_idchar[*ip->bufp])
      ++ip->bufp;
    SKIP_WHITE_SPACE (ip->bufp);
    if (paren) {
      if (*ip->bufp != ')')
    goto oops;
      ++ip->bufp;
    }
    break;

oops:

    error ("`defined' must be followed by ident or (ident)");
    break;

  default:
    error ("cccp error: invalid special hash type"); /* time for gdb */
    abort ();
  }
  len = strlen (buf);
  check_expand (op, len);
  bcopy (buf, op->bufp, len);
  op->bufp += len;

  return;
}


/* Routines to handle #directives */

/*
 * Process include file by reading it in and calling rescan.
 * Expects to see "fname" or <fname> on the input.
 */

do_include (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  char *fname;      /* Dynamically allocated fname buffer */
  U_CHAR *fbeg, *fend;      /* Beginning and end of fname */

  struct file_name_list *stackp = include; /* Chain of dirs to search */
  struct file_name_list dsp[1]; /* First in chain, if #include "..." */
  int flen;

  int f;            /* file number */

  int retried = 0;      /* Have already tried macro
                   expanding the include line*/
  FILE_BUF trybuf;      /* It got expanded into here */
  int system_header_p = 0;  /* 0 for "...", 1 for <...> */

  f= -1;            /* JF we iz paranoid! */

get_filename:

  fbeg = buf;
  SKIP_WHITE_SPACE (fbeg);
  /* Discard trailing whitespace so we can easily see
     if we have parsed all the significant chars we were given.  */
  while (limit != fbeg && is_hor_space[limit[-1]]) limit--;

  switch (*fbeg++) {
  case '\"':
    fend = fbeg;
    while (fend != limit && *fend != '\"')
      fend++;
    if (*fend == '\"' && fend + 1 == limit) {
      FILE_BUF *fp;

      /* We have "filename".  Figure out directory this source
     file is coming from and put it on the front of the list. */

      /* If -I- was specified, don't search current dir, only spec'd ones. */
      if (ignore_srcdir)
     break;

      for (fp = &instack[indepth]; fp >= instack; fp--)
    {
      int n;
      char *ep,*nam;
#ifndef sgi
      extern char *rindex ();
#endif

      if ((nam = fp->fname) != NULL) {
        /* Found a named file.  Figure out dir of the file,
           and put it in front of the search list.  */
        dsp[0].next = stackp;
        stackp = dsp;
#ifndef VMS
        ep = (char*)rindex (nam, '/');
#else               /* VMS */
        ep = rindex (nam, ']');
        if (ep == NULL) ep = rindex (nam, '>');
        if (ep == NULL) ep = rindex (nam, ':');
        if (ep != NULL) ep++;
#endif              /* VMS */
        if (ep != NULL) {
          n = ep - nam;
          dsp[0].fname = (char *) alloca (n + 1);
          strncpy (dsp[0].fname, nam, n);
          dsp[0].fname[n] = '\0';
          if (n > max_include_len) max_include_len = n;
        } else {
          dsp[0].fname = 0; /* Current directory */
        }
        break;
      }
    }
      break;
    }
    goto fail;

  case '<':
    fend = fbeg;
    while (fend != limit && *fend != '>') fend++;
    if (*fend == '>' && fend + 1 == limit) {
      system_header_p = 1;
      /* If -I-, start with the first -I dir after the -I-.  */
      if (first_bracket_include)
    stackp = first_bracket_include;
      break;
    }
    goto fail;

  default:
  fail:
    if (retried) {
      error ("#include expects \"fname\" or <fname>");
      return 1;
    } else {
      trybuf = expand_to_temp_buffer (buf, limit, 0);
      buf = (U_CHAR *) alloca (trybuf.bufp - trybuf.buf + 1);
      bcopy (trybuf.buf, buf, trybuf.bufp - trybuf.buf);
      limit = buf + (trybuf.bufp - trybuf.buf);
      free (trybuf.buf);
      retried++;
      goto get_filename;
    }
  }

  flen = fend - fbeg;
  fname = (char *) alloca (max_include_len + flen + 2);
  /* + 2 above for slash and terminating null.  */

  /* If specified file name is absolute, just open it.  */

  if (*fbeg == '/') {
    strncpy (fname, fbeg, flen);
    fname[flen] = 0;
    f = _OPEN (fname, O_RDONLY, 0666);
  } else {
    /* Search directory path, trying to open the file.
       Copy each filename tried into FNAME.  */

    for (; stackp; stackp = stackp->next) {
      if (stackp->fname) {
    strcpy (fname, stackp->fname);
#ifdef WIN32
        if (fname[strlen(fname) - 1] != '\\')
#endif
    strcat (fname, "/");
    fname[strlen (fname) + flen] = 0;
      } else {
    fname[0] = 0;
      }
      strncat (fname, fbeg, flen);
#ifdef VMS
      /* Change this 1/2 Unix 1/2 VMS file specification into a
         full VMS file specification */
      if (stackp->fname && (stackp->fname[0] != 0)) {
    /* Fix up the filename */
    hack_vms_include_specification (fname);
      } else {
        /* This is a normal VMS filespec, so use it unchanged.  */
    strncpy (fname, fbeg, flen);
    fname[flen] = 0;
      }
#endif /* VMS */
#if 0
      USER_MESSAGE("acpp include: %s\n", fname);
#endif      
      if ((f = _OPEN (fname, O_RDONLY, 0666)) >= 0)
    break;
    }
  }

  if (f < 0) {
    strncpy (fname, fbeg, flen);
    fname[flen] = 0;
    error_from_errno (fname);
    /* For -M, add this file to the dependencies.  */
    if (print_deps > (system_header_p || (system_include_depth > 0))) {
      if (system_header_p) {
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
    if(warnings_off == 0) 
#endif
        warning ("nonexistent file <%.*s> omitted from dependency output",
         fend - fbeg, fbeg);
    }
      else
    {
#ifdef sgi
      if(deps_prefix) {
        deps_output("\n",1);
        deps_output(deps_prefix,strlen(deps_prefix));
      }
#endif
      deps_output (fbeg, fend - fbeg);
      deps_output (" ", 0);
    }
    }
  } else {

    /* Check to see if this include file is a once-only include file.
       If so, give up.  */

    struct file_name_list* ptr;

    for (ptr = dont_repeat_files; ptr; ptr = ptr->next) {
      if (!strcmp (ptr->fname, fname)) {
    close (f);
        return 1;               /* This file was once'd. */
      }
    }

    for (ptr = all_include_files; ptr; ptr = ptr->next) {
      if (!strcmp (ptr->fname, fname))
        break;              /* This file was included before. */
    }

    if (ptr == 0) {
      /* This is the first time for this file.  */
      /* Add it to list of files included.  */

      ptr = (struct file_name_list *) xmalloc (sizeof (struct file_name_list));
      ptr->next = all_include_files;
      all_include_files = ptr;
      ptr->fname = savestring (fname);

      /* For -M, add this file to the dependencies.  */
      if (print_deps > (system_header_p || (system_include_depth > 0))) {
#ifdef sgi
      if(deps_prefix) {
        deps_output("\n",1);
        deps_output(deps_prefix,strlen(deps_prefix));
      }
#endif
    deps_output (fname, strlen (fname));
    deps_output (" ", 0);
      }
#ifdef sgi
      if (deps_file)
      {
    printf("****************No MDupdate***************\n");
/*  MDupdate(deps_file, ptr->fname);*/
      }
#endif
    }   

    if (system_header_p)
      system_include_depth++;

    /* Actually process the file.  */
    finclude (f, fname, op);

    if (system_header_p)
      system_include_depth--;

    close (f);
  }
}

/* Process the contents of include file FNAME, already open on descriptor F,
   with output to OP.  */

finclude (f, fname, op)
     int f;
     char *fname;
     FILE_BUF *op;
{
  int st_mode;
  long st_size;
  long i;
  FILE_BUF *fp;         /* For input stack frame */
  int success = 0;

  CHECK_DEPTH (return 1;);

#ifdef DWARF_DM
    {
        FILE_BUF *ip;
        ip = &instack[indepth];
        dm_push_new(fname,ip->lineno);
    }
#endif
  if (file_size_and_mode (f, &st_mode, &st_size) < 0)
    goto nope;      /* Impossible? */

  fp = &instack[indepth + 1];
  bzero (fp, sizeof (FILE_BUF));
  fp->fname = fname;
  fp->length = 0;
  fp->lineno = 1;
  fp->if_stack = if_stack;

  if (st_mode & S_IFREG) {
    fp->buf = (U_CHAR *) alloca (st_size + 2);
    fp->bufp = fp->buf;

    /* Read the file contents, knowing that st_size is an upper bound
       on the number of bytes we can read.  */
    while (st_size > 0) {
      i = read (f, fp->buf + fp->length, st_size);
      if (i <= 0) {
    if (i == 0) break;
    goto nope;
      }
      fp->length += i;
      st_size -= i;
    }
  }
  else {
    /* Cannot count its file size before reading.
       First read the entire file into heap and
       copy them into buffer on stack. */

    U_CHAR *bufp;
    U_CHAR *basep;
    int bsize = 2000;

    st_size = 0;
    basep = (U_CHAR *) xmalloc (bsize + 2);
    bufp = basep;

    for (;;) {
      i = read (f, bufp, bsize - st_size);
      if (i < 0)
    goto nope;      /* error! */
      if (i == 0)
    break;  /* End of file */
      st_size += i;
      bufp += i;
      if (bsize == st_size) {   /* Buffer is full! */
      bsize *= 2;
      basep = (U_CHAR *) xrealloc (basep, bsize + 2);
      bufp = basep + st_size;   /* May have moved */
    }
    }
    fp->buf = (U_CHAR *) alloca (st_size + 2);
    fp->bufp = fp->buf;
    bcopy (basep, fp->buf, st_size);
    fp->length = st_size;
    free (basep);
  }

  if (!no_trigraphs)
    trigraph_pcp (fp);

  if (fp->length > 0 && fp->buf[fp->length-1] != '\n')
    fp->buf[fp->length++] = '\n';
  fp->buf[fp->length] = '\0';

  success = 1;
  indepth++;

  output_line_command (fp, op, 0, enter_file);
  rescan (op, 0);
  indepth--;
  output_line_command (&instack[indepth], op, 0, leave_file);

nope:
#ifdef DWARF_DM
    {
        dm_finish_current();
    }
#endif

  if (!success)
    perror_with_name (fname);

  close (f);
}

/* The arglist structure is built by do_define to tell
   collect_definition where the argument names begin.  That
   is, for a define like "#define f(x,y,z) foo+x-bar*y", the arglist
   would contain pointers to the strings x, y, and z.
   Collect_definition would then build a DEFINITION node,
   with reflist nodes pointing to the places x, y, and z had
   appeared.  So the arglist is just convenience data passed
   between these two routines.  It is not kept around after
   the current #define has been processed and entered into the
   hash table. */

struct arglist {
  struct arglist *next;
  U_CHAR *name;
  int length;
  int argno;
#ifdef sgi
  int duplicate_warned;
#endif
};

/* Process a #define command.
BUF points to the contents of the #define command, as a continguous string.
LIMIT points to the first character past the end of the definition.
KEYWORD is the keyword-table entry for #define.  */

do_define (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  U_CHAR *bp;           /* temp ptr into input buffer */
  U_CHAR *symname;      /* remember where symbol name starts */
  int sym_length;       /* and how long it is */
#ifdef DWARF_DM
    U_CHAR *end_name;
    FILE_BUF *ip;
#endif

  DEFINITION *defn;
  int arglengths = 0;       /* Accumulate lengths of arg names
                   plus number of args.  */
  int hashcode;

  bp = buf;

  while (is_hor_space[*bp])
    bp++;
  if (!is_idstart[*bp])
#ifdef sgi
    if(*bp == 0 || *bp == '\n') {
       error ("#define has no identifier (no macro name is present)");
    } else {
    if(warnings_off == 0)
       warning ("macro name in #define starts with an inappropriate character");
    }
#else
    warning ("macro name starts with a digit");
#endif

  symname = bp;         /* remember where it starts */
  while (is_idchar[*bp] && bp < limit) {
    bp++;
  }
  sym_length = bp - symname;

#ifdef sgi
  if(showdefines) {
    print_def_undef(op,"define",symname,sym_length);
  }
#endif

  /* lossage will occur if identifiers or control keywords are broken
     across lines using backslash.  This is not the right place to take
     care of that. */

  if (*bp == '(') {
    struct arglist *arg_ptrs = NULL;
    int argno = 0;

    bp++;           /* skip '(' */
    SKIP_WHITE_SPACE (bp);

    /* Loop over macro argument names.  */
    while (*bp != ')') {
      struct arglist *temp;

      temp = (struct arglist *) alloca (sizeof (struct arglist));
      temp->name = bp;
      temp->next = arg_ptrs;
      temp->argno = argno++;
#ifdef sgi
      temp->duplicate_warned = 0;
#endif
      arg_ptrs = temp;

      if (!is_idstart[*bp]
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
                           && warnings_off == 0
#endif
                                               )
    warning ("parameter name starts with a digit in #define");

      /* Find the end of the arg name.  */
      while (is_idchar[*bp]) {
    bp++;
      }
      temp->length = bp - temp->name;
      arglengths += temp->length + 2;
      SKIP_WHITE_SPACE (bp);
      if (temp->length == 0 || (*bp != ',' && *bp != ')')) {
    error ("badly punctuated parameter list in #define");
    goto nope;
      }
      if (*bp == ',') {
    bp++;
    SKIP_WHITE_SPACE (bp);
      }
      if (bp >= limit) {
    error ("unterminated parameter list in #define");
    goto nope;
      }
#ifdef sgi
    /* determine if a name is repeated. 3.8.3 page 90 line 16.*/
    /* This is n**2 in number of macro paramaters. Ugh. */
    {
        struct arglist *atemp;
     if(pedantic)
       for(atemp = arg_ptrs->next;  atemp; atemp = atemp->next) {
        if(arg_ptrs->length == atemp->length &
          atemp->duplicate_warned == 0 && 
          strncmp(arg_ptrs->name,atemp->name,atemp->length) == 0) {
            U_CHAR *str;
            str = (U_CHAR *)xmalloc(atemp->length+1);
            strncpy(str,atemp->name,atemp->length);
            str[atemp->length] = 0;
            if(warnings_off == 0)
                warning("duplicate macro parameter (%s) is illegal (ANSI 3.8.3)",str);
            free(str);
            atemp->duplicate_warned=1;
        }
       }
    }
#endif
    }
#ifdef DWARF_DM
    end_name = bp;
#endif

    ++bp;           /* skip paren */
    /* Skip exactly one space or tab if any.  */
#ifdef DWARF_DM
    ip = &instack[indepth];
 dm_add_define(symname,bp - symname, bp,limit - bp,ip->lineno);
#endif
    if (bp < limit && (*bp == ' ' || *bp == '\t')) ++bp;
    /* now everything from bp before limit is the definition. */
    defn = collect_expansion (bp, limit, argno, arg_ptrs);

    /* Now set defn->argnames to the result of concatenating
       the argument names in reverse order
       with comma-space between them.  */
    defn->argnames = (U_CHAR *) xmalloc (arglengths + 1);
    {
      struct arglist *temp;
      int i = 0;
      for (temp = arg_ptrs; temp; temp = temp->next) {
    bcopy (temp->name, &defn->argnames[i], temp->length);
    i += temp->length;
    if (temp->next != 0) {
      defn->argnames[i++] = ',';
      defn->argnames[i++] = ' ';
    }
      }
      defn->argnames[i] = 0;
    }
  } else {
    /* simple expansion or empty definition; gobble it */
    if (is_hor_space[*bp])
      ++bp;     /* skip exactly one blank/tab char */
    /* now everything from bp before limit is the definition. */
#ifdef DWARF_DM
    ip = &instack[indepth];
 dm_add_define(symname,sym_length, bp,limit - bp,ip->lineno);
#endif
    defn = collect_expansion (bp, limit, -1, 0);
    defn->argnames = (U_CHAR *) "";
  }

  hashcode = hashf (symname, sym_length, HASHSIZE);

  {
    HASHNODE *hp;
    if ((hp = lookup (symname, sym_length, hashcode)) != NULL) {
      if (hp->type != T_MACRO
      || compare_defs (defn, hp->value.defn)) {
    U_CHAR *msg;            /* what pain... */
    msg = (U_CHAR *) alloca (sym_length + 20);
    bcopy (symname, msg, sym_length);
    strcpy (msg + sym_length, " redefined");
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
    if(warnings_off == 0)
#endif
        warning (msg);
      }
      /* Replace the old definition.  */
      hp->type = T_MACRO;
      hp->value.defn = defn;
    } else
      install (symname, sym_length, T_MACRO, defn, hashcode);
  }

  return 0;

nope:

  return 1;
}

/*
 * return zero if two DEFINITIONs are isomorphic
 */
int
compare_defs (d1, d2)
     DEFINITION *d1, *d2;
{
  register struct reflist *a1, *a2;
  register U_CHAR *p1 = d1->expansion;
  register U_CHAR *p2 = d2->expansion;
  int first = 1;

  if (d1->nargs != d2->nargs)
    return 1;
  if (strcmp (d1->argnames, d2->argnames))
    return 1;
  for (a1 = d1->pattern, a2 = d2->pattern; a1 && a2;
       a1 = a1->next, a2 = a2->next) {
    if (!((a1->nchars == a2->nchars && ! strncmp (p1, p2, a1->nchars))
      || ! comp_def_part (first, p1, a1->nchars, p2, a2->nchars, 0))
    || a1->argno != a2->argno
    || a1->stringify != a2->stringify
    || a1->raw_before != a2->raw_before
    || a1->raw_after != a2->raw_after)
      return 1;
    first = 0;
    p1 += a1->nchars;
    p2 += a2->nchars;
  }
  if (a1 != a2)
    return 1;
  if (comp_def_part (first, p1, d1->length - (p1 - d1->expansion),
             p2, d2->length - (p2 - d2->expansion), 1))
    return 1;
  return 0;
}

/* Return 1 if two parts of two macro definitions are effectively different.
   One of the parts starts at BEG1 and has LEN1 chars;
   the other has LEN2 chars at BEG2.
   Any sequence of whitespace matches any other sequence of whitespace.
   FIRST means these parts are the first of a macro definition;
    so ignore leading whitespace entirely.
   LAST means these parts are the last of a macro definition;
    so ignore trailing whitespace entirely.  */

comp_def_part (first, beg1, len1, beg2, len2, last)
     int first;
     U_CHAR *beg1, *beg2;
     int len1, len2;
     int last;
{
  register U_CHAR *end1 = beg1 + len1;
  register U_CHAR *end2 = beg2 + len2;
  if (first) {
    while (beg1 != end1 && is_space[*beg1]) beg1++;
    while (beg2 != end2 && is_space[*beg2]) beg2++;
  }
  if (last) {
    while (beg1 != end1 && is_space[end1[-1]]) end1--;
    while (beg2 != end2 && is_space[end2[-1]]) end2--;
  }
  while (beg1 != end1 && beg2 != end2) {
    if (is_space[*beg1] && is_space[*beg2]) {
      while (beg1 != end1 && is_space[*beg1]) beg1++;
      while (beg2 != end2 && is_space[*beg2]) beg2++;
    } else if (*beg1 == *beg2) {
      beg1++; beg2++;
    } else break;
  }
  return (beg1 != end1) || (beg2 != end2);
}

/* Read a replacement list for a macro with parameters.
   Build the DEFINITION structure.
   Reads characters of text starting at BUF until LIMIT.
   ARGLIST specifies the formal parameters to look for
   in the text of the definition; NARGS is the number of args
   in that list, or -1 for a macro name that wants no argument list.
   MACRONAME is the macro name itself (so we can avoid recursive expansion)
   and NAMELEN is its length in characters.
   
Note that comments and backslash-newlines have already been deleted
from the argument.  */

/* Leading and trailing Space, Tab, etc. are converted to markers
   Newline Space, Newline Tab, etc.
   Newline Space makes a space in the final output
   but is discarded if stringified.  (Newline Tab is similar but
   makes a Tab instead.)

   If there is no trailing whitespace, a Newline Space is added at the end
   to prevent concatenation that would be contrary to the standard.  */

DEFINITION *
collect_expansion (buf, end, nargs, arglist)
     U_CHAR *buf, *end;
     int nargs;
     struct arglist *arglist;
{
  DEFINITION *defn;
  register U_CHAR *p, *limit, *lastp, *exp_p;
  struct reflist *endpat = NULL;
  /* Pointer to first nonspace after last ## seen.  */
  U_CHAR *concat = 0;
  /* Pointer to first nonspace after last single-# seen.  */
  U_CHAR *stringify = 0;
  int maxsize;
  int expected_delimiter = '\0';
#ifdef sgi
  int token_count = 0;
  int last_token_stringify = 0;
#endif

  /* Scan thru the replacement list, ignoring comments and quoted
     strings, picking up on the macro calls.  It does a linear search
     thru the arg list on every potential symbol.  Profiling might say
     that something smarter should happen. */

  if (end < buf)
    abort ();

  /* Find the beginning of the trailing whitespace.  */
  /* Find end of leading whitespace.  */
  limit = end;
  p = buf;
  while (p < limit && is_space[limit[-1]]) limit--;
  while (p < limit && is_space[*p]) p++;

  /* Allocate space for the text in the macro definition.
     Leading and trailing whitespace chars need 2 bytes each.
     Each other input char may or may not need 1 byte,
     so this is an upper bound.
     The extra 2 are for invented trailing newline-marker and final null.  */
  maxsize = (sizeof (DEFINITION)
         + 2 * (end - limit) + 2 * (p - buf)
         + (limit - p) + 3);
  defn = (DEFINITION *) xcalloc (1, maxsize);

  defn->nargs = nargs;
  exp_p = defn->expansion = (U_CHAR *) defn + sizeof (DEFINITION);
  lastp = exp_p;

  p = buf;

  /* Convert leading whitespace to Newline-markers.  */
  while (p < limit && is_space[*p]) {
    *exp_p++ = '\n';
    *exp_p++ = *p++;
  }

  /* Process the main body of the definition.  */
  while (p < limit) {
    int skipped_arg = 0;
    register U_CHAR c = *p++;

#ifdef sgi
    token_count++;
    last_token_stringify = 0;
#endif
    *exp_p++ = c;

    if (!traditional) {
      switch (c) {
      case '\'':
      case '\"':
    for (; p < limit && *p != c; p++) {
      *exp_p++ = *p;
      if (*p == '\\') {
        *exp_p++ = *++p;
      }
    }
    *exp_p++ = *p++;
    break;

    /* Special hack: if a \# is written in the #define
       include a # in the definition.  This is useless for C code
       but useful for preprocessing other things.  */

      case '\\':
    if (p < limit && *p == '#') {
      /* Pass through this # */
      exp_p--;
      *exp_p++ = *p++;
    } else if (p < limit) {
      /* Otherwise backslash goes through but makes next char ordinary.  */
      *exp_p++ = *p++;
    }
    break;

      case '#':
    if (p < limit && *p == '#') {
      /* ##: concatenate preceding and following tokens.  */
      /* Take out the first #, discard preceding whitespace.  */
      exp_p--;
      while (exp_p > lastp && is_hor_space[exp_p[-1]])
        --exp_p;
      /* Skip the second #.  */
      p++;
      /* Discard following whitespace.  */
      SKIP_WHITE_SPACE (p);
      concat = p;
#ifdef sgi
      last_token_stringify = 1;
      if(token_count == 1  && pedantic) {
         if(warnings_off == 0)
        warning("A ## preprocessing token must not be at the beginning of a replacement list (ANSI 3.8.3.3)");
      }
#endif
    } else {
      /* Single #: stringify following argument ref.
         Don't leave the # in the expansion.  */
      exp_p--;
      SKIP_WHITE_SPACE (p);
      if (p == limit || ! is_idstart[*p] || nargs <= 0)
        error ("# operator should be followed by a macro argument name\n");
      else
        stringify = p;
    }
    break;
      }
    } else {
      /* In -traditional mode, recognize arguments inside strings and
     and character constants, and ignore special properties of #.
     Arguments inside strings are considered "stringified", but no
     extra quote marks are supplied.  */
      switch (c) {
      case '\'':
      case '\"':
    if (expected_delimiter != '\0') {
      if (c == expected_delimiter)
        expected_delimiter = '\0';
    } else
      expected_delimiter = c;
    break;

      case '\\':
    /* Backslash quotes delimiters and itself, but not macro args.  */
    if (expected_delimiter != 0 && p < limit
        && (*p == expected_delimiter || *p == '\\')) {
      *exp_p++ = *p++;
      continue;
    }
    break;

      case '/':
    if (expected_delimiter != '\0') /* No comments inside strings.  */
      break;
    if (*p == '*') {
      /* If we find a comment that wasn't removed by handle_directive,
         this must be -traditional.  So replace the comment with
         nothing at all.  */
      exp_p--;
      p += 1;
      while (p < limit && !(p[-2] == '*' && p[-1] == '/'))
        p++;
      /* Mark this as a concatenation-point, as if it had been ##.  */
      concat = p;
    }
    break;
      }
    }

    if (is_idchar[c] && nargs > 0) {
      U_CHAR *id_beg = p - 1;
      int id_len;

      --exp_p;
      while (p != limit && is_idchar[*p]) p++;
      id_len = p - id_beg;

      if (is_idstart[c]) {
    register struct arglist *arg;

    for (arg = arglist; arg != NULL; arg = arg->next) {
      struct reflist *tpat;

      if (arg->name[0] == c
          && arg->length == id_len
          && strncmp (arg->name, id_beg, id_len) == 0) {
        /* make a pat node for this arg and append it to the end of
           the pat list */
        tpat = (struct reflist *) xmalloc (sizeof (struct reflist));
        tpat->next = NULL;
        tpat->raw_before = concat == id_beg;
        tpat->raw_after = 0;
        tpat->stringify = (traditional ? expected_delimiter != '\0'
                   : stringify == id_beg);

        if (endpat == NULL)
          defn->pattern = tpat;
        else
          endpat->next = tpat;
        endpat = tpat;

        tpat->argno = arg->argno;
        tpat->nchars = exp_p - lastp;
        {
          register U_CHAR *p1 = p;
          SKIP_WHITE_SPACE (p1);
          if (p1 + 2 <= limit && p1[0] == '#' && p1[1] == '#')
        tpat->raw_after = 1;
        }
        lastp = exp_p;  /* place to start copying from next time */
        skipped_arg = 1;
        break;
      }
    }
      }

      /* If this was not a macro arg, copy it into the expansion.  */
      if (! skipped_arg) {
    register U_CHAR *lim1 = p;
    p = id_beg;
    while (p != lim1)
      *exp_p++ = *p++;
    if (stringify == id_beg)
      error ("# operator should be followed by a macro argument name\n");
      }
    }
  }
#ifdef sgi
  if(last_token_stringify && pedantic)
     if(warnings_off == 0)
    warning("A ## preprocessing token must not be at the end of a replacement list (ANSI 3.8.3.3)");
#endif

  if (limit < end) {
    /* Convert trailing whitespace to Newline-markers.  */
    while (limit < end && is_space[*limit]) {
      *exp_p++ = '\n';
      *exp_p++ = *limit++;
    }
  } else if (!traditional) {
    /* There is no trailing whitespace, so invent some.  */
    *exp_p++ = '\n';
    *exp_p++ = ' ';
  }

  *exp_p = '\0';

  defn->length = exp_p - defn->expansion;

  /* Crash now if we overrun the allocated size.  */
  if (defn->length + 1 > maxsize)
    abort ();

#if 0
/* This isn't worth the time it takes.  */
  /* give back excess storage */
  defn->expansion = (U_CHAR *) xrealloc (defn->expansion, defn->length + 1);
#endif

  return defn;
}

/*
 * interpret #line command.  Remembers previously seen fnames
 * in its very own hash table.
 */
#define FNAME_HASHSIZE 37

do_line (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  register U_CHAR *bp;
  FILE_BUF *ip = &instack[indepth];
  FILE_BUF tem;
  int new_lineno;
  enum file_change_code file_change = same_file;

  /* Expand any macros.  */
  tem = expand_to_temp_buffer (buf, limit, 0);

  /* Point to macroexpanded line, which is null-terminated now.  */
  bp = tem.buf;
  SKIP_WHITE_SPACE (bp);

  if (!isdigit (*bp)) {
    error ("invalid format #line command");
    return 1;
  }

  /* The Newline at the end of this line remains to be processed.
     To put the next line at the specified line number,
     we must store a line number now that is one less.  */
  new_lineno = atoi (bp) - 1;

  /* skip over the line number.  */
  while (isdigit (*bp))
    bp++;
  if (*bp && !is_space[*bp]) {
    error ("invalid format #line command");
    return 1;
  }
    
  SKIP_WHITE_SPACE (bp);

  if (*bp == '\"') {
    static HASHNODE *fname_table[FNAME_HASHSIZE];
    HASHNODE *hp, **hash_bucket;
    U_CHAR *fname;
    int fname_length;

    fname = ++bp;

    while (*bp && *bp != '\"')
      bp++;
    if (*bp != '\"') {
      error ("invalid format #line command");
      return 1;
    }

    fname_length = bp - fname;

    bp++;
    SKIP_WHITE_SPACE (bp);
    if (*bp) {
      if (*bp == '1')
    file_change = enter_file;
      else if (*bp == '2')
    file_change = leave_file;
      else {
    error ("invalid format #line command");
    return 1;
      }

      bp++;
      SKIP_WHITE_SPACE (bp);
      if (*bp) {
    error ("invalid format #line command");
    return 1;
      }
    }

    hash_bucket =
      &fname_table[hashf (fname, fname_length, FNAME_HASHSIZE)];
    for (hp = *hash_bucket; hp != NULL; hp = hp->next)
      if (hp->length == fname_length &&
      strncmp (hp->value.cpval, fname, fname_length) == 0) {
    ip->fname = hp->value.cpval;
    break;
      }
    if (hp == 0) {
      /* Didn't find it; cons up a new one.  */
      hp = (HASHNODE *) xcalloc (1, sizeof (HASHNODE) + fname_length + 1);
      hp->next = *hash_bucket;
      *hash_bucket = hp;

      hp->length = fname_length;
      ip->fname = hp->value.cpval = ((char *) hp) + sizeof (HASHNODE);
      bcopy (fname, hp->value.cpval, fname_length);
    }
  } else if (*bp) {
    error ("invalid format #line command");
    return 1;
  }

  ip->lineno = new_lineno;
  output_line_command (ip, op, 0, file_change);
  check_expand (op, ip->length - (ip->bufp - ip->buf));
}

/*
 * remove all definitions of symbol from symbol table.
 * according to un*x /lib/cpp, it is not an error to undef
 * something that has no definitions, so it isn't one here either.
 */
do_undef (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  HASHNODE *hp;
#ifdef sgi
    U_CHAR *start_name;
#endif

  SKIP_WHITE_SPACE (buf);
#ifdef sgi
  if (!is_idstart[*buf]) {
    if(*buf == 0 || *buf == '\n') {
       error ("#undef has no identifier (no macro name is present)");
    } else {
      if(warnings_off == 0)
       warning ("macro name in #undef starts with an inappropriate character");
    }
  }
#endif

#ifndef sgi
  while ((hp = lookup (buf, -1, -1)) != NULL) {
    if (hp->type != T_MACRO)
      error ("undefining `%s'", hp->name);
    delete_macro (hp);
  }
#else
  while ((hp = lookup (buf, -1, -1)) != NULL) {
    if (hp->type != T_MACRO)
      error ("undefining `%s'", hp->name);
    delete_macro (hp);
  } /* silent about not finding identifier */
  start_name = buf;
  while (is_idchar[*buf] && buf < limit) {
    buf++;
  }
#ifdef DWARF_DM
{
    FILE_BUF *ip;
    ip = &instack[indepth];
    dm_add_undef(start_name, buf - start_name,ip->lineno);
}
#endif
  if(showdefines) {
    print_def_undef(op,"undef",start_name,buf - start_name);
  }
  if (pedantic) {
    SKIP_WHITE_SPACE (buf);
    if (buf != limit)
     if(warnings_off == 0)
      warning ("text following #undef identifier  violates ANSI standard");
  }
#endif
}

/*
 * Report a fatal error detected by the program we are processing.
 * Use the text of the line in the error message, then terminate.
 * (We use error() because it prints the filename & line#.)
 */
do_error (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int length = limit - buf;
  char *copy = (char *) xmalloc (length + 1);
  bcopy (buf, copy, length);
  copy[length] = 0;
  SKIP_WHITE_SPACE (copy);
  error ("#error %s", copy);
  return (FATAL_EXIT_CODE);
}

/* Remember the name of the current file being read from so that we can
   avoid ever including it again.  */

do_once ()
{
  int i;
  FILE_BUF *ip = NULL;

  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != NULL) {
      ip = &instack[i];
      break;
    }

  if (ip != NULL)
    {
      struct file_name_list *new;

      new = (struct file_name_list *) xmalloc (sizeof (struct file_name_list));
      new->next = dont_repeat_files;
      dont_repeat_files = new;
      new->fname = savestring (ip->fname);
    }
}

/* #pragma and its argument line have already been copied to the output file.
   Here just check for recognized pragmas.  */

do_pragma (buf, limit)
     U_CHAR *buf, *limit;
{
  while (*buf == ' ' || *buf == '\t')
    buf++;
  if (!strncmp (buf, "once", 4))
    do_once ();
}

#if 0
/* This was a fun hack, but #pragma seems to start to be useful.
   By failing to recognize it, we pass it through unchanged to cc1.  */

/*
 * the behavior of the #pragma directive is implementation defined.
 * this implementation defines it as follows.
 */
do_pragma ()
{
  close (0);
  if (_OPEN ("/dev/tty", O_RDONLY, 0666) != 0)
    goto nope;
  close (1);
  if (_OPEN ("/dev/tty", O_WRONLY, 0666) != 1)
    goto nope;
  execl ("/usr/games/hack", "#pragma", 0);
  execl ("/usr/games/rogue", "#pragma", 0);
  execl ("/usr/new/emacs", "-f", "hanoi", "9", "-kill", 0);
  execl ("/usr/local/emacs", "-f", "hanoi", "9", "-kill", 0);
nope:
  fatal ("You are in a maze of twisty compiler features, all different");
}
#endif

/* Just ignore #ident, on systems where we define it at all.  */
do_ident ()
{
#ifdef sgi
  if (pedantic > 1 )
   if(warnings_off == 0)
    warning ("ANSI C does not allow #ident; #ident ignored ");
#else
  if (pedantic )
    error ("ANSI C does not allow #ident");
#endif
}

/* Just ignore #sccs, on systems where we define it at all.  */
do_sccs ()
{
  if (pedantic)
    error ("ANSI C does not allow #sccs");
}

/*
 * handle #if command by
 *   1) inserting special `defined' keyword into the hash table
 *  that gets turned into 0 or 1 by special_symbol (thus,
 *  if the luser has a symbol called `defined' already, it won't
 *      work inside the #if command)
 *   2) rescan the input into a temporary output buffer
 *   3) pass the output buffer to the yacc parser and collect a value
 *   4) clean up the mess left from steps 1 and 2.
 *   5) call conditional_skip to skip til the next #endif (etc.),
 *      or not, depending on the value from step 3.
 */

do_if (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int value;
  FILE_BUF *ip = &instack[indepth];

  value = eval_if_expression (buf, limit - buf);
  conditional_skip (ip, value == 0, T_IF);
}

/*
 * handle a #elif directive by not changing  if_stack  either.
 * see the comment above do_else.
 */

do_elif (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int value;
  FILE_BUF *ip = &instack[indepth];

  if (if_stack == instack[indepth].if_stack) {
    error ("#elif not within a conditional");
    return 1;
  } else {
    if (if_stack->type != T_IF && if_stack->type != T_ELIF) {
      error ("#elif after #else");
      USER_MESSAGE(" (matches line %d", if_stack->lineno);
      if (if_stack->fname != NULL && ip->fname != NULL &&
      strcmp (if_stack->fname, ip->fname) != 0)
    USER_MESSAGE(", file %s", if_stack->fname);
      USER_MESSAGE(")\n");
    }
    if_stack->type = T_ELIF;
  }

  if (if_stack->if_succeeded)
    skip_if_group (ip, 0);
  else {
    value = eval_if_expression (buf, limit - buf);
    if (value == 0)
      skip_if_group (ip, 0);
    else {
      ++if_stack->if_succeeded; /* continue processing input */
      output_line_command (ip, op, 1, same_file);
    }
  }
}

#if 0
/* THIS DOESN'T EVEN BEGIN TO WORK (yet) */
/*
 * return true if what we are looking at is an assertion expression,
 */
int
is_assertion(buf)
     U_CHAR *buf;
{
  int c;
  do {
    c = *buf++;
  } while(isspace(c) || c == '!'); /* can negate an arbitrary number of
                      times */

  return (c == '#');
}
#endif

/*
 * evaluate a #if expression in BUF, of length LENGTH,
 * then parse the result as a C expression and return the value as an int.
 */
int
eval_if_expression (buf, length)
     U_CHAR *buf;
     int length;
{
  FILE_BUF temp_obuf;
  HASHNODE *save_defined;
  int value;

  save_defined = install ("defined", -1, T_SPEC_DEFINED, 0, -1);
  temp_obuf = expand_to_temp_buffer (buf, buf + length, 0);
  delete_macro (save_defined);  /* clean up special symbol */

  /* the AT&T preprocessor made an extension to ANSI which proves to
   * be sufficiently useful that I have decided to go ahead and add
   * it eventually.  It works by:
   *
   *   # assert assertion-name ( preprocessor tokens )
   *   # unassert assertion-name
   *
   * Having made an assertion of this form, one can use the assertion
   * in a preprocessor conditional by saying
   *
   *   # if #assertion-name(preprocessor-tokens)
   *
   * If the assertion has been made with the preprocessor tokens in
   * question, the result is to return true.  If the preprocessor
   * tokens don't match, or the assertion hasn't been made, the value is
   * considered to be false.
   */

#if 0
  value = (is_assertion(temp_obuf.buf) ?
       parse_assertion_extension (temp_obuf.buf) :
       parse_c_expression (temp_obuf.buf));
#else
  value = parse_c_expression (temp_obuf.buf);
#endif
  
  free (temp_obuf.buf);

  return value;
}

/*
 * routine to handle ifdef/ifndef.  Try to look up the symbol,
 * then do or don't skip to the #endif/#else/#elif depending
 * on what directive is actually being processed.
 */
do_xifdef (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int skip;
  FILE_BUF *ip = &instack[indepth];
  U_CHAR *end; 

  /* Discard leading and trailing whitespace.  */
  SKIP_WHITE_SPACE (buf);
  while (limit != buf && is_hor_space[limit[-1]]) limit--;

  /* Find the end of the identifier at the beginning.  */
  for (end = buf; is_idchar[*end]; end++);

  if (end == buf) {
    skip = (keyword->type == T_IFDEF);
    if (! traditional)
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
     if(warnings_off == 0)
#endif
      warning (end == limit ? "#%s with no argument"
           : "#%s argument starts with punctuation",
           keyword->name);
  } else {
    if (pedantic && buf[0] >= '0' && buf[0] <= '9') {
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
     if(warnings_off == 0)
#endif
      warning ("#%s argument starts with a digit", keyword->name);
    } else if (end != limit && !traditional) {
#if defined sgi || defined WIN32 || defined __linux__ || defined __APPLE__
     if(warnings_off == 0)
#endif
      warning ("garbage at end of #%s argument", keyword->name);
    }

    skip = (lookup (buf, end-buf, -1) == NULL) ^ (keyword->type == T_IFNDEF);
  }

  conditional_skip (ip, skip, T_IF);
}

/*
 * push TYPE on stack; then, if SKIP is nonzero, skip ahead.
 */
void
conditional_skip (ip, skip, type)
     FILE_BUF *ip;
     int skip;
     enum node_type type;
{
  IF_STACK_FRAME *temp;

  temp = (IF_STACK_FRAME *) xcalloc (1, sizeof (IF_STACK_FRAME));
  temp->fname = ip->fname;
  temp->lineno = ip->lineno;
  temp->next = if_stack;
  if_stack = temp;

  if_stack->type = type;

  if (skip != 0) {
    skip_if_group (ip, 0);
    return;
  } else {
    ++if_stack->if_succeeded;
    output_line_command (ip, &outbuf, 1, same_file);
  }
}

/*
 * skip to #endif, #else, or #elif.  adjust line numbers, etc.
 * leaves input ptr at the sharp sign found.
 * If ANY is nonzero, return at next directive of any sort.
 */
void
skip_if_group (ip, any)
     FILE_BUF *ip;
     int any;
{
  register U_CHAR *bp = ip->bufp, *cp;
  register U_CHAR *endb = ip->buf + ip->length;
  struct directive *kt;
  IF_STACK_FRAME *save_if_stack = if_stack; /* don't pop past here */
  U_CHAR *beg_of_line = bp;

  while (bp < endb) {
    switch (*bp++) {
    case '/':           /* possible comment */
      if (*bp == '\\' && bp[1] == '\n')
    newline_fix (bp);
      if (*bp == '*'
      || (cplusplus && *bp == '/')) {
    ip->bufp = ++bp;
    bp = skip_to_end_of_comment (ip, &ip->lineno);
      }
      break;
    case '\"':
    case '\'':
      if (!traditional)
    bp = skip_quoted_string (bp - 1, endb, ip->lineno, &ip->lineno, 0, 0);
      break;
    case '\\':
      /* Char after backslash loses its special meaning.  */
      if (bp < endb) {
    if (*bp == '\n')
      ++ip->lineno;     /* But do update the line-count.  */
    bp++;
      }
      break;
    case '\n':
      ++ip->lineno;
      beg_of_line = bp;
      break;
    case '#':
      ip->bufp = bp - 1;

      /* # keyword: a # must be first nonblank char on the line */
      if (beg_of_line == 0)
    break;
      /* Scan from start of line, skipping whitespace, comments
     and backslash-newlines, and see if we reach this #.
     If not, this # is not special.  */
      bp = beg_of_line;
      while (1) {
    if (is_hor_space[*bp])
      bp++;
    else if (*bp == '\\' && bp[1] == '\n')
      bp += 2;
    else if (*bp == '/' && bp[1] == '*') {
      bp += 2;
      while (!(*bp == '*' && bp[1] == '/'))
        bp++;
      bp += 2;
    }
    else if (cplusplus && *bp == '/' && bp[1] == '/') {
      bp += 2;
      while (*bp++ != '\n') ;
        }
    else break;
      }
      if (bp != ip->bufp) {
    bp = ip->bufp + 1;  /* Reset bp to after the #.  */
    break;
      }

      bp = ip->bufp + 1;        /* point at '#' */

      /* Skip whitespace and \-newline.  */
      while (1) {
    if (is_hor_space[*bp])
      bp++;
    else if (*bp == '\\' && bp[1] == '\n')
      bp += 2;
#ifdef sgi
    /*  cypress 66662.   
        In false if, comment after # before endif not handled before 
    */
    else if (*bp == '/' && bp[1] == '*') {
      bp += 2;
      while (!(*bp == '*' && bp[1] == '/'))
        bp++;
      bp += 2;
    }
#endif
    else break;
      }

      cp = bp;

      /* Now find end of directive name.
     If we encounter a backslash-newline, exchange it with any following
     symbol-constituents so that we end up with a contiguous name.  */

      while (1) {
    if (is_idchar[*bp])
      bp++;
    else {
      if (*bp == '\\' && bp[1] == '\n')
        name_newline_fix (bp);
      if (is_idchar[*bp])
        bp++;
      else break;
    }
      }

      for (kt = directive_table; kt->length >= 0; kt++) {
    IF_STACK_FRAME *temp;
    if (strncmp (cp, kt->name, kt->length) == 0
        && !is_idchar[cp[kt->length]]) {

      /* If we are asked to return on next directive,
         do so now.  */
      if (any)
        return;

      switch (kt->type) {
      case T_IF:
      case T_IFDEF:
      case T_IFNDEF:
        temp = (IF_STACK_FRAME *) xcalloc (1, sizeof (IF_STACK_FRAME));
        temp->next = if_stack;
        if_stack = temp;
        temp->lineno = ip->lineno;
        temp->fname = ip->fname;
        temp->type = kt->type;
        break;
      case T_ELSE:
      case T_ENDIF:
        if (pedantic && if_stack != save_if_stack)
          validate_else (bp);
      case T_ELIF:
        if (if_stack == instack[indepth].if_stack) {
          error ("#%s not within a conditional", kt->name);
          break;
        }
        else if (if_stack == save_if_stack)
          return;       /* found what we came for */

        if (kt->type != T_ENDIF) {
          if (if_stack->type == T_ELSE)
        error ("#else or #elif after #else");
          if_stack->type = kt->type;
          break;
        }

        temp = if_stack;
        if_stack = if_stack->next;
        free (temp);
        break;
      }
      break;
    }
      }
    }
  }
  ip->bufp = bp;
  /* after this returns, rescan will exit because ip->bufp
     now points to the end of the buffer.
     rescan is responsible for the error message also.  */
}

/*
 * handle a #else directive.  Do this by just continuing processing
 * without changing  if_stack ;  this is so that the error message
 * for missing #endif's etc. will point to the original #if.  It
 * is possible that something different would be better.
 */
do_else (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  FILE_BUF *ip = &instack[indepth];

  if (pedantic) {
    SKIP_WHITE_SPACE (buf);
    if (buf != limit)
#ifdef sgi
    if(warnings_off == 0) {
    ONE_TIME_ENDIF(
      warning ("text following #else violates ANSI standard")
    );
    }
#else
      warning ("text following #else violates ANSI standard");
#endif
  }

  if (if_stack == instack[indepth].if_stack) {
    error ("#else not within a conditional");
    return 1;
  } else {
    if (if_stack->type != T_IF && if_stack->type != T_ELIF) {
      error ("#else after #else");
      USER_MESSAGE(" (matches line %d", if_stack->lineno);
      if (strcmp (if_stack->fname, ip->fname) != 0)
    USER_MESSAGE(", file %s", if_stack->fname);
      USER_MESSAGE(")\n");
    }
    if_stack->type = T_ELSE;
  }

  if (if_stack->if_succeeded)
    skip_if_group (ip, 0);
  else {
    ++if_stack->if_succeeded;   /* continue processing input */
    output_line_command (ip, op, 1, same_file);
  }
}

/*
 * unstack after #endif command
 */
do_endif (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  if (pedantic) {
    SKIP_WHITE_SPACE (buf);
    if (buf != limit)
#ifdef sgi
    if(warnings_off == 0) {
    ONE_TIME_ENDIF(
      warning ("text following #endif violates ANSI standard")
    );
    }
#else
      warning ("text following #endif violates ANSI standard");
#endif
  }

  if (if_stack == instack[indepth].if_stack)
    error ("unbalanced #endif");
  else {
    IF_STACK_FRAME *temp = if_stack;
    if_stack = if_stack->next;
    free (temp);
    output_line_command (&instack[indepth], op, 1, same_file);
  }
}

/* When an #else or #endif is found while skipping failed conditional,
   if -pedantic was specified, this is called to warn about text after
   the command name.  P points to the first char after the command name.  */

validate_else (p)
     register U_CHAR *p;
{
  /* Advance P over whitespace and comments.  */
  while (1) {
    if (*p == '\\' && p[1] == '\n')
      p += 2;
    if (is_hor_space[*p])
      p++;
    else if (*p == '/') {
      if (p[1] == '\\' && p[2] == '\n')
    newline_fix (p + 1);
      if (p[1] == '*') {
    p += 2;
    /* Don't bother warning about unterminated comments
       since that will happen later.  Just be sure to exit.  */
    while (*p) {
      if (p[1] == '\\' && p[2] == '\n')
        newline_fix (p + 1);
      if (*p == '*' && p[1] == '/') {
        p += 2;
        break;
      }
      p++;
    }
      }
      else if (cplusplus && p[1] == '/') {
    p += 2;
    while (*p && *p++ != '\n') ;
      }
    } else break;
  }
  if (*p && *p != '\n')
#ifdef sgi
    if(warnings_off == 0) {
    ONE_TIME_ENDIF(
    warning ("text following #else or #endif violates ANSI standard")
    );
    }
#else
    warning ("text following #else or #endif violates ANSI standard");
#endif
}

/*
 * Skip a comment, assuming the input ptr immediately follows the
 * initial slash-star.  Bump line counter as necessary.
 * (The canonical line counter is &ip->lineno).
 * Don't use this routine (or the next one) if bumping the line
 * counter is not sufficient to deal with newlines in the string.
 */
U_CHAR *
skip_to_end_of_comment (ip, line_counter)
     register FILE_BUF *ip;
     int *line_counter;     /* place to remember newlines, or NULL */
{
  register U_CHAR *limit = ip->buf + ip->length;
  register U_CHAR *bp = ip->bufp;
  FILE_BUF *op = &outbuf;   /* JF */
  int output = put_out_comments && !line_counter;

    /* JF this line_counter stuff is a crock to make sure the
       comment is only put out once, no matter how many times
       the comment is skipped.  It almost works */
  if (output) {
    *op->bufp++ = '/';
    *op->bufp++ = '*';
  }
  if (cplusplus && bp[-1] == '/') {
    if (output) {
      while (bp < limit)
    if ((*op->bufp++ = *bp++) == '\n') {
      bp--;
      break;
    }
      op->bufp[-1] = '*';
      *op->bufp++ = '/';
      *op->bufp++ = '\n';
    } else {
      while (bp < limit) {
    if (*bp++ == '\n') {
      bp--;
      break;
    }
      }
    }
    ip->bufp = bp;
    return bp;
  }
  while (bp < limit) {
    if (output)
      *op->bufp++ = *bp;
    switch (*bp++) {
    case '\n':
      if (line_counter != NULL)
    ++*line_counter;
      if (output)
    ++op->lineno;
      break;
    case '*':
      if (*bp == '\\' && bp[1] == '\n')
    newline_fix (bp);
      if (*bp == '/') {
        if (output)
      *op->bufp++ = '/';
    ip->bufp = ++bp;
    return bp;
      }
      break;
    }
  }
  ip->bufp = bp;
  return bp;
}

/*
 * Skip over a quoted string.  BP points to the opening quote.
 * Returns a pointer after the closing quote.  Don't go past LIMIT.
 * START_LINE is the line number of the starting point (but it need
 * not be valid if the starting point is inside a macro expansion).
 *
 * The input stack state is not changed.
 *
 * If COUNT_NEWLINES is nonzero, it points to an int to increment
 * for each newline passed.
 *
 * If BACKSLASH_NEWLINES_P is nonzero, store 1 thru it
 * if we pass a backslash-newline.
 *
 * If EOFP is nonzero, set *EOFP to 1 if the string is unterminated.
 */
U_CHAR *
skip_quoted_string (bp, limit, start_line, count_newlines, backslash_newlines_p, eofp)
     register U_CHAR *bp;
     register U_CHAR *limit;
     int start_line;
     int *count_newlines;
     int *backslash_newlines_p;
     int *eofp;
{
  register U_CHAR c, match;

  match = *bp++;
  while (1) {
    if (bp >= limit) {
      error_with_line (line_for_error (start_line),
               "unterminated string or character constant");
      if (eofp)
    *eofp = 1;
      break;
    }
    c = *bp++;
    if (c == '\\') {
      while (*bp == '\\' && bp[1] == '\n') {
    if (backslash_newlines_p)
      *backslash_newlines_p = 1;
    if (count_newlines)
      ++*count_newlines;
    bp += 2;
      }
      if (*bp == '\n' && count_newlines) {
    if (backslash_newlines_p)
      *backslash_newlines_p = 1;
    ++*count_newlines;
      }
      bp++;
    } else if (c == '\n') {
      if (match == '\'') {
    error_with_line (line_for_error (start_line),
             "unterminated character constant");
    bp--;
    if (eofp)
      *eofp = 1;
    break;
      }
      if (traditional) {    /* Unterminated strings are 'legal'.  */
    if (eofp)
      *eofp = 1;
    break;
      }
      if (count_newlines)
    ++*count_newlines;
    } else if (c == match)
      break;
  }
  return bp;
}

/*
 * write out a #line command, for instance, after an #include file.
 * If CONDITIONAL is nonzero, we can omit the #line if it would
 * appear to be a no-op, and we can output a few newlines instead
 * if we want to increase the line number by a small amount.
 * FILE_CHANGE says whether we are entering a file, leaving, or neither.
 */

void
output_line_command (ip, op, conditional, file_change)
     FILE_BUF *ip, *op;
     int conditional;
     enum file_change_code file_change;
{
  int len;
  char line_cmd_buf[500];

  if (no_line_commands
      || ip->fname == NULL
      || no_output) {
    op->lineno = ip->lineno;
    return;
  }

  if (conditional) {
    if (ip->lineno == op->lineno)
      return;

    /* If the inherited line number is a little too small,
       output some newlines instead of a #line command.  */
    if (ip->lineno > op->lineno && ip->lineno < op->lineno + 8) {
      check_expand (op, 10);
      while (ip->lineno > op->lineno) {
    *op->bufp++ = '\n';
    op->lineno++;
      }
      return;
    }
  }

#ifdef OUTPUT_LINE_COMMANDS
  sprintf (line_cmd_buf, "#line %d \"%s\"", ip->lineno, ip->fname);
#else
  sprintf (line_cmd_buf, "# %d \"%s\"", ip->lineno, ip->fname);
#endif
#if  0
  /* on sgi machines this gives the compiler indigestion, which in turn
     upsets dbx... */
  if (file_change != same_file)
    strcat (line_cmd_buf, file_change == enter_file ? " 1" : " 2");
#endif
  len = strlen (line_cmd_buf);
  line_cmd_buf[len++] = '\n';
  check_expand (op, len + 1);
  if (op->bufp > op->buf && op->bufp[-1] != '\n')
    *op->bufp++ = '\n';
  bcopy (line_cmd_buf, op->bufp, len);
  op->bufp += len;
  op->lineno = ip->lineno;
}

/* This structure represents one parsed argument in a macro call.
   `raw' points to the argument text as written (`raw_length' is its length).
   `expanded' points to the argument's macro-expansion
   (its length is `expand_length').
   `stringified_length' is the length the argument would have
   if stringified.
   `free1' and `free2', if nonzero, point to blocks to be freed
   when the macro argument data is no longer needed.  */

struct argdata {
  U_CHAR *raw, *expanded;
  int raw_length, expand_length;
  int stringified_length;
  U_CHAR *free1, *free2;
  char newlines;
  char comments;
};

/* Expand a macro call.
   HP points to the symbol that is the macro being called.
   Put the result of expansion onto the input stack
   so that subsequent input by our caller will use it.

   If macro wants arguments, caller has already verified that
   an argument list follows; arguments come from the input stack.  */

void
macroexpand (hp, op)
     HASHNODE *hp;
     FILE_BUF *op;
{
  int nargs;
  DEFINITION *defn = hp->value.defn;
  register U_CHAR *xbuf;
  int xbuf_len;
  int start_line = instack[indepth].lineno;

  CHECK_DEPTH (return;);

  /* it might not actually be a macro.  */
  if (hp->type != T_MACRO) {
    special_symbol (hp, op);
    return;
  }

  nargs = defn->nargs;

  if (nargs >= 0) {
    register int i;
    struct argdata *args;
    char *parse_error = 0;

    args = (struct argdata *) alloca ((nargs + 1) * sizeof (struct argdata));

    for (i = 0; i < nargs; i++) {
      args[i].raw = args[i].expanded = (U_CHAR *) "";
      args[i].raw_length = args[i].expand_length
    = args[i].stringified_length = 0;
      args[i].free1 = args[i].free2 = 0;
    }

    /* Parse all the macro args that are supplied.  I counts them.
       The first NARGS args are stored in ARGS.
       The rest are discarded.  */
    i = 0;
    do {
      /* Discard the open-parenthesis or comma before the next arg.  */
      ++instack[indepth].bufp;
      if (parse_error = macarg ((i < nargs || (nargs == 0 && i == 0)) ? &args[i] : 0))
    {
      error_with_line (line_for_error (start_line), parse_error);
      break;
    }
      i++;
    } while (*instack[indepth].bufp != ')');

    if (nargs == 0 && i == 1) {
      register U_CHAR *bp = args[0].raw;
      register U_CHAR *lim = bp + args[0].raw_length;
      while (bp != lim && is_space[*bp]) bp++;
      if (bp != lim)
    error ("arguments given to macro `%s'", hp->name);
    } else if (i < nargs)
      error ("only %d args to macro `%s'", i, hp->name);
    else if (i > nargs)
      error ("too many (%d) args to macro `%s'", i, hp->name);

    /* Swallow the closeparen.  */
    ++instack[indepth].bufp;

    /* If macro wants zero args, we parsed the arglist for checking only.
       Read directly from the macro definition.  */
    if (nargs == 0) {
      xbuf = defn->expansion;
      xbuf_len = defn->length;
    } else {
      register U_CHAR *exp = defn->expansion;
      register int offset;  /* offset in expansion,
                   copied a piece at a time */
      register int totlen;  /* total amount of exp buffer filled so far */

      register struct reflist *ap;

      /* Macro really takes args.  Compute the expansion of this call.  */

      /* Compute length in characters of the macro's expansion.  */
      xbuf_len = defn->length;
      for (ap = defn->pattern; ap != NULL; ap = ap->next) {
    if (ap->stringify)
      xbuf_len += args[ap->argno].stringified_length;
    else if (ap->raw_before || ap->raw_after)
      xbuf_len += args[ap->argno].raw_length;
    else
      xbuf_len += args[ap->argno].expand_length;
      }

      xbuf = (U_CHAR *) xmalloc (xbuf_len + 1);

      /* Generate in XBUF the complete expansion
     with arguments substituted in.
     TOTLEN is the total size generated so far.
     OFFSET is the index in the definition
     of where we are copying from.  */
      offset = totlen = 0;
      for (ap = defn->pattern; ap != NULL; ap = ap->next) {
    register struct argdata *arg = &args[ap->argno];

    for (i = 0; i < ap->nchars; i++)
      xbuf[totlen++] = exp[offset++];

    if (ap->stringify != 0) {
      int arglen = arg->raw_length;
      int escaped = 0;
      int in_string = 0;
      int c;
      i = 0;
      while (i < arglen
         && (c = arg->raw[i], is_space[c]))
        i++;
      while (i < arglen
         && (c = arg->raw[arglen - 1], is_space[c]))
        arglen--;
      if (!traditional)
        xbuf[totlen++] = '\"'; /* insert beginning quote */
      for (; i < arglen; i++) {
        c = arg->raw[i];

        /* Special markers Newline Space
           generate nothing for a stringified argument.  */
        if (c == '\n' && arg->raw[i+1] != '\n') {
          i++;
          continue;
        }

        /* Internal sequences of whitespace are replaced by one space.  */
#ifdef sgi
       if(in_string && is_hor_space[c] ) {
        /* ok as is: take in-string unchanged */
       } else
#endif
        if (c == '\n' ? arg->raw[i+1] == '\n' : is_space[c]) {
          while (1) {
        if (c == '\n' && arg->raw[i+1] == '\n')
          i += 2;
        else if (c != '\n' && is_space[c])
          i++;
        else break;
        c = arg->raw[i];
          }
          i--;
          c = ' ';
        }

        if (escaped)
          escaped = 0;
        else {
          if (c == '\\')
        escaped = 1;
          if (in_string && c == in_string)
        in_string = 0;
          else if (c == '\"' || c == '\'')
        in_string = c;
        }

        /* Escape these chars */
        if (c == '\"' || (in_string && c == '\\'))
          xbuf[totlen++] = '\\';
        if (isprint (c))
          xbuf[totlen++] = c;
        else {
          sprintf ((char *) &xbuf[totlen], "\\%03o", (unsigned int) c);
          totlen += 4;
        }
      }
      if (!traditional)
        xbuf[totlen++] = '\"'; /* insert ending quote */
    } else if (ap->raw_before || ap->raw_after) {
      U_CHAR *p1 = arg->raw;
      U_CHAR *l1 = p1 + arg->raw_length;
      if (ap->raw_before) {
        while (p1 != l1 && is_space[*p1]) p1++;
        while (p1 != l1 && is_idchar[*p1])
          xbuf[totlen++] = *p1++;
        /* Delete any no-reexpansion marker that follows
           an identifier at the beginning of the argument
           if the argument is concatenated with what precedes it.  */
        if (p1[0] == '\n' && p1[1] == '-')
          p1 += 2;
      }
      if (ap->raw_after) {
        /* Arg is concatenated after: delete trailing whitespace,
           whitespace markers, and no-reexpansion markers.  */
        while (p1 != l1) {
          if (is_space[l1[-1]]) l1--;
          else if (l1[-1] == '-') {
        U_CHAR *p2 = l1 - 1;
        /* If a `-' is preceded by an odd number of newlines then it
           and the last newline are a no-reexpansion marker.  */
        while (p2 != p1 && p2[-1] == '\n') p2--;
        if ((l1 - 1 - p2) & 1) {
          l1 -= 2;
        }
        else break;
          }
          else break;
        }
      }
      bcopy (p1, xbuf + totlen, l1 - p1);
      totlen += l1 - p1;
    } else {
      bcopy (arg->expanded, xbuf + totlen, arg->expand_length);
      totlen += arg->expand_length;
    }

    if (totlen > xbuf_len)
      abort ();
      }

      /* if there is anything left of the definition
     after handling the arg list, copy that in too. */

      for (i = offset; i < defn->length; i++)
    xbuf[totlen++] = exp[i];

      xbuf[totlen] = 0;
      xbuf_len = totlen;

      for (i = 0; i < nargs; i++) {
    if (args[i].free1 != 0)
      free (args[i].free1);
    if (args[i].free2 != 0)
      free (args[i].free2);
      }
    }
  } else {
    xbuf = defn->expansion;
    xbuf_len = defn->length;
  }

  /* Now put the expansion on the input stack
     so our caller will commence reading from it.  */
  {
    register FILE_BUF *ip2;

    ip2 = &instack[++indepth];

    ip2->fname = 0;
    ip2->lineno = 0;
    ip2->buf = xbuf;
    ip2->length = xbuf_len;
    ip2->bufp = xbuf;
    ip2->free_ptr = (nargs > 0) ? xbuf : 0;
    ip2->macro = hp;
    ip2->if_stack = if_stack;

    /* Recursive macro use sometimes works traditionally.
       #define foo(x,y) bar(x(y,0), y)
       foo(foo, baz)  */

    if (!traditional)
      hp->type = T_DISABLED;
  }
}

/*
 * Parse a macro argument and store the info on it into *ARGPTR.
 * Return nonzero to indicate a syntax error.
 */

char *
macarg (argptr)
     register struct argdata *argptr;
{
  FILE_BUF *ip = &instack[indepth];
  int paren = 0;
  int newlines = 0;
  int comments = 0;

  /* Try to parse as much of the argument as exists at this
     input stack level.  */
  U_CHAR *bp = macarg1 (ip->bufp, ip->buf + ip->length,
            &paren, &newlines, &comments);

  /* If we find the end of the argument at this level,
     set up *ARGPTR to point at it in the input stack.  */
  if (!(ip->fname != 0 && (newlines != 0 || comments != 0))
      && bp != ip->buf + ip->length) {
    if (argptr != 0) {
      argptr->raw = ip->bufp;
      argptr->raw_length = bp - ip->bufp;
    }
    ip->bufp = bp;
  } else {
    /* This input stack level ends before the macro argument does.
       We must pop levels and keep parsing.
       Therefore, we must allocate a temporary buffer and copy
       the macro argument into it.  */
    int bufsize = bp - ip->bufp;
    int extra = newlines;
    U_CHAR *buffer = (U_CHAR *) xmalloc (bufsize + extra + 1);
    int final_start = 0;

    bcopy (ip->bufp, buffer, bufsize);
    ip->bufp = bp;
    ip->lineno += newlines;

    while (bp == ip->buf + ip->length) {
      if (instack[indepth].macro == 0) {
    free (buffer);
    return "unterminated macro call";
      }
      ip->macro->type = T_MACRO;
      free (ip->buf);
      ip = &instack[--indepth];
      newlines = 0;
      comments = 0;
      bp = macarg1 (ip->bufp, ip->buf + ip->length, &paren,
            &newlines, &comments);
      final_start = bufsize;
      bufsize += bp - ip->bufp;
      extra += newlines;
      buffer = (U_CHAR *) xrealloc (buffer, bufsize + extra + 1);
      bcopy (ip->bufp, buffer + bufsize - (bp - ip->bufp), bp - ip->bufp);
      ip->bufp = bp;
      ip->lineno += newlines;
    }

    /* Now, if arg is actually wanted, record its raw form,
       discarding comments and duplicating newlines in whatever
       part of it did not come from a macro expansion.
       EXTRA space has been preallocated for duplicating the newlines.
       FINAL_START is the index of the start of that part.  */
    if (argptr != 0) {
      argptr->raw = buffer;
      argptr->raw_length = bufsize;
      argptr->free1 = buffer;
      argptr->newlines = newlines;
      argptr->comments = comments;
      if ((newlines || comments) && ip->fname != 0)
    argptr->raw_length
      = final_start +
        discard_comments (argptr->raw + final_start,
                  argptr->raw_length - final_start,
                  newlines);
      argptr->raw[argptr->raw_length] = 0;
      if (argptr->raw_length > bufsize + extra)
    abort ();
    }
  }

  /* If we are not discarding this argument,
     macroexpand it and compute its length as stringified.
     All this info goes into *ARGPTR.  */

  if (argptr != 0) {
    FILE_BUF obuf;
    register U_CHAR *buf, *lim;
    register int totlen;
#ifdef sgi
    int escaped = 0;
    int in_string = 0;
#endif

    obuf = expand_to_temp_buffer (argptr->raw,
                  argptr->raw + argptr->raw_length,
                  1);

    argptr->expanded = obuf.buf;
    argptr->expand_length = obuf.length;
    argptr->free2 = obuf.buf;

    buf = argptr->raw;
    lim = buf + argptr->raw_length;

    while (buf != lim && is_space[*buf])
      buf++;
    while (buf != lim && is_space[lim[-1]])
      lim--;
    totlen = traditional ? 0 : 2;   /* Count opening and closing quote.  */
#ifdef sgi
    /* need the same logic as in real stringify code: must not
    reduce stuff inside "" or '' */
    while (buf != lim) {
      register U_CHAR c = *buf++;
      totlen++;
      /* Internal sequences of whitespace are replaced by one space.  */
      if (escaped)
      escaped = 0;
      else {
    if (c == '\\')
        escaped = 1;
    if (in_string && c == in_string)
        in_string = 0;
    else if (c == '\"' || c == '\'')
            in_string = c;
      }
      if (is_space[c] && !in_string) {
    SKIP_ALL_WHITE_SPACE (buf);
    }
      else if (c == '\"' || c == '\\') /* escape these chars */
    totlen++;
      else if (!isprint (c))
    totlen += 3;
    }
#else
    while (buf != lim) {
      register U_CHAR c = *buf++;
      totlen++;
      /* Internal sequences of whitespace are replaced by one space.  */
      if (is_space[c]) {
    SKIP_ALL_WHITE_SPACE (buf);
    }
      else if (c == '\"' || c == '\\') /* escape these chars */
    totlen++;
      else if (!isprint (c))
    totlen += 3;
    }
#endif
    argptr->stringified_length = totlen;
  }
  return 0;
}

/* Scan text from START (inclusive) up to LIMIT (exclusive),
   counting parens in *DEPTHPTR,
   and return if reach LIMIT
   or before a `)' that would make *DEPTHPTR negative
   or before a comma when *DEPTHPTR is zero.
   Single and double quotes are matched and termination
   is inhibited within them.  Comments also inhibit it.
   Value returned is pointer to stopping place.

   Increment *NEWLINES each time a newline is passed.
   Set *COMMENTS to 1 if a comment is seen.  */

U_CHAR *
macarg1 (start, limit, depthptr, newlines, comments)
     U_CHAR *start;
     register U_CHAR *limit;
     int *depthptr, *newlines, *comments;
{
  register U_CHAR *bp = start;

  while (bp < limit) {
    switch (*bp) {
    case '(':
      (*depthptr)++;
      break;
    case ')':
      if (--(*depthptr) < 0)
    return bp;
      break;
    case '\\':
      /* Backslash makes following char not special.  */
      if (bp + 1 < limit) bp++;
      break;
    case '\n':
      ++*newlines;
      break;
    case '/':
      if (bp[1] == '\\' && bp[2] == '\n')
    newline_fix (bp + 1);
      if (cplusplus && bp[1] == '/') {
    *comments = 1;
    bp += 2;
    while (bp < limit && *bp++ != '\n') ;
    ++*newlines;
    break;
      }
      if (bp[1] != '*' || bp + 1 >= limit)
    break;
      *comments = 1;
      bp += 2;
      while (bp + 1 < limit) {
    if (bp[0] == '*'
        && bp[1] == '\\' && bp[2] == '\n')
      newline_fix (bp + 1);
    if (bp[0] == '*' && bp[1] == '/')
      break;
    if (*bp == '\n') ++*newlines;
    bp++;
      }
      break;
    case '\'':
    case '\"':
      {
    int quotec;
    for (quotec = *bp++; bp + 1 < limit && *bp != quotec; bp++) {
      if (*bp == '\\') {
        bp++;
        if (*bp == '\n')
          ++*newlines;
        while (*bp == '\\' && bp[1] == '\n') {
          bp += 2;
        }
      } else if (*bp == '\n') {
        ++*newlines;
        if (quotec == '\'')
          break;
      }
    }
      }
      break;
    case ',':
      if ((*depthptr) == 0)
    return bp;
      break;
    }
    bp++;
  }

  return bp;
}

/* Discard comments and duplicate newlines
   in the string of length LENGTH at START,
   except inside of string constants.
   The string is copied into itself with its beginning staying fixed.  

   NEWLINES is the number of newlines that must be duplicated.
   We assume that that much extra space is available past the end
   of the string.  */

int
discard_comments (start, length, newlines)
     U_CHAR *start;
     int length;
     int newlines;
{
  register U_CHAR *ibp;
  register U_CHAR *obp;
  register U_CHAR *limit;
  register int c;

  /* If we have newlines to duplicate, copy everything
     that many characters up.  Then, in the second part,
     we will have room to insert the newlines
     while copying down.
     NEWLINES may actually be too large, because it counts
     newlines in string constants, and we don't duplicate those.
     But that does no harm.  */
  if (newlines > 0) {
    ibp = start + length;
    obp = ibp + newlines;
    limit = start;
    while (limit != ibp)
      *--obp = *--ibp;
  }

  ibp = start + newlines;
  limit = start + length + newlines;
  obp = start;

  while (ibp < limit) {
    *obp++ = c = *ibp++;
    switch (c) {
    case '\n':
      /* Duplicate the newline.  */
      *obp++ = '\n';
      break;

    case '/':
      if (*ibp == '\\' && ibp[1] == '\n')
    newline_fix (ibp);
      /* Delete any comment.  */
      if (cplusplus && ibp[0] == '/') {
    obp--;
    ibp++;
    while (ibp < limit && *ibp++ != '\n') ;
    break;
      }
      if (ibp[0] != '*' || ibp + 1 >= limit)
    break;
      obp--;
      ibp++;
      while (ibp + 1 < limit) {
    if (ibp[0] == '*'
        && ibp[1] == '\\' && ibp[2] == '\n')
      newline_fix (ibp + 1);
    if (ibp[0] == '*' && ibp[1] == '/')
      break;
    ibp++;
      }
      ibp += 2;
      break;

    case '\'':
    case '\"':
      /* Notice and skip strings, so that we don't
     think that comments start inside them,
     and so we don't duplicate newlines in them.  */
      {
    int quotec = c;
    while (ibp < limit) {
      *obp++ = c = *ibp++;
      if (c == quotec)
        break;
      if (c == '\n' && quotec == '\'')
        break;
      if (c == '\\' && ibp < limit) {
        while (*ibp == '\\' && ibp[1] == '\n')
          ibp += 2;
        *obp++ = *ibp++;
      }
    }
      }
      break;
    }
  }

  return obp - start;
}


/* Error including a message from `errno'.  */

error_from_errno (name)
     char *name;
{
  int i;
  FILE_BUF *ip = NULL;
#if !defined WIN32 && !defined __linux__ && !defined __APPLE__
  extern int errno, sys_nerr;
  extern char *sys_errlist[];
#endif

  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != NULL) {
      ip = &instack[i];
      break;
    }

  if (ip != NULL)
    USER_MESSAGE("%s:%d: ", _get_filename_without_path(ip->fname), ip->lineno);

#if defined WIN32 || defined __linux__ || defined __APPLE__
  perror(name);
#else
  if (errno < sys_nerr)
    USER_MESSAGE("%s: %s\n", name, sys_errlist[errno]);
  else
    USER_MESSAGE("%s: undocumented I/O error\n", name);
#endif

  errors++;
  return 0;
}

/* Print error message but don't count it.  */

warning (msg, arg1, arg2, arg3)
     char *msg;
{
  int i;
  FILE_BUF *ip = NULL;

#if 0
  fprintf(fp_out, "warning\n");
#endif

  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != NULL) {
      ip = &instack[i];
      break;
    }

  if (ip != NULL)
    USER_MESSAGE("%s:%d: warning: ", _get_filename_without_path(ip->fname), ip->lineno);
  
  USER_MESSAGE(msg, arg1, arg2, arg3);
  USER_MESSAGE("\n");
  return 0;
}

error_with_line (line, msg, arg1, arg2, arg3)
     int line;
     char *msg;
{
  int i;
  FILE_BUF *ip = NULL;

  fprintf(fp_out, "error with line\n");

  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != NULL) {
      ip = &instack[i];
      break;
    }

  if (ip != NULL)
    USER_MESSAGE("%s:%d: ", _get_filename_without_path(ip->fname), line);
  
  USER_MESSAGE(msg, arg1, arg2, arg3);
  USER_MESSAGE("\n");
  errors++;
  return 0;
}

/* Return the line at which an error occurred.
   The error is not necessarily associated with the current spot
   in the input stack, so LINE says where.  LINE will have been
   copied from ip->lineno for the current input level.
   If the current level is for a file, we return LINE.
   But if the current level is not for a file, LINE is meaningless.
   In that case, we return the lineno of the innermost file.  */
int
line_for_error (line)
     int line;
{
  int i;
  int line1 = line;

  fprintf(fp_out, "line for error\n");

  for (i = indepth; i >= 0; ) {
    if (instack[i].fname != 0)
      return line1;
    i--;
    if (i < 0)
      return 0;
    line1 = instack[i].lineno;
  }
}

/*
 * If OBUF doesn't have NEEDED bytes after OPTR, make it bigger.
 *
 * As things stand, nothing is ever placed in the output buffer to be
 * removed again except when it's KNOWN to be part of an identifier,
 * so flushing and moving down everything left, instead of expanding,
 * should work ok.
 */

int
grow_outbuf (obuf, needed)
     register FILE_BUF *obuf;
     register int needed;
{
  register U_CHAR *p;
  int minsize;

  if (obuf->length - (obuf->bufp - obuf->buf) > needed)
    return 1;

  /* Make it at least twice as big as it is now.  */
  obuf->length *= 2;
  /* Make it have at least 150% of the free space we will need.  */
  minsize = (3 * needed) / 2 + (obuf->bufp - obuf->buf);
  if (minsize > obuf->length)
    obuf->length = minsize;

  if ((p = (U_CHAR *) xrealloc (obuf->buf, obuf->length)) == NULL)
    memory_full ();

  obuf->bufp = p + (obuf->bufp - obuf->buf);
  obuf->buf = p;
}

/* Symbol table for macro names and special symbols */

/*
 * install a name in the main hash table, even if it is already there.
 *   name stops with first non alphanumeric, except leading '#'.
 * caller must check against redefinition if that is desired.
 * delete_macro () removes things installed by install () in fifo order.
 * this is important because of the `defined' special symbol used
 * in #if, and also if pushdef/popdef directives are ever implemented.
 *
 * If LEN is >= 0, it is the length of the name.
 * Otherwise, compute the length by scanning the entire name.
 *
 * If HASH is >= 0, it is the precomputed hash code.
 * Otherwise, compute the hash code.
 */
HASHNODE *
install (name, len, type, value, hash)
     U_CHAR *name;
     int len;
     enum node_type type;
     int value;
     int hash;
        /* watch out here if sizeof (U_CHAR *) != sizeof (int) */
{
  register HASHNODE *hp;
  register int i, bucket;
  register U_CHAR *p, *q;

  if (len < 0) {
    p = name;
    while (is_idchar[*p])
      p++;
    len = p - name;
  }

  if (hash < 0)
    hash = hashf (name, len, HASHSIZE);

  i = sizeof (HASHNODE) + len + 1;
  hp = (HASHNODE *) xmalloc (i);
  bucket = hash;
  hp->bucket_hdr = &hashtab[bucket];
  hp->next = hashtab[bucket];
  hashtab[bucket] = hp;
  hp->prev = NULL;
  if (hp->next != NULL)
    hp->next->prev = hp;
  hp->type = type;
  hp->length = len;
  hp->value.ival = value;
  hp->name = ((U_CHAR *) hp) + sizeof (HASHNODE);
  p = hp->name;
  q = name;
  for (i = 0; i < len; i++)
    *p++ = *q++;
  hp->name[len] = 0;
  return hp;
}

/*
 * find the most recent hash node for name name (ending with first
 * non-identifier char) installed by install
 *
 * If LEN is >= 0, it is the length of the name.
 * Otherwise, compute the length by scanning the entire name.
 *
 * If HASH is >= 0, it is the precomputed hash code.
 * Otherwise, compute the hash code.
 */
HASHNODE *
lookup (name, len, hash)
     U_CHAR *name;
     int len;
     int hash;
{
  register U_CHAR *bp;
  register HASHNODE *bucket;

  if (len < 0) {
    for (bp = name; is_idchar[*bp]; bp++) ;
    len = bp - name;
  }

  if (hash < 0)
    hash = hashf (name, len, HASHSIZE);

  bucket = hashtab[hash];
  while (bucket) {
    if (bucket->length == len && strncmp (bucket->name, name, len) == 0)
      return bucket;
    bucket = bucket->next;
  }
  return NULL;
}

/*
 * Delete a hash node.  Some weirdness to free junk from macros.
 * More such weirdness will have to be added if you define more hash
 * types that need it.
 */

/* Note that the DEFINITION of a macro is removed from the hash table
   but its storage is not freed.  This would be a storage leak
   except that it is not reasonable to keep undefining and redefining
   large numbers of macros many times.
   In any case, this is necessary, because a macro can be #undef'd
   in the middle of reading the arguments to a call to it.
   If #undef freed the DEFINITION, that would crash.  */

delete_macro (hp)
     HASHNODE *hp;
{

  if (hp->prev != NULL)
    hp->prev->next = hp->next;
  if (hp->next != NULL)
    hp->next->prev = hp->prev;

  /* make sure that the bucket chain header that
     the deleted guy was on points to the right thing afterwards. */
  if (hp == *hp->bucket_hdr)
    *hp->bucket_hdr = hp->next;

#if 0
  if (hp->type == T_MACRO) {
    DEFINITION *d = hp->value.defn;
    struct reflist *ap, *nextap;

    for (ap = d->pattern; ap != NULL; ap = nextap) {
      nextap = ap->next;
      free (ap);
    }
    free (d);
  }
#endif
  free (hp);
}

/*
 * return hash function on name.  must be compatible with the one
 * computed a step at a time, elsewhere
 */
int
hashf (name, len, hashsize)
     register U_CHAR *name;
     register int len;
     int hashsize;
{
  register int r = 0;

  while (len--)
    r = HASHSTEP (r, *name++);

  return MAKE_POS (r) % hashsize;
}

/* Dump all macro definitions as #defines to stdout.  */

void
dump_all_macros ()
{
  int bucket;

  for (bucket = 0; bucket < HASHSIZE; bucket++) {
    register HASHNODE *hp;

    for (hp = hashtab[bucket]; hp; hp= hp->next) {
      if (hp->type == T_MACRO) {
    register DEFINITION *defn = hp->value.defn;
    struct reflist *ap;
    int offset;
    int concat;


    /* Print the definition of the macro HP.  */

    fprintf(fp_out, "#define %s", hp->name);
    if (defn->nargs >= 0) {
      int i;

      fprintf(fp_out, "(");
      for (i = 0; i < defn->nargs; i++) {
        dump_arg_n (defn, i);
        if (i + 1 < defn->nargs)
          fprintf(fp_out, ", ");
      }
      fprintf(fp_out, ")");
    }

    fprintf(fp_out, " ");

    offset = 0;
    concat = 0;
    for (ap = defn->pattern; ap != NULL; ap = ap->next) {
      dump_defn_1 (defn->expansion, offset, ap->nchars);
      if (ap->nchars != 0)
        concat = 0;
      offset += ap->nchars;
      if (ap->stringify)
        fprintf(fp_out, " #");
      if (ap->raw_before && !concat)
        fprintf(fp_out, " ## ");
      concat = 0;
      dump_arg_n (defn, ap->argno);
      if (ap->raw_after) {
        fprintf(fp_out, " ## ");
        concat = 1;
      }
    }
    dump_defn_1 (defn->expansion, offset, defn->length - offset);
    fprintf(fp_out, "\n");
      }
    }
  }
}

/* Output to stdout a substring of a macro definition.
   BASE is the beginning of the definition.
   Output characters START thru LENGTH.
   Discard newlines outside of strings, thus
   converting funny-space markers to ordinary spaces.  */

dump_defn_1 (base, start, length)
     U_CHAR *base;
     int start;
     int length;
{
  U_CHAR *p = base + start;
  U_CHAR *limit = base + start + length;

  while (p < limit) {
    if (*p != '\n')
      putc (*p, fp_out ? fp_out : stdout);
    else if (*p == '\"' || *p =='\'') {
      U_CHAR *p1 = skip_quoted_string (p, limit, 0, 0, 0, 0);
      fwrite (p, p1 - p, 1, stdout);
      p = p1 - 1;
    }
    p++;
  }
}

/* Print the name of argument number ARGNUM of macro definition DEFN.
   Recall that DEFN->argnames contains all the arg names
   concatenated in reverse order with comma-space in between.  */

dump_arg_n (defn, argnum)
     DEFINITION *defn;
     int argnum;
{
  register U_CHAR *p = defn->argnames;
  while (argnum + 1 < defn->nargs) {
    p = (U_CHAR *) index (p, ' ') + 1;
    argnum++;
  }

  while (*p && *p != ',') {
    putc (*p, fp_out ? fp_out : stdout);
    p++;
  }
}

/* Initialize syntactic classifications of characters.  */

initialize_char_syntax ()
{
  register int i;

  /*
   * Set up is_idchar and is_idstart tables.  These should be
   * faster than saying (is_alpha (c) || c == '_'), etc.
   * Must do set up these things before calling any routines tthat
   * refer to them.
   */
  for (i = 'a'; i <= 'z'; i++) {
    is_idchar[i - 'a' + 'A'] = 1;
    is_idchar[i] = 1;
    is_idstart[i - 'a' + 'A'] = 1;
    is_idstart[i] = 1;
  }
  for (i = '0'; i <= '9'; i++)
    is_idchar[i] = 1;
  is_idchar['_'] = 1;
  is_idstart['_'] = 1;
  is_idchar['$'] = dollars_in_ident;
  is_idstart['$'] = dollars_in_ident;

  /* horizontal space table */
  is_hor_space[' '] = 1;
  is_hor_space['\t'] = 1;
  is_hor_space['\v'] = 1;
  is_hor_space['\f'] = 1;

  is_space[' '] = 1;
  is_space['\t'] = 1;
  is_space['\v'] = 1;
  is_space['\f'] = 1;
  is_space['\n'] = 1;
}

/* Initialize the built-in macros.  */

initialize_builtins ()
{
  install ("__LINE__", -1, T_SPECLINE, 0, -1);
  install ("__DATE__", -1, T_DATE, 0, -1);
  install ("__FILE__", -1, T_FILE, 0, -1);
  install ("__BASE_FILE__", -1, T_BASE_FILE, 0, -1);
  install ("__INCLUDE_LEVEL__", -1, T_INCLUDE_LEVEL, 0, -1);
  install ("__VERSION__", -1, T_VERSION, 0, -1);
  install ("__TIME__", -1, T_TIME, 0, -1);
  install ("__ANSI_CPP__", -1, T_CONST, 1, -1); 
/*if (!traditional)
    install ("__STDC__", -1, T_CONST, 1, -1); */
/*  install ("__GNU__", -1, T_CONST, 1, -1);  */
/*  This is supplied using a -D by the compiler driver
    so that it is present only when truly compiling with GNU C.  */
}

/*
 * process a given definition string, for initialization
 * If STR is just an identifier, define it with value 1.
 * If STR has anything after the identifier, then it should
 * be identifier-space-definition.
 */
make_definition (str)
     U_CHAR *str;
{
  FILE_BUF *ip;
  struct directive *kt;
  U_CHAR *buf, *p;

  buf = str;
  p = str;
  while (is_idchar[*p]) p++;
  if (*p == 0) {
    buf = (U_CHAR *) alloca (p - buf + 4);
    strcpy (buf, str);
    strcat (buf, " 1");
  }
  
  ip = &instack[++indepth];
  ip->fname = "*Initialization*";

  ip->buf = ip->bufp = buf;
  ip->length = strlen (buf);
  ip->lineno = 1;
  ip->macro = 0;
  ip->free_ptr = 0;
  ip->if_stack = if_stack;

  for (kt = directive_table; kt->type != T_DEFINE; kt++)
    ;

  /* pass NULL as output ptr to do_define since we KNOW it never
     does any output.... */
  do_define (buf, buf + strlen (buf) , NULL, kt);
  --indepth;
}

/* JF, this does the work for the -U option */
make_undef (str)
     U_CHAR *str;
{
  FILE_BUF *ip;
  struct directive *kt;

  ip = &instack[++indepth];
  ip->fname = "*undef*";

  ip->buf = ip->bufp = str;
  ip->length = strlen (str);
  ip->lineno = 1;
  ip->macro = 0;
  ip->free_ptr = 0;
  ip->if_stack = if_stack;

  for (kt = directive_table; kt->type != T_UNDEF; kt++)
    ;

  do_undef (str,str + strlen (str) - 1, NULL, kt);
  --indepth;
}

/* Add output to `deps_buffer' for the -M switch.
   STRING points to the text to be output.
   SIZE is the number of bytes, or 0 meaning output until a null.
   If SIZE is nonzero, we break the line first, if it is long enough.  */

deps_output (string, size)
     char *string;
     int size;
{
#ifndef MAX_OUTPUT_COLUMNS
#define MAX_OUTPUT_COLUMNS 75
#endif
#ifndef sgi
  if (size != 0 && deps_column != 0
      && size + deps_column > MAX_OUTPUT_COLUMNS) {
    deps_output ("\\\n  ", 0);
    deps_column = 0;
  }
#endif
  if (size == 0)
    size = strlen (string);

  if (deps_size + size + 1 > deps_allocated_size) {
    deps_allocated_size = deps_size + size + 50;
    deps_allocated_size *= 2;
    deps_buffer = (char *) xrealloc (deps_buffer, deps_allocated_size);
  }
  bcopy (string, &deps_buffer[deps_size], size);
  deps_size += size;
  deps_column += size;
  deps_buffer[deps_size] = 0;
}

#if !defined BSD && !defined __APPLE__
#ifndef BSTRING

void
bzero (b, length)
     register char *b;
     register int length;
{
#ifdef VMS
  short zero = 0;
  long max_str = 65535;

  while (length > max_str) {
    (void) LIB$MOVC5 (&zero, &zero, &zero, &max_str, b);
    length -= max_str;
    b += max_str;
  }
  (void) LIB$MOVC5 (&zero, &zero, &zero, &length, b);
#else
  while (length-- > 0)
    *b++ = 0;
#endif /* not VMS */
}

void
bcopy (b1, b2, length)
     register char *b1;
     register char *b2;
     register int length;
{
#ifdef VMS
  long max_str = 65535;

  while (length > max_str) {
    (void) LIB$MOVC3 (&max_str, b1, b2);
    length -= max_str;
    b1 += max_str;
    b2 += max_str;
  }
  (void) LIB$MOVC3 (&length, b1, b2);
#else
  while (length-- > 0)
    *b2++ = *b1++;
#endif /* not VMS */
}

int
bcmp (b1, b2, length)   /* This could be a macro! */
     register char *b1;
     register char *b2;
      register int length;
{
#ifdef VMS
   struct dsc$descriptor_s src1 = {length, DSC$K_DTYPE_T, DSC$K_CLASS_S, b1};
   struct dsc$descriptor_s src2 = {length, DSC$K_DTYPE_T, DSC$K_CLASS_S, b2};

   return STR$COMPARE (&src1, &src2);
#else
   while (length-- > 0)
     if (*b1++ != *b2++)
       return 1;

   return 0;
#endif /* not VMS */
}
#endif /* not BSTRING */
#endif /* not BSD */


void
fatal (str, arg)
     char *str, *arg;
{
   USER_MESSAGE("%s: ", acpp_progname);
   USER_MESSAGE(str, arg);
   USER_MESSAGE("\n");
/*  return (FATAL_EXIT_CODE);*/
}

void
perror_with_name (name)
     char *name;
{
#if !defined WIN32 && !defined __linux__ && !defined __APPLE__ 
   extern int errno, sys_nerr;
   extern char *sys_errlist[];
#endif

   USER_MESSAGE("%s: ", acpp_progname);

#if defined WIN32 || defined __linux__ || defined __APPLE__
  perror(name);
#else
   if (errno < sys_nerr)
      USER_MESSAGE("%s: %s\n", name, sys_errlist[errno]);
   else
      USER_MESSAGE("%s: undocumented I/O error\n", name);
#endif
   errors++;

}

void
pfatal_with_name (name)
     char *name;
{
   perror_with_name (name);
/*  return (FATAL_EXIT_CODE);*/
}


void
memory_full ()
{
   fatal ("Memory exhausted.");
}


char *
xmalloc (size)
     int size;
{
  /*extern char *malloc ();*/
  register char *ptr = malloc (size);
  if (ptr != 0) return (ptr);
  memory_full ();
  /*NOTREACHED*/
}

char *
xrealloc (old, size)
     char *old;
     int size;
{
  /*extern char *realloc ();*/
  register char *ptr = realloc (old, size);
  if (ptr != 0) return (ptr);
  memory_full ();
  /*NOTREACHED*/
}

char *
xcalloc (number, size)
     int number, size;
{
  /*extern char *malloc ();*/
  register int total = number * size;
  register char *ptr = malloc (total);
  if (ptr != 0) {
    if (total > 100)
      bzero (ptr, total);
    else {
      /* It's not too long, so loop, zeroing by longs.
     It must be safe because malloc values are always well aligned.  */
      register long *zp = (long *) ptr;
      register long *zl = (long *) (ptr + total - 4);
      register int i = total - 4;
      while (zp < zl)
    *zp++ = 0;
      if (i < 0)
    i = 0;
      while (i < total)
    ptr[i++] = 0;
    }
    return ptr;
  }
  memory_full ();
  /*NOTREACHED*/
}

char *
savestring (input)
     char *input;
{
  int size = strlen (input);
  char *output = xmalloc (size + 1);
  strcpy (output, input);
  return output;
}

/* Get the file-mode and data size of the file open on FD
   and store them in *MODE_POINTER and *SIZE_POINTER.  */

int
file_size_and_mode (fd, mode_pointer, size_pointer)
     int fd;
     int *mode_pointer;
     long int *size_pointer;
{
  struct stat sbuf;

  if (fstat (fd, &sbuf) < 0) return (-1);
  if (mode_pointer) *mode_pointer = sbuf.st_mode;
  if (size_pointer) *size_pointer = sbuf.st_size;
  return 0;
}

#ifdef  VMS

/* Under VMS we need to fix up the "include" specification
   filename so that everything following the 1st slash is
   changed into its correct VMS file specification. */

hack_vms_include_specification (fname)
     char *fname;
{
  register char *cp, *cp1, *cp2;
  char Local[512];
  extern char *index (), *rindex ();

  /* Ignore leading "./"s */
  while (fname[0] == '.' && fname[1] == '/')
    strcpy (fname, fname+2);
  /* Look for the boundary between the VMS and UNIX filespecs */
  cp = rindex (fname, ']'); /* Look for end of dirspec. */
  if (cp == 0) cp == rindex (fname, '>'); /* ... Ditto          */
  if (cp == 0) cp == rindex (fname, ':'); /* Look for end of devspec. */
  if (cp) {
    cp++;
  } else {
    cp = index (fname, '/');    /* Look for the "/" */
  }
  /* See if we found that 1st slash */
  if (cp == 0) return;      /* Nothing to do!!! */
  if (*cp != '/') return;   /* Nothing to do!!! */
  /* Point to the UNIX filename part (which needs to be fixed!) */
  cp1 = cp+1;
  /* If the directory spec is not rooted, we can just copy
     the UNIX filename part and we are done */
  if (((cp - fname) > 2)
      && ((cp[-1] == ']') || (cp[-1] == '>'))
      && (cp[-2] != '.')) {
    strcpy (cp, cp1);
    return;
  }
  /* If there are no other slashes then the filename will be
     in the "root" directory.  Otherwise, we need to add
     directory specifications. */
  if (index (cp1, '/') == 0) {
    /* Just add "[000000]" as the directory string */
    strcpy (Local, "[000000]");
    cp2 = Local + strlen (Local);
  } else {
    /* Open the directory specification */
    cp2 = Local;
    *cp2++ = '[';
    /* As long as there are still subdirectories to add, do them. */
    while (index (cp1, '/') != 0) {
      /* If this token is "." we can ignore it */
      if ((cp1[0] == '.') && (cp1[1] == '/')) {
    cp1 += 2;
    continue;
      }
      /* Add a subdirectory spec. */
      if (cp2 != Local+1) *cp2++ = '.';
      /* If this is ".." then the spec becomes "-" */
      if ((cp1[0] == '.') && (cp1[1] == '.') && (cp[2] == '/')) {
    /* Add "-" and skip the ".." */
    *cp2++ = '-';
    cp1 += 3;
    continue;
      }
      /* Copy the subdirectory */
      while (*cp1 != '/') *cp2++= *cp1++;
      cp1++;            /* Skip the "/" */
    }
    /* Close the directory specification */
    *cp2++ = ']';
  }
  /* Now add the filename */
  while (*cp1) *cp2++ = *cp1++;
  *cp2 = 0;
  /* Now append it to the original VMS spec. */
  strcpy (cp, Local);
  return;
}
#endif  /* VMS */

#ifdef  VMS

/* These are the read/write replacement routines for
   VAX-11 "C".  They make read/write behave enough
   like their UNIX counterparts that CCCP will work */

int
read (fd, buf, size)
     int fd;
     char *buf;
     int size;
{
#undef  read    /* Get back the REAL read routine */
  register int i;
  register int total = 0;

  /* Read until the buffer is exhausted */
  while (size > 0) {
    /* Limit each read to 32KB */
    i = (size > (32*1024)) ? (32*1024) : size;
    i = read (fd, buf, i);
    if (i <= 0) {
      if (i == 0) return (total);
      return(i);
    }
    /* Account for this read */
    total += i;
    buf += i;
    size -= i;
  }
  return (total);
}

int
write (fd, buf, size)
     int fd;
     char *buf;
     int size;
{
#undef  write   /* Get back the REAL write routine */
  int i;
  int j;

  /* Limit individual writes to 32Kb */
  i = size;
  while (i > 0) {
    j = (i > (32*1024)) ? (32*1024) : i;
    if (write (fd, buf, j) < 0) return (-1);
    /* Account for the data written */
    buf += j;
    i -= j;
  }
  return (size);
}

#endif /* VMS */
