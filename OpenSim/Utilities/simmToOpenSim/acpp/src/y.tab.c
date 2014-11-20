/* A Bison parser, made by GNU Bison 1.875b.  */

/* Skeleton parser for Yacc-like parsing with Bison,
   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003 Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */

/* As a special exception, when this file is copied by Bison into a
   Bison output file, you may use that output file without restriction.
   This special exception was added by the Free Software Foundation
   in version 1.24 of Bison.  */

/* Written by Richard Stallman by simplifying the original so called
   ``semantic'' parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     INT = 258,
     CHAR = 259,
     NAME = 260,
     ERROR = 261,
     DVALUE = 262,
     OR = 263,
     AND = 264,
     NOTEQUAL = 265,
     EQUAL = 266,
     GEQ = 267,
     LEQ = 268,
     RSH = 269,
     LSH = 270,
     UNARY = 271
   };
#endif
#define INT 258
#define CHAR 259
#define NAME 260
#define ERROR 261
#define DVALUE 262
#define OR 263
#define AND 264
#define NOTEQUAL 265
#define EQUAL 266
#define GEQ 267
#define LEQ 268
#define RSH 269
#define LSH 270
#define UNARY 271




/* Copy the first part of user declarations.  */
#line 26 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"

#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "config.h"
#include <setjmp.h>
#include <ctype.h>
/* #define YYDEBUG 1 */

  int yylex ();
  void yyerror ();
  int inside_math = 0;
  int expression_value;
  double math_value;
  
  extern FILE* fp_out;

  static jmp_buf parse_return_error;

  /* some external tables of character types */
  extern unsigned char is_idstart[], is_idchar[];
#if defined sgi || defined WIN32
/* table to tell if c is horizontal space.  isspace () thinks that
   newline is space; this is not a good idea for this program. */
char is_hor_space[256];
#endif

#ifdef WIN32
#define gettxt(S1,S2) (S1 ## S2)
#endif

#ifndef CHAR_TYPE_SIZE
#define CHAR_TYPE_SIZE BITS_PER_UNIT
#endif

acpp_error (msg)
{
#ifdef INSIDE_SIMM
   error(0, msg); // 0 = SIMM error action of "recover"
#else
  fprintf (stderr, "error: %s\n", msg);
#endif
}

acpp_warning (msg)
{
#ifdef INSIDE_SIMM
   error(0, msg); // 0 = SIMM error action of "recover"
#else
  fprintf (stderr, "warning: %s\n", msg);
#endif
}

/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

#if ! defined (YYSTYPE) && ! defined (YYSTYPE_IS_DECLARED)
#line 62 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
typedef union YYSTYPE {
  struct constant {long value; int unsignedp;} integer;
  int voidval;
  char *sval;
  double dval;
} YYSTYPE;
/* Line 191 of yacc.c.  */
#line 151 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.tab.c"
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif



/* Copy the second part of user declarations.  */


/* Line 214 of yacc.c.  */
#line 163 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.tab.c"

#if ! defined (yyoverflow) || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# if YYSTACK_USE_ALLOCA
#  define YYSTACK_ALLOC alloca
# else
#  ifndef YYSTACK_USE_ALLOCA
#   if defined (alloca) || defined (_ALLOCA_H)
#    define YYSTACK_ALLOC alloca
#   else
#    ifdef __GNUC__
#     define YYSTACK_ALLOC __builtin_alloca
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning. */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
# else
#  if defined (__STDC__) || defined (__cplusplus)
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   define YYSIZE_T size_t
#  endif
#  define YYSTACK_ALLOC malloc
#  define YYSTACK_FREE free
# endif
#endif /* ! defined (yyoverflow) || YYERROR_VERBOSE */


#if (! defined (yyoverflow) \
     && (! defined (__cplusplus) \
     || (YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  short yyss;
  YYSTYPE yyvs;
  };

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (short) + sizeof (YYSTYPE))             \
      + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)      \
      do                    \
    {                   \
      register YYSIZE_T yyi;        \
      for (yyi = 0; yyi < (Count); yyi++)   \
        (To)[yyi] = (From)[yyi];        \
    }                   \
      while (0)
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack)                    \
    do                                  \
      {                                 \
    YYSIZE_T yynewbytes;                        \
    YYCOPY (&yyptr->Stack, Stack, yysize);              \
    Stack = &yyptr->Stack;                      \
    yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
    yyptr += yynewbytes / sizeof (*yyptr);              \
      }                                 \
    while (0)

#endif

#if defined (__STDC__) || defined (__cplusplus)
   typedef signed char yysigned_char;
#else
   typedef short yysigned_char;
#endif

/* YYFINAL -- State number of the termination state. */
#define YYFINAL  25
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   226

/* YYNTOKENS -- Number of terminals. */
#define YYNTOKENS  36
/* YYNNTS -- Number of nonterminals. */
#define YYNNTS  5
/* YYNRULES -- Number of rules. */
#define YYNRULES  39
/* YYNRULES -- Number of states. */
#define YYNSTATES  79

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   271

#define YYTRANSLATE(YYX)                        \
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const unsigned char yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    30,     2,     2,     2,    28,    15,     2,
      32,    33,    26,    24,    10,    25,     2,    27,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     9,     2,
      18,     2,    19,     8,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,    14,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    34,    13,    35,    31,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,    11,    12,    16,    17,    20,    21,    22,
      23,    29
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const unsigned char yyprhs[] =
{
       0,     0,     3,     5,     7,     9,    13,    16,    19,    22,
      26,    29,    33,    37,    41,    45,    49,    53,    55,    59,
      63,    67,    71,    75,    79,    83,    87,    91,    95,    99,
     103,   107,   111,   115,   119,   123,   127,   133,   135,   137
};

/* YYRHS -- A `-1'-separated list of the rules' RHS. */
static const yysigned_char yyrhs[] =
{
      37,     0,    -1,    38,    -1,    40,    -1,    39,    -1,    38,
      10,    39,    -1,    25,    39,    -1,    30,    39,    -1,    31,
      39,    -1,    32,    38,    33,    -1,    25,    40,    -1,    32,
      40,    33,    -1,    34,    40,    35,    -1,    40,    26,    40,
      -1,    40,    27,    40,    -1,    40,    24,    40,    -1,    40,
      25,    40,    -1,     7,    -1,    39,    26,    39,    -1,    39,
      27,    39,    -1,    39,    28,    39,    -1,    39,    24,    39,
      -1,    39,    25,    39,    -1,    39,    23,    39,    -1,    39,
      22,    39,    -1,    39,    17,    39,    -1,    39,    16,    39,
      -1,    39,    21,    39,    -1,    39,    20,    39,    -1,    39,
      18,    39,    -1,    39,    19,    39,    -1,    39,    15,    39,
      -1,    39,    14,    39,    -1,    39,    13,    39,    -1,    39,
      12,    39,    -1,    39,    11,    39,    -1,    39,     8,    39,
       9,    39,    -1,     3,    -1,     4,    -1,     5,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const unsigned char yyrline[] =
{
       0,    94,    94,    96,   101,   102,   107,   110,   113,   116,
     121,   123,   125,   130,   132,   139,   141,   143,   148,   154,
     165,   176,   179,   182,   188,   194,   197,   200,   206,   212,
     218,   224,   227,   230,   233,   236,   239,   242,   244,   246
};
#endif

#if YYDEBUG || YYERROR_VERBOSE
/* YYTNME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals. */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "INT", "CHAR", "NAME", "ERROR", "DVALUE", 
  "'?'", "':'", "','", "OR", "AND", "'|'", "'^'", "'&'", "NOTEQUAL", 
  "EQUAL", "'<'", "'>'", "GEQ", "LEQ", "RSH", "LSH", "'+'", "'-'", "'*'", 
  "'/'", "'%'", "UNARY", "'!'", "'~'", "'('", "')'", "'{'", "'}'", 
  "$accept", "start", "exp1", "exp", "exp3", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const unsigned short yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,    63,    58,
      44,   263,   264,   124,    94,    38,   265,   266,    60,    62,
     267,   268,   269,   270,    43,    45,    42,    47,    37,   271,
      33,   126,    40,    41,   123,   125
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const unsigned char yyr1[] =
{
       0,    36,    37,    37,    38,    38,    39,    39,    39,    39,
      40,    40,    40,    40,    40,    40,    40,    40,    39,    39,
      39,    39,    39,    39,    39,    39,    39,    39,    39,    39,
      39,    39,    39,    39,    39,    39,    39,    39,    39,    39
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const unsigned char yyr2[] =
{
       0,     2,     1,     1,     1,     3,     2,     2,     2,     3,
       2,     3,     3,     3,     3,     3,     3,     1,     3,     3,
       3,     3,     3,     3,     3,     3,     3,     3,     3,     3,
       3,     3,     3,     3,     3,     3,     5,     1,     1,     1
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const unsigned char yydefact[] =
{
       0,    37,    38,    39,    17,     0,     0,     0,     0,     0,
       0,     2,     4,     3,     6,    10,     0,     0,     7,     8,
       0,     0,     0,     0,     0,     1,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       9,    11,    12,     5,     0,    35,    34,    33,    32,    31,
      26,    25,    29,    30,    28,    27,    24,    23,    21,    22,
      18,    19,    20,    15,    16,    13,    14,     0,    36
};

/* YYDEFGOTO[NTERM-NUM]. */
static const yysigned_char yydefgoto[] =
{
      -1,    10,    20,    12,    15
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -22
static const short yypact[] =
{
      48,   -22,   -22,   -22,   -22,    48,    66,    66,    48,    49,
      15,     7,   112,    68,   -22,   -22,    66,    66,   -22,   -22,
       9,   -17,    49,    49,   -21,   -22,    66,    66,    66,    66,
      66,    66,    66,    66,    66,    66,    66,    66,    66,    66,
      66,    66,    66,    66,    66,    66,    49,    49,    49,    49,
     -22,   -22,   -22,   112,    91,   129,   145,   160,   174,   187,
     198,   198,    35,    35,    35,    35,    19,    19,    39,    39,
     -22,   -22,   -22,   -14,   -14,   -22,   -22,    66,   112
};

/* YYPGOTO[NTERM-NUM].  */
static const yysigned_char yypgoto[] =
{
     -22,   -22,    18,    -5,    41
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -1
static const unsigned char yytable[] =
{
      14,    18,    19,    46,    47,    48,    49,    46,    47,    48,
      49,    14,    48,    49,    52,    25,    51,    26,    11,    26,
       0,    53,    54,    55,    56,    57,    58,    59,    60,    61,
      62,    63,    64,    65,    66,    67,    68,    69,    70,    71,
      72,    13,    50,    41,    42,    43,    44,    45,     0,    21,
      24,     1,     2,     3,     0,     4,     4,    39,    40,    41,
      42,    43,    44,    45,    21,    43,    44,    45,     0,     1,
       2,     3,    78,     5,    22,     0,     0,     0,     6,     7,
       8,    23,     9,     9,     0,     0,     0,    73,    74,    75,
      76,    16,    46,    47,    48,    49,     6,     7,    17,    27,
      77,     0,    28,    29,    30,    31,    32,    33,    34,    35,
      36,    37,    38,    39,    40,    41,    42,    43,    44,    45,
      27,     0,     0,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    29,    30,    31,    32,    33,    34,    35,    36,    37,
      38,    39,    40,    41,    42,    43,    44,    45,    30,    31,
      32,    33,    34,    35,    36,    37,    38,    39,    40,    41,
      42,    43,    44,    45,    31,    32,    33,    34,    35,    36,
      37,    38,    39,    40,    41,    42,    43,    44,    45,    32,
      33,    34,    35,    36,    37,    38,    39,    40,    41,    42,
      43,    44,    45,    33,    34,    35,    36,    37,    38,    39,
      40,    41,    42,    43,    44,    45,    35,    36,    37,    38,
      39,    40,    41,    42,    43,    44,    45
};

static const yysigned_char yycheck[] =
{
       5,     6,     7,    24,    25,    26,    27,    24,    25,    26,
      27,    16,    26,    27,    35,     0,    33,    10,     0,    10,
      -1,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,     0,    33,    24,    25,    26,    27,    28,    -1,     8,
       9,     3,     4,     5,    -1,     7,     7,    22,    23,    24,
      25,    26,    27,    28,    23,    26,    27,    28,    -1,     3,
       4,     5,    77,    25,    25,    -1,    -1,    -1,    30,    31,
      32,    32,    34,    34,    -1,    -1,    -1,    46,    47,    48,
      49,    25,    24,    25,    26,    27,    30,    31,    32,     8,
       9,    -1,    11,    12,    13,    14,    15,    16,    17,    18,
      19,    20,    21,    22,    23,    24,    25,    26,    27,    28,
       8,    -1,    -1,    11,    12,    13,    14,    15,    16,    17,
      18,    19,    20,    21,    22,    23,    24,    25,    26,    27,
      28,    12,    13,    14,    15,    16,    17,    18,    19,    20,
      21,    22,    23,    24,    25,    26,    27,    28,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    14,    15,    16,    17,    18,    19,
      20,    21,    22,    23,    24,    25,    26,    27,    28,    15,
      16,    17,    18,    19,    20,    21,    22,    23,    24,    25,
      26,    27,    28,    16,    17,    18,    19,    20,    21,    22,
      23,    24,    25,    26,    27,    28,    18,    19,    20,    21,
      22,    23,    24,    25,    26,    27,    28
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const unsigned char yystos[] =
{
       0,     3,     4,     5,     7,    25,    30,    31,    32,    34,
      37,    38,    39,    40,    39,    40,    25,    32,    39,    39,
      38,    40,    25,    32,    40,     0,    10,     8,    11,    12,
      13,    14,    15,    16,    17,    18,    19,    20,    21,    22,
      23,    24,    25,    26,    27,    28,    24,    25,    26,    27,
      33,    33,    35,    39,    39,    39,    39,    39,    39,    39,
      39,    39,    39,    39,    39,    39,    39,    39,    39,    39,
      39,    39,    39,    40,    40,    40,    40,     9,    39
};

#if ! defined (YYSIZE_T) && defined (__SIZE_TYPE__)
# define YYSIZE_T __SIZE_TYPE__
#endif
#if ! defined (YYSIZE_T) && defined (size_t)
# define YYSIZE_T size_t
#endif
#if ! defined (YYSIZE_T)
# if defined (__STDC__) || defined (__cplusplus)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# endif
#endif
#if ! defined (YYSIZE_T)
# define YYSIZE_T unsigned int
#endif

#define yyerrok     (yyerrstatus = 0)
#define yyclearin   (yychar = YYEMPTY)
#define YYEMPTY     (-2)
#define YYEOF       0

#define YYACCEPT    goto yyacceptlab
#define YYABORT     goto yyabortlab
#define YYERROR     goto yyerrlab1


/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */

#define YYFAIL      goto yyerrlab

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)                  \
do                              \
  if (yychar == YYEMPTY && yylen == 1)              \
    {                               \
      yychar = (Token);                     \
      yylval = (Value);                     \
      yytoken = YYTRANSLATE (yychar);               \
      YYPOPSTACK;                       \
      goto yybackup;                        \
    }                               \
  else                              \
    {                               \
      yyerror ("syntax error: cannot back up");\
      YYERROR;                          \
    }                               \
while (0)

#define YYTERROR    1
#define YYERRCODE   256

/* YYLLOC_DEFAULT -- Compute the default location (before the actions
   are run).  */

#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)         \
  Current.first_line   = Rhs[1].first_line;      \
  Current.first_column = Rhs[1].first_column;    \
  Current.last_line    = Rhs[N].last_line;       \
  Current.last_column  = Rhs[N].last_column;
#endif

/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (YYLEX_PARAM)
#else
# define YYLEX yylex ()
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)            \
do {                        \
  if (yydebug)                  \
    YYFPRINTF Args;             \
} while (0)

# define YYDSYMPRINT(Args)          \
do {                        \
  if (yydebug)                  \
    yysymprint Args;                \
} while (0)

# define YYDSYMPRINTF(Title, Token, Value, Location)        \
do {                                \
  if (yydebug)                          \
    {                               \
      YYFPRINTF (stderr, "%s ", Title);             \
      yysymprint (stderr,                   \
                  Token, Value);    \
      YYFPRINTF (stderr, "\n");                 \
    }                               \
} while (0)

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (cinluded).                                                   |
`------------------------------------------------------------------*/

#if defined (__STDC__) || defined (__cplusplus)
static void
yy_stack_print (short *bottom, short *top)
#else
static void
yy_stack_print (bottom, top)
    short *bottom;
    short *top;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (/* Nothing. */; bottom <= top; ++bottom)
    YYFPRINTF (stderr, " %d", *bottom);
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)                \
do {                                \
  if (yydebug)                          \
    yy_stack_print ((Bottom), (Top));               \
} while (0)


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if defined (__STDC__) || defined (__cplusplus)
static void
yy_reduce_print (int yyrule)
#else
static void
yy_reduce_print (yyrule)
    int yyrule;
#endif
{
  int yyi;
  unsigned int yylno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %u), ",
             yyrule - 1, yylno);
  /* Print the symbols being reduced, and their result.  */
  for (yyi = yyprhs[yyrule]; 0 <= yyrhs[yyi]; yyi++)
    YYFPRINTF (stderr, "%s ", yytname [yyrhs[yyi]]);
  YYFPRINTF (stderr, "-> %s\n", yytname [yyr1[yyrule]]);
}

# define YY_REDUCE_PRINT(Rule)      \
do {                    \
  if (yydebug)              \
    yy_reduce_print (Rule);     \
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YYDSYMPRINT(Args)
# define YYDSYMPRINTF(Title, Token, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   SIZE_MAX < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#if YYMAXDEPTH == 0
# undef YYMAXDEPTH
#endif

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined (__GLIBC__) && defined (_STRING_H)
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
static YYSIZE_T
#   if defined (__STDC__) || defined (__cplusplus)
yystrlen (const char *yystr)
#   else
yystrlen (yystr)
     const char *yystr;
#   endif
{
  register const char *yys = yystr;

  while (*yys++ != '\0')
    continue;

  return yys - yystr - 1;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined (__GLIBC__) && defined (_STRING_H) && defined (_GNU_SOURCE)
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
static char *
#   if defined (__STDC__) || defined (__cplusplus)
yystpcpy (char *yydest, const char *yysrc)
#   else
yystpcpy (yydest, yysrc)
     char *yydest;
     const char *yysrc;
#   endif
{
  register char *yyd = yydest;
  register const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

#endif /* !YYERROR_VERBOSE */



#if YYDEBUG
/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if defined (__STDC__) || defined (__cplusplus)
static void
yysymprint (FILE *yyoutput, int yytype, YYSTYPE *yyvaluep)
#else
static void
yysymprint (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  /* Pacify ``unused variable'' warnings.  */
  (void) yyvaluep;

  if (yytype < YYNTOKENS)
    {
      YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
# ifdef YYPRINT
      YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# endif
    }
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  switch (yytype)
    {
      default:
        break;
    }
  YYFPRINTF (yyoutput, ")");
}

#endif /* ! YYDEBUG */
/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

#if defined (__STDC__) || defined (__cplusplus)
static void
yydestruct (int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yytype, yyvaluep)
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  /* Pacify ``unused variable'' warnings.  */
  (void) yyvaluep;

  switch (yytype)
    {

      default:
        break;
    }
}


/* Prevent warnings from -Wmissing-prototypes.  */

#ifdef YYPARSE_PARAM
# if defined (__STDC__) || defined (__cplusplus)
int yyparse (void *YYPARSE_PARAM);
# else
int yyparse ();
# endif
#else /* ! YYPARSE_PARAM */
#if defined (__STDC__) || defined (__cplusplus)
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */



/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
# if defined (__STDC__) || defined (__cplusplus)
int yyparse (void *YYPARSE_PARAM)
# else
int yyparse (YYPARSE_PARAM)
  void *YYPARSE_PARAM;
# endif
#else /* ! YYPARSE_PARAM */
#if defined (__STDC__) || defined (__cplusplus)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{
  
  register int yystate;
  register int yyn;
  int yyresult;
  /* Number of tokens to shift before error messages enabled.  */
  int yyerrstatus;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken = 0;

  /* Three stacks and their tools:
     `yyss': related to states,
     `yyvs': related to semantic values,
     `yyls': related to locations.

     Refer to the stacks thru separate pointers, to allow yyoverflow
     to reallocate them elsewhere.  */

  /* The state stack.  */
  short yyssa[YYINITDEPTH];
  short *yyss = yyssa;
  register short *yyssp;

  /* The semantic value stack.  */
  YYSTYPE yyvsa[YYINITDEPTH];
  YYSTYPE *yyvs = yyvsa;
  register YYSTYPE *yyvsp;



#define YYPOPSTACK   (yyvsp--, yyssp--)

  YYSIZE_T yystacksize = YYINITDEPTH;

  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;


  /* When reducing, the number of symbols on the RHS of the reduced
     rule.  */
  int yylen;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY;     /* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */

  yyssp = yyss;
  yyvsp = yyvs;

  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed. so pushing a state here evens the stacks.
     */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
    /* Give user a chance to reallocate the stack. Use copies of
       these so that the &'s don't force the real ones into
       memory.  */
    YYSTYPE *yyvs1 = yyvs;
    short *yyss1 = yyss;


    /* Each stack pointer address is followed by the size of the
       data in use in that stack, in bytes.  This used to be a
       conditional around just the two extra args, but that might
       be undefined if yyoverflow is a macro.  */
    yyoverflow ("parser stack overflow",
            &yyss1, yysize * sizeof (*yyssp),
            &yyvs1, yysize * sizeof (*yyvsp),

            &yystacksize);

    yyss = yyss1;
    yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyoverflowlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
    goto yyoverflowlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
    yystacksize = YYMAXDEPTH;

      {
    short *yyss1 = yyss;
    union yyalloc *yyptr =
      (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
    if (! yyptr)
      goto yyoverflowlab;
    YYSTACK_RELOCATE (yyss);
    YYSTACK_RELOCATE (yyvs);

#  undef YYSTACK_RELOCATE
    if (yyss1 != yyssa)
      YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;


      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
          (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
    YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

/* Do appropriate processing given the current state.  */
/* Read a lookahead token if we need one and don't already have one.  */
/* yyresume: */

  /* First try to decide what to do without reference to lookahead token.  */

  yyn = yypact[yystate];
  if (yyn == YYPACT_NINF)
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YYDSYMPRINTF ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yyn == 0 || yyn == YYTABLE_NINF)
    goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  /* Shift the lookahead token.  */
  YYDPRINTF ((stderr, "Shifting token %s, ", yytname[yytoken]));

  /* Discard the token being shifted unless it is eof.  */
  if (yychar != YYEOF)
    yychar = YYEMPTY;

  *++yyvsp = yylval;


  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  yystate = yyn;
  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 2:
#line 95 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { expression_value = yyvsp[0].integer.value; math_value = 0.0; ;}
    break;

  case 3:
#line 97 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { math_value = yyvsp[0].dval; expression_value = 0; ;}
    break;

  case 5:
#line 103 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer = yyvsp[0].integer; ;}
    break;

  case 6:
#line 108 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = - yyvsp[0].integer.value;
              yyval.integer.unsignedp = yyvsp[0].integer.unsignedp; ;}
    break;

  case 7:
#line 111 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = ! yyvsp[0].integer.value;
              yyval.integer.unsignedp = 0; ;}
    break;

  case 8:
#line 114 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = ~ yyvsp[0].integer.value;
              yyval.integer.unsignedp = yyvsp[0].integer.unsignedp; ;}
    break;

  case 9:
#line 117 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer = yyvsp[-1].integer; ;}
    break;

  case 10:
#line 122 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.dval = - yyvsp[0].dval; ;}
    break;

  case 11:
#line 124 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.dval = yyvsp[-1].dval; ;}
    break;

  case 12:
#line 126 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.dval = yyvsp[-1].dval; ;}
    break;

  case 13:
#line 131 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.dval = yyvsp[-2].dval * yyvsp[0].dval; ;}
    break;

  case 14:
#line 133 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { if (yyvsp[0].dval == 0.0)
                    {
                    acpp_error("division by zero in expression");
                    yyvsp[0].dval = 1.0;
                    }
                    yyval.dval = yyvsp[-2].dval / yyvsp[0].dval; ;}
    break;

  case 15:
#line 140 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.dval = yyvsp[-2].dval + yyvsp[0].dval; ;}
    break;

  case 16:
#line 142 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.dval = yyvsp[-2].dval - yyvsp[0].dval; ;}
    break;

  case 17:
#line 144 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.dval = yylval.dval; ;}
    break;

  case 18:
#line 149 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp;
              if (yyval.integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value * yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value * yyvsp[0].integer.value; ;}
    break;

  case 19:
#line 155 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { if (yyvsp[0].integer.value == 0)
                {
                  acpp_error("division by zero in #if");
                  yyvsp[0].integer.value = 1;
                }
              yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp;
              if (yyval.integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value / yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value / yyvsp[0].integer.value; ;}
    break;

  case 20:
#line 166 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { if (yyvsp[0].integer.value == 0)
                {
                  acpp_error("division by zero in #if");
                  yyvsp[0].integer.value = 1;
                }
              yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp;
              if (yyval.integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value % yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value % yyvsp[0].integer.value; ;}
    break;

  case 21:
#line 177 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = yyvsp[-2].integer.value + yyvsp[0].integer.value;
              yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp; ;}
    break;

  case 22:
#line 180 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = yyvsp[-2].integer.value - yyvsp[0].integer.value;
              yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp; ;}
    break;

  case 23:
#line 183 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp;
              if (yyval.integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value << yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value << yyvsp[0].integer.value; ;}
    break;

  case 24:
#line 189 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp;
              if (yyval.integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value >> yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value >> yyvsp[0].integer.value; ;}
    break;

  case 25:
#line 195 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = (yyvsp[-2].integer.value == yyvsp[0].integer.value);
              yyval.integer.unsignedp = 0; ;}
    break;

  case 26:
#line 198 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = (yyvsp[-2].integer.value != yyvsp[0].integer.value);
              yyval.integer.unsignedp = 0; ;}
    break;

  case 27:
#line 201 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.unsignedp = 0;
              if (yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value <= yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value <= yyvsp[0].integer.value; ;}
    break;

  case 28:
#line 207 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.unsignedp = 0;
              if (yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value >= yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value >= yyvsp[0].integer.value; ;}
    break;

  case 29:
#line 213 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.unsignedp = 0;
              if (yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value < yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value < yyvsp[0].integer.value; ;}
    break;

  case 30:
#line 219 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.unsignedp = 0;
              if (yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp)
                yyval.integer.value = (unsigned) yyvsp[-2].integer.value > yyvsp[0].integer.value;
              else
                yyval.integer.value = yyvsp[-2].integer.value > yyvsp[0].integer.value; ;}
    break;

  case 31:
#line 225 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = yyvsp[-2].integer.value & yyvsp[0].integer.value;
              yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp; ;}
    break;

  case 32:
#line 228 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = yyvsp[-2].integer.value ^ yyvsp[0].integer.value;
              yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp; ;}
    break;

  case 33:
#line 231 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = yyvsp[-2].integer.value | yyvsp[0].integer.value;
              yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp; ;}
    break;

  case 34:
#line 234 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = (yyvsp[-2].integer.value && yyvsp[0].integer.value);
              yyval.integer.unsignedp = 0; ;}
    break;

  case 35:
#line 237 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = (yyvsp[-2].integer.value || yyvsp[0].integer.value);
              yyval.integer.unsignedp = 0; ;}
    break;

  case 36:
#line 240 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = yyvsp[-4].integer.value ? yyvsp[-2].integer.value : yyvsp[0].integer.value;
              yyval.integer.unsignedp = yyvsp[-2].integer.unsignedp || yyvsp[0].integer.unsignedp; ;}
    break;

  case 37:
#line 243 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer = yylval.integer; ;}
    break;

  case 38:
#line 245 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer = yylval.integer; ;}
    break;

  case 39:
#line 247 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"
    { yyval.integer.value = 0;
              yyval.integer.unsignedp = 0; ;}
    break;


    }

/* Line 999 of yacc.c.  */
#line 1372 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.tab.c"

  yyvsp -= yylen;
  yyssp -= yylen;


  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;


  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if YYERROR_VERBOSE
      yyn = yypact[yystate];

      if (YYPACT_NINF < yyn && yyn < YYLAST)
    {
      YYSIZE_T yysize = 0;
      int yytype = YYTRANSLATE (yychar);
      const char* yyprefix;
      char *yymsg;
      int yyx;

      /* Start YYX at -YYN if negative to avoid negative indexes in
         YYCHECK.  */
      int yyxbegin = yyn < 0 ? -yyn : 0;

      /* Stay within bounds of both yycheck and yytname.  */
      int yychecklim = YYLAST - yyn;
      int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
      int yycount = 0;

      yyprefix = ", expecting ";
      for (yyx = yyxbegin; yyx < yyxend; ++yyx)
        if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
          {
        yysize += yystrlen (yyprefix) + yystrlen (yytname [yyx]);
        yycount += 1;
        if (yycount == 5)
          {
            yysize = 0;
            break;
          }
          }
      yysize += (sizeof ("syntax error, unexpected ")
             + yystrlen (yytname[yytype]));
      yymsg = (char *) YYSTACK_ALLOC (yysize);
      if (yymsg != 0)
        {
          char *yyp = yystpcpy (yymsg, "syntax error, unexpected ");
          yyp = yystpcpy (yyp, yytname[yytype]);

          if (yycount < 5)
        {
          yyprefix = ", expecting ";
          for (yyx = yyxbegin; yyx < yyxend; ++yyx)
            if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
              {
            yyp = yystpcpy (yyp, yyprefix);
            yyp = yystpcpy (yyp, yytname[yyx]);
            yyprefix = " or ";
              }
        }
          yyerror (yymsg);
          YYSTACK_FREE (yymsg);
        }
      else
        yyerror ("syntax error; also virtual memory exhausted");
    }
      else
#endif /* YYERROR_VERBOSE */
    yyerror ("syntax error");
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
     error, discard it.  */

      /* Return failure if at end of input.  */
      if (yychar == YYEOF)
        {
      /* Pop the error token.  */
          YYPOPSTACK;
      /* Pop the rest of the stack.  */
      while (yyss < yyssp)
        {
          YYDSYMPRINTF ("Error: popping", yystos[*yyssp], yyvsp, yylsp);
          yydestruct (yystos[*yyssp], yyvsp);
          YYPOPSTACK;
        }
      YYABORT;
        }

      YYDSYMPRINTF ("Error: discarding", yytoken, &yylval, &yylloc);
      yydestruct (yytoken, &yylval);
      yychar = YYEMPTY;

    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*----------------------------------------------------.
| yyerrlab1 -- error raised explicitly by an action.  |
`----------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;  /* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (yyn != YYPACT_NINF)
    {
      yyn += YYTERROR;
      if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
        {
          yyn = yytable[yyn];
          if (0 < yyn)
        break;
        }
    }

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
    YYABORT;

      YYDSYMPRINTF ("Error: popping", yystos[*yyssp], yyvsp, yylsp);
      yydestruct (yystos[yystate], yyvsp);
      yyvsp--;
      yystate = *--yyssp;

      YY_STACK_PRINT (yyss, yyssp);
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  YYDPRINTF ((stderr, "Shifting error token, "));

  *++yyvsp = yylval;


  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#ifndef yyoverflow
/*----------------------------------------------.
| yyoverflowlab -- parser overflow comes here.  |
`----------------------------------------------*/
yyoverflowlab:
  yyerror ("parser stack overflow");
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
  return yyresult;
}


#line 250 "C:\\Users\\Simm\\acppTest\\acpp\\src\\cexp.y"


/* During parsing of a C expression, the pointer to the next character
   is in this variable.  */

static char *lexptr;

/* Take care of parsing a number (anything that starts with a digit).
   Set yylval and return the token type; update lexptr.
   LEN is the number of characters in it.  */

int
parse_number (olen)
     int olen;
{
  register char *p = lexptr;
  register long n = 0;
  register int c;
  register int base = 10;
  register int len = olen;

  /* Check to see if this is a floating point number or not */
  for (c = 0; c < len; c++)
  {
    if (p[c] == '.')
    {
       if (inside_math == 0)
       {
          yyerror ("floating point numbers not allowed in #if expressions");
          return ERROR;
       }
       else
       {
          char *np = (char*)alloca(len + 1);
          bcopy(p, np, len);
          np[len] = '\0';
          yylval.dval = atof(np);
          lexptr += len;
          return DVALUE;
       }
    }
  }

  yylval.integer.unsignedp = 0;

  if (len >= 3 && (!strncmp (p, "0x", 2) || !strncmp (p, "0X", 2))) {
    p += 2;
    base = 16;
    len -= 2;
  }
  else if (*p == '0')
    base = 8;

  while (len > 0) {
    c = *p++;
    len--;
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';

    if (c >= '0' && c <= '9') {
      n *= base;
      n += c - '0';
    } else if (base == 16 && c >= 'a' && c <= 'f') {
      n *= base;
      n += c - 'a' + 10;
    } else {
      /* `l' means long, and `u' means unsigned.  */
      while (1) {
    if (c == 'l' || c == 'L')
      ;
    else if (c == 'u' || c == 'U')
      yylval.integer.unsignedp = 1;
    else
      break;

    if (len == 0)
      break;
    c = *p++;
    len--;
      }
      /* Don't look for any more digits after the suffixes.  */
      break;
    }
  }

  if (len != 0) {
    yyerror ("Invalid number in #if expression");
    return ERROR;
  }

  lexptr = p;

  if (inside_math > 0)
  {
     yylval.dval = (double)n;
     return DVALUE;
  }
  else
  {
     /* If too big to be signed, consider it unsigned.  */
     if (n < 0)
        yylval.integer.unsignedp = 1;

     yylval.integer.value = n;
     return INT;
  }

  return INT;
}

struct token {
  char *operator;
  int token;
};

#define NULL 0

static struct token tokentab2[] = {
  {"&&", AND},
  {"||", OR},
  {"<<", LSH},
  {">>", RSH},
  {"==", EQUAL},
  {"!=", NOTEQUAL},
  {"<=", LEQ},
  {">=", GEQ},
  {NULL, ERROR}
};

/* Read one token, getting characters through lexptr.  */

int
yylex ()
{
  register int c;
  register int namelen;
  register char *tokstart;
  register struct token *toktab;

 retry:

  tokstart = lexptr;
  c = *tokstart;
  /* See if it is a special token of length 2.  */
  for (toktab = tokentab2; toktab->operator != NULL; toktab++)
    if (c == *toktab->operator && tokstart[1] == toktab->operator[1]) {
      lexptr += 2;
      return toktab->token;
    }

  switch (c) {
  case 0:
    return 0;
    
  case ' ':
  case '\t':
  case '\n':
    lexptr++;
    goto retry;
    
#ifdef sgi
  case 'L':
    {
        /* look for wide char constants */
        int c2;
        int i;
        for(i = 1, c2 = *(lexptr+i); is_hor_space[c2];
             ++i,c2 = *(lexptr+i)) 
            /* do nothing on horiz space stuff */   {  }
        if(c2 != '\'') {
            /* Not a wide char const. An identifier */
            break;
        }
        lexptr += i ; /* points to the quote now
            Is wide char constant */
        tokstart = lexptr;
        c = c2;  /* c  now a quote */
    }
#endif
  case '\'':
    lexptr++;
    c = *lexptr++;
    if (c == '\\')
      c = parse_escape (&lexptr);

    /* Sign-extend the constant if chars are signed on target machine.  */
    {
      if (lookup ("__CHAR_UNSIGNED__", sizeof ("__CHAR_UNSIGNED__")-1, -1)
      || ((c >> (CHAR_TYPE_SIZE - 1)) & 1) == 0)
    yylval.integer.value = c & ((1 << CHAR_TYPE_SIZE) - 1);
      else
    yylval.integer.value = c | ~((1 << CHAR_TYPE_SIZE) - 1);
    }

    yylval.integer.unsignedp = 0;
    c = *lexptr++;
    if (c != '\'') {
      yyerror ("Invalid character constant in #if");
      return ERROR;
    }
    
    return CHAR;

    /* some of these chars are invalid in constant expressions;
       maybe do something about them later */
  case '/':
  case '+':
  case '-':
  case '*':
  case '%':
  case '|':
  case '&':
  case '^':
  case '~':
  case '!':
  case '@':
  case '<':
  case '>':
  case '(':
  case ')':
  case '[':
  case ']':
  case '.':
  case '?':
  case ':':
  case '=':
  case '{':
  case '}':
  case ',':
    lexptr++;
    return c;
    
  case '"':
    yyerror ("double quoted strings not allowed in #if expressions");
    return ERROR;
  }

  /* If you are not inside a math expression, then numbers have
   * to be integers. Thus they should contain only numerals from 0 to 9.
   * However, a decimal point is allowed because it is checked for
   * later (inside parse_number).
   * If you are inside a math expression, then the number can start with
   * a numeral or decimal point, and can contain a trailing exponent.
   */
  if ((c >= '0' && c <= '9') || c == '.')
  {
     if (inside_math == 0)
     {
        for (namelen = 0;
             c = tokstart[namelen], is_idchar[c] || c == '.'; 
             namelen++)
           ;
     }
     else
     {
        int exp = 0;
        for (namelen = 0;
             c = tokstart[namelen], is_idchar[c] || c == '.' || (exp == 1 && (c == '-' || c == '+')); 
             namelen++)
        {
           if (exp == 0 && (c == 'e' || c == 'E' || c == 'g' || c == 'G'))
              exp = 1;
           else if (exp == 1 && (c == '-' || c == '+'))
              exp = 0;
        }
     }

     return parse_number (namelen);
  }
  
  if (!is_idstart[c]) {
    yyerror ("Invalid token in expression");
    return ERROR;
  }
  
  /* It is a name.  See how long it is.  */
  
  for (namelen = 0; is_idchar[tokstart[namelen]]; namelen++)
    ;
  
  lexptr += namelen;
  return NAME;
}


/* Parse a C escape sequence.  STRING_PTR points to a variable
   containing a pointer to the string to parse.  That pointer
   is updated past the characters we use.  The value of the
   escape sequence is returned.

   A negative value means the sequence \ newline was seen,
   which is supposed to be equivalent to nothing at all.

   If \ is followed by a null character, we return a negative
   value and leave the string pointer pointing at the null character.

   If \ is followed by 000, we return 0 and leave the string pointer
   after the zeros.  A value of 0 does not mean end of string.  */

int
parse_escape (string_ptr)
     char **string_ptr;
{
  register int c = *(*string_ptr)++;
  switch (c)
    {
    case 'a':
      return TARGET_BELL;
    case 'b':
      return TARGET_BS;
    case 'e':
      return 033;
    case 'f':
      return TARGET_FF;
    case 'n':
      return TARGET_NEWLINE;
    case 'r':
      return TARGET_CR;
    case 't':
      return TARGET_TAB;
    case 'v':
      return TARGET_VT;
    case '\n':
      return -2;
    case 0:
      (*string_ptr)--;
      return 0;
    case '^':
      c = *(*string_ptr)++;
      if (c == '\\')
    c = parse_escape (string_ptr);
      if (c == '?')
    return 0177;
      return (c & 0200) | (c & 037);
      
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
      {
    register int i = c - '0';
    register int count = 0;
    while (++count < 3)
      {
        c = *(*string_ptr)++;
        if (c >= '0' && c <= '7')
          i = (i << 3) + c - '0';
        else
          {
        (*string_ptr)--;
        break;
          }
      }
    if ((i & ~((1 << CHAR_TYPE_SIZE) - 1)) != 0)
      {
        i &= (1 << CHAR_TYPE_SIZE) - 1;
        acpp_warning ("octal character constant does not fit in a byte");
      }
    return i;
      }
    case 'x':
      {
    register int i = 0;
    register int count = 0;
    for (;;)
      {
        c = *(*string_ptr)++;
        if (c >= '0' && c <= '9')
          i = (i << 4) + c - '0';
        else if (c >= 'a' && c <= 'f')
          i = (i << 4) + c - 'a' + 10;
        else if (c >= 'A' && c <= 'F')
          i = (i << 4) + c - 'A' + 10;
        else
          {
        (*string_ptr)--;
        break;
          }
      }
    if ((i & ~((1 << BITS_PER_UNIT) - 1)) != 0)
      {
        i &= (1 << BITS_PER_UNIT) - 1;
        acpp_warning ("hex character constant does not fit in a byte");
      }
    return i;
      }
    default:
      return c;
    }
}

void
yyerror (s)
     char *s;
{
  acpp_error (s);
  longjmp (parse_return_error, 1);
}

/* This page contains the entry point to this file.  */

#if 0
/* THIS DOESN'T EVEN BEGIN TO WORK */
/* Parse STRING as an assertion (the AT&T ANSI cpp extension), and complain if
 * this fails to use up all of the contents of STRING */
int
parse_assertion_extension (buf)
     char *buf;
{
  int token;
  int c;
  int negate = 0;
  int value = 0;
  
  for(c = *buf; c != '#'; c = *buf++) {
    if (c == '!')
      negate = !negate;
    else if(!isspace(c))
      acpp_error("Unexpected character in assertion expression\n");
  }
  
  /* buf is now one past the '#' character */
  lexptr = buf;

  if((token = yylex()) != NAME)
    acpp_error("Syntax for assertion conditionals is '#if #assertion-name(tokens)'\n");

  /* for the moment - check the syntax, but don't actually *do* anything. */
  return (negate ? !value : value);
}
#endif

/* Parse STRING as an expression, and complain if this fails
   to use up all of the contents of STRING.  */
/* We do not support C comments.  They should be removed before
   this function is called.  */

int
parse_c_expression (string)
     char *string;
{
  lexptr = string;
  
  if (lexptr == 0 || *lexptr == 0) {
    acpp_error("empty #if expression");
    return 0;           /* don't include the #if group */
  }

  /* if there is some sort of scanning error, just return 0 and assume
     the parsing routine has printed an error message somewhere.
     there is surely a better thing to do than this.     */
  if (setjmp (parse_return_error))
    return 0;

  if (yyparse ())
    return 0;           /* actually this is never reached
                   the way things stand. */
  if (*lexptr)
    acpp_error("Junk after end of expression.");

  return expression_value;  /* set by yyparse () */
}

double
parse_c_math_expression (string)
     char *string;
{
  lexptr = string;
  
  if (lexptr == 0 || *lexptr == 0) {
    acpp_error("empty #if expression");
    return 0;           /* don't include the #if group */
  }

  /* if there is some sort of scanning error, just return 0 and assume
     the parsing routine has printed an error message somewhere.
     there is surely a better thing to do than this.     */
  if (setjmp (parse_return_error))
    return 0;

  if (yyparse ())
    return 0;           /* actually this is never reached
                   the way things stand. */
  if (*lexptr)
    acpp_error("Junk after end of expression.");

  return math_value + (double)expression_value; /* set by yyparse () */
}

#ifdef TEST_EXP_READER
/* main program, for testing purposes. */
main ()
{
  int n, c;
  char buf[1024];
  extern int yydebug;
/*
  yydebug = 1;
*/
  initialize_random_junk ();

  for (;;) {
    printf ("enter expression: ");
    n = 0;
    while ((buf[n] = getchar ()) != '\n' && buf[n] != EOF)
      n++;
    if (buf[n] == EOF)
      break;
    buf[n] = '\0';
    printf ("parser returned %d\n", parse_c_expression (buf));
  }
}

/* table to tell if char can be part of a C identifier. */
unsigned char is_idchar[256];
/* table to tell if char can be first char of a c identifier. */
unsigned char is_idstart[256];
#ifndef sgi
/* table to tell if c is horizontal space.  isspace () thinks that
   newline is space; this is not a good idea for this program. */
char is_hor_space[256];
#endif

/*
 * initialize random junk in the hash table and maybe other places
 */
initialize_random_junk ()
{
  register int i;

  /*
   * Set up is_idchar and is_idstart tables.  These should be
   * faster than saying (is_alpha (c) || c == '_'), etc.
   * Must do set up these things before calling any routines tthat
   * refer to them.
   */
  for (i = 'a'; i <= 'z'; i++) {
    ++is_idchar[i - 'a' + 'A'];
    ++is_idchar[i];
    ++is_idstart[i - 'a' + 'A'];
    ++is_idstart[i];
  }
  for (i = '0'; i <= '9'; i++)
    ++is_idchar[i];
  ++is_idchar['_'];
  ++is_idstart['_'];
#ifdef DOLLARS_IN_IDENTIFIERS
  ++is_idchar['$'];
  ++is_idstart['$'];
#endif

  /* horizontal space table */
  ++is_hor_space[' '];
  ++is_hor_space['\t'];
}

struct hashnode *
lookup (name, len, hash)
     char *name;
     int len;
     int hash;
{
  return (DEFAULT_SIGNED_CHAR) ? 0 : ((struct hashnode *) -1);
}
#endif


