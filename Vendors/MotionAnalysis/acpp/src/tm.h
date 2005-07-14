/* Definitions of target machine for GNU compiler.  MIPS version.
   Contributed by   A. Lichnewsky, lich@inria.inria.fr
   Copyright (C) 1989 Free Software Foundation, Inc.

This file is part of GNU CC.

GNU CC is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 1, or (at your option)
any later version.

GNU CC is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with GNU CC; see the file COPYING.  If not, write to
the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.  */

/* ??? This file needs to be reformatted so that it looks like the
   rest of GCC. ???  */

/* Currently we know of 2 MIPS machines: MIPS- M series, and DECStations.  */
/* Therefore we discriminate by declaring DECStations with */
/* #define DECSTATION */

/* Names to predefine in the preprocessor for this target machine.  */

#define CPP_PREDEFINES "-Dmips -Dunix"
#ifdef DECSTATION
#define CPP_SPEC "-DR3000 -DLANGUAGE_C -DMIPSEL -DSYSTYPE_BSD "
#else
#define CPP_SPEC "-DR3000  -Dhost_mips -DMIPSEB -DSYSTYPE_BSD -DLANGUAGE_C "
#endif

/*----------------------------------------------------------------------

SWITCHES:

    -O    optimization. Implies -fstrength-reduce -fomit-frame-pointer
    -O2   optimization. Implies -O

          Tries to make use of short displacements using the
          Sdata and Sbss sections. This uses the -G switches of as and ld.

    -G <size>
          Pass size to as and ld. Default -G 8.

    -mG0 -mG1 -mG2
          Construct a size to be passed to GCC for Data / Sdata selection.

          Value is ( (i=G0 + 2 G1 + 4 G2) , (i < 6) ? ( 1<<i) :(1 <<(i+3)))
          Same value should be passed to as + ld using -G.

	  Default = -mG1 -mG0 (Value = 8).

    -G32  Implies -G 32 -mG2 -mnG1 -mG0.


    -bestGnum
          Pass -bestGnum flag to ld. This helps setting best value for
          the -G parameter.

----------------------------------------------------------------------*/



/***********************************************************************

WARNING:

    No attempt to select (configure) the -B and -I parameters has been
    made inside this version of gcc. They should be made (eg. thru a
    shell script).

    -I should be set in such a way that the include file "va-mips.h"
    gets included (via "varargs.h") for varargs. Otherwise gcc will not
    bootstrap -- and produce wrong code for varargs.


***********************************************************************/


/* Switch  Recognition by gcc.c   */

#ifdef SWITCH_TAKES_ARG
#undef SWITCH_TAKES_ARG
#endif

#define SWITCH_TAKES_ARG(CHAR)						\
  ((CHAR) == 'D' || (CHAR) == 'U' || (CHAR) == 'o'			\
   || (CHAR) == 'e' || (CHAR) == 'T' || (CHAR) == 'u'			\
   || (CHAR) == 'I' || (CHAR) == 'Y' || (CHAR) == 'm'			\
   || (CHAR) == 'L' || (CHAR) == 'G')




/* Extra switches sometimes passed to the assembler.  */

#define ASM_SPEC   "%{O:-O2} %{O2: -O2} %{!G32: %{G*}}			\
%{!G:%{!G32: -G 8}} %{G32: -G 32}"


/* Extra switches sometimes passed to the loader.  */


#define LINK_SPEC  "%{!G32:%{G*}					\
%{!G:%{!G32:%{mG0:%eYou should include ld/as option -G}			\
%{mG1:%eYou should include ld/as option -G}				\
%{mG2:%eYou should include ld/as option -G}				\
 -G 8}}}								\
%{G32: -G 32}								\
%{bestGnum: -bestGnum} "

/* CC1 SPECS */

#define CC1_SPEC   "%{!O2:%{O:-O -fstrength-reduce -fomit-frame-pointer}}\
                    %{O2:-O -fstrength-reduce -fomit-frame-pointer -mgpOPT}\
                    %{g:%eThis port of GCC does not support -g flag}	\
                    %{G32: -mG2 -mnG1 }					\
                    %{G32:%{!O2:%eOption -G32 may require -O2}}"


/* Print subsidiary information on the compiler version in use.  */

#ifdef DECSTATION
#define TARGET_VERSION printf (" (AL-MIPS 1.10) <DECStation>\n");
				/* Depends on MIPS ASM. */
#else
#define TARGET_VERSION printf (" (AL-MIPS 1.10) <MIPS>\n");
				/* Depends on MIPS ASM. */
#endif
#define TARGET_VERSNUM "1 10"

/* Do not Generate DBX debugging information.  */

/* #define DBX_DEBUGGING_INFO */

/* Run-time compilation parameters selecting different hardware subsets.  */

extern int target_flags;

/* Macros used in the machine description to test the flags.  */

/* Nonzero if compiling code that Unix assembler can assemble.  */
#define TARGET_UNIX_ASM (target_flags & 1)
				/* Debug Mode */
#define TARGET_DEBUG_MODE (target_flags & 2)
#define TARGET_DEBUGA_MODE (target_flags & 4)
#define TARGET_DEBUGB_MODE (target_flags & 16)
#define TARGET_DEBUGC_MODE (target_flags & 32)
#define TARGET_DEBUGD_MODE (target_flags & 64)
				/* Register Naming in .s ($21 vs. $a0) */
#define TARGET_NAME_REGS (target_flags & 8)
				/* Use addu / subbu or get FIXED_OVFL TRAPS */
#define TARGET_NOFIXED_OVFL (target_flags & 128)
				/* Optimize for Sdata/Sbss */
#define TARGET_GP_OPT (target_flags & 4096)
#define TARGET_GVALUE ((target_flags >> 8 ) & 0xf)



/* Macro to define tables used to set the flags.
   This is a list in braces of pairs in braces,
   each pair being { "NAME", VALUE }
   where VALUE is the bits to set or minus the bits to clear.
   An empty string NAME is used to identify the default VALUE.  */

#define TARGET_SWITCHES							\
  { {"unix", 1},							\
    {"gnu", -1},							\
    {"debug", 2 }, 		/* RELOAD and CONSTRAINTS Related DEBUG */\
    {"nodebug", -2 },							\
    {"debuga",   4 }, 		/* CALLING SEQUENCE RELATED DEBUG */	\
    {"nodebuga", -4 },							\
    {"debugb",   16 }, 		/* GLOBAL/LOCAL ALLOC  DEBUG */		\
    {"nodebugb", -16 },							\
    {"debugc",   32 }, 		/* SPILL/RELOAD REGISTER ALLOCATOR DEBUG */\
    {"nodebugc", -32 },							\
    {"debugd",   64 }, 		/* CSE DEBUG */				\
    {"nodebugd", -64 },							\
    {"rnames",   8 }, 		/* Output register names like $a0 */	\
    {"nornames", -8 },  	/* Output register numbers like $21 */	\
    {"nofixed-ovfl",128},       /* use addu and subu                */	\
    {"fixed-ovfl", -128},       /* use add and sub                */	\
				/* Following used to support the data/sdata */\
				/* feature */				\
    {"G0",256},								\
    {"nG0",-256},							\
    {"G1",512},								\
    {"nG1",-512},							\
    {"G2",1024},							\
    {"nG2",-1024},							\
    {"gpOPT", 4096},		/* DO the full GP optimization data/sdata.. */\
    {"ngpOPT", -4096},\
    { "", TARGET_DEFAULT}}

/* Default target_flags if no switches specified.  */

#define TARGET_DEFAULT 897

/* Default GVALUE  (data item size threshold for selection of Sdata/data)
   is computed : GVALUE ==  ( ((i=G0+2*G1+4*G2) < 6)
				        ? 1<<i
					: 1<< (i+6))
*/
#define MIPS_GVALUE_DEFAULT 8

/* Target machine storage layout */

/* Define this if most significant bit is lowest numbered
   in instructions that operate on numbered bit-fields.
*/
/* #define BITS_BIG_ENDIAN */

/* Define this if most significant byte of a word is the lowest numbered.
*/
#ifndef DECSTATION
#define BYTES_BIG_ENDIAN
#endif
/* Define this if most significant word of a multiword number is numbered.
*/
#ifndef DECSTATION
#define WORDS_BIG_ENDIAN
#endif
/* Number of bits in an addressible storage unit */
#define BITS_PER_UNIT 8

/* Width in bits of a "word", which is the contents of a machine register.
   Note that this is not necessarily the width of data type `int';
   if using 16-bit ints on a 68000, this would still be 32.
   But on a machine with 16-bit registers, this would be 16.  */
#define BITS_PER_WORD 32

/* Width of a word, in units (bytes).  */
#define UNITS_PER_WORD 4

/* Width in bits of a pointer.
   See also the macro `Pmode' defined below.  */
#define POINTER_SIZE 32

/* Allocation boundary (in *bits*) for storing pointers in memory.  */
#define POINTER_BOUNDARY 32

/* Allocation boundary (in *bits*) for storing arguments in argument list.  */
#define PARM_BOUNDARY 32

/* Give parms extra alignment, up to this much, if their types want it.  */
#define MAX_PARM_BOUNDARY 64

/* Allocation boundary (in *bits*) for the code of a function.  */
#define FUNCTION_BOUNDARY 32

/* Alignment of field after `int : 0' in a structure.  */
#define EMPTY_FIELD_BOUNDARY 32

/* Every structure's size must be a multiple of this.  */
#define STRUCTURE_SIZE_BOUNDARY 16

/* There is no point aligning anything to a rounder boundary than this.  */
#define BIGGEST_ALIGNMENT 64

/* Define this if move instructions will actually fail to work
   when given unaligned data.  */
#define STRICT_ALIGNMENT

/* Standard register usage.  */

/* Number of actual hardware registers.
   The hardware registers are assigned numbers for the compiler
   from 0 to just below FIRST_PSEUDO_REGISTER.
   All registers that the compiler knows about must be given numbers,
   even those that are not normally considered general registers.  */
#define FIRST_PSEUDO_REGISTER 64

/* 1 for registers that have pervasive standard uses
   and are not available for the register allocator.

   On the MIPS, see conventions, page D-2

   I have chosen not to  take Multiply/Divide HI,LO or PC into
   account.
*/
#define FIXED_REGISTERS {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
		         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1,\
		         1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,\
		         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0	\
}


/* 1 for registers not available across function calls.
   These must include the FIXED_REGISTERS and also any
   registers that can be used without being saved.
   The latter must include the registers where values are returned
   and the register where structure-value addresses are passed.
   Aside from that, you can include as many other registers as you like.  */
#define CALL_USED_REGISTERS {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,\
		             0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1,\
		             1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,\
		             1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0\
}


/* Return number of consecutive hard regs needed starting at reg REGNO
   to hold something of mode MODE.
   This is ordinarily the length in words of a value of mode MODE
   but can be less for certain modes in special long registers.

   On the MIPS, all general registers are one word long. I have chosen to
   use Floating point register pairs.
*/
#define HARD_REGNO_NREGS(REGNO, MODE)					\
 (((MODE == SFmode) ||(MODE == DFmode)) ? 2 :				\
  ((GET_MODE_SIZE (MODE) + UNITS_PER_WORD - 1) / UNITS_PER_WORD))

/* Value is 1 if hard register REGNO can hold a value of machine-mode MODE.
   On the MIPS, all general registers can hold all  modes, except
   FLOATING POINT.  */

#define HARD_REGNO_MODE_OK(REGNO, MODE)					\
   ((REGNO) < 32 ? (int) (((MODE) != SFmode) && ((MODE) != DFmode))	\
    : (int) (((MODE) == SFmode || (MODE) == DFmode)			\
	     && ((REGNO) & 1) == 0))


/* Value is 1 if it is a good idea to tie two pseudo registers
   when one has mode MODE1 and one has mode MODE2.
   If HARD_REGNO_MODE_OK could produce different values for MODE1 and MODE2,
   for any hard reg, then this must be 0 for correct output.  */
#define MODES_TIEABLE_P(MODE1, MODE2)					\
  (   ((MODE1) == SFmode || (MODE1) == DFmode)				\
   == ((MODE2) == SFmode || (MODE2) == DFmode))

/* MIPS pc is apparently not overloaded on a register.  */
/* #define PC_REGNUM 15                                 */

/* Register to use for pushing function arguments.  */
#define STACK_POINTER_REGNUM 29

/* Base register for access to local variables of the function.  */
#define FRAME_POINTER_REGNUM 30

/* Value should be nonzero if functions must have frame pointers.
   Zero means the frame pointer need not be set up (and parms
   may be accessed via the stack pointer) in functions that seem suitable.
   This is computed in `reload', in reload1.c.  */

/* This is now 1 because we don't know until too late
   whether the function is a varargs function.
   Such functions currently require extra stack slots on the mips.  */
#define FRAME_POINTER_REQUIRED 1

/* Base register for access to arguments of the function.  */
#define ARG_POINTER_REGNUM FRAME_POINTER_REGNUM

/* Register in which static-chain is passed to a function.  */
#define STATIC_CHAIN_REGNUM 2

/* Register in which address to store a structure value
   is passed to a function.  */
#define STRUCT_VALUE_REGNUM 3

/* Define the classes of registers for register constraints in the
   machine description.  Also define ranges of constants.

   One of the classes must always be named ALL_REGS and include all hard regs.
   If there is more than one class, another class must be named NO_REGS
   and contain no registers.

   The name GENERAL_REGS must be the name of a class (or an alias for
   another name such as ALL_REGS).  This is the class of registers
   that is allowed by "g" or "r" in a register constraint.
   Also, registers outside this class are allocated only when
   instructions express preferences for them.

   The classes must be numbered in nondecreasing order; that is,
   a larger-numbered class must never be contained completely
   in a smaller-numbered class.

   For any two classes, it is very desirable that there be another
   class that represents their union.  */

/* The MIPS has general and floating point registers,
*/


enum reg_class  { NO_REGS, GR_REGS, FP_REGS, ALL_REGS, LIM_REG_CLASSES } ;

#define N_REG_CLASSES (int) LIM_REG_CLASSES

#define GENERAL_REGS GR_REGS

/* Give names of register classes as strings for dump file.   */

#define REG_CLASS_NAMES							\
 {"NO_REGS", "GR_REGS", "FP_REGS", "ALL_REGS" }

/* Define which registers fit in which classes.
   This is an initializer for a vector of HARD_REG_SET
   of length N_REG_CLASSES.  */

#define REG_CLASS_CONTENTS {{0x00000000, 0x00000000},			\
                            {0xffffffff, 0x00000000},			\
                            {0x00000000, 0xffffffff},			\
			    {0xffffffff, 0xffffffff}}


/* The same information, inverted:
   Return the class number of the smallest class containing
   reg number REGNO.  This could be a conditional expression
   or could index an array.  */

#define REGNO_REG_CLASS(REGNO)						\
   ( (REGNO >= 32) ? FP_REGS : GR_REGS)

/* Define a table that lets us find quickly all the reg classes
   containing a given one.  This is the initializer for an
   N_REG_CLASSES x N_REG_CLASSES array of reg class codes.
   Row N is a sequence containing all the class codes for
   classes that contain all the regs in class N.  Each row
   contains no duplicates, and is terminated by LIM_REG_CLASSES.  */

/* We give just a dummy for the first element, which is for NO_REGS.  */
/* #define REG_CLASS_SUPERCLASSES  {{LIM_REG_CLASSES},			\
  {GR_REGS,ALL_REGS,LIM_REG_CLASSES},					\
  {FP_REGS,ALL_REGS,LIM_REG_CLASSES},					\
  {ALL_REGS,LIM_REG_CLASSES}						\
}
*/
/* We give just a dummy for the first element, which is for NO_REGS.  */
#define REG_CLASS_SUPERCLASSES  {{LIM_REG_CLASSES},			\
  {ALL_REGS,LIM_REG_CLASSES},						\
  {ALL_REGS,LIM_REG_CLASSES},						\
  {LIM_REG_CLASSES}							\
}

/* The inverse relationship:
   for each class, a list of all reg classes contained in it.  */
#define REG_CLASS_SUBCLASSES						\
{{LIM_REG_CLASSES},							\
  {GR_REGS,LIM_REG_CLASSES},						\
  {FP_REGS,LIM_REG_CLASSES},\
  {GR_REGS, FP_REGS, ALL_REGS, LIM_REG_CLASSES}\
}

/* Define a table that lets us find quickly the class
   for the subunion of any two classes.

   We say "subunion" because the result need not be exactly
   the union; it may instead be a subclass of the union
   (though the closer to the union, the better).
   But if it contains anything beyond union of the two classes,
   you will lose!

   This is an initializer for an N_REG_CLASSES x N_REG_CLASSES
   array of reg class codes.  The subunion of classes C1 and C2
   is just element [C1, C2].  */

#define REG_CLASS_SUBUNION  {{NO_REGS,  GR_REGS,   FP_REGS,  ALL_REGS},	\
 {GR_REGS,  GR_REGS,   ALL_REGS, ALL_REGS},				\
 {FP_REGS,  ALL_REGS,  FP_REGS,  ALL_REGS},				\
 {ALL_REGS, ALL_REGS,  ALL_REGS, ALL_REGS}}

/* The class value for index registers, and the one for base regs.  */

#define INDEX_REG_CLASS GR_REGS
#define BASE_REG_CLASS  GR_REGS


				/* REGISTER AND CONSTANT CLASSES
				 */

/* Get reg_class from a letter such as appears in the machine
description.  */
				/* DEFINED REGISTER CLASSES:
				**
				** 'f'     : Floating point registers
				** 'y'     : General register when used to
				**           transfer chunks of Floating point
				**           with mfc1 mtc1 insn
				 */

#define REG_CLASS_FROM_LETTER(C)					\
   ((C) == 'f' ? FP_REGS:						\
     (C) == 'y' ? GR_REGS:NO_REGS)

/* The letters I, J, K, L and M in a register constraint string
   can be used to stand for particular ranges of immediate operands.
   This macro defines what the ranges are.
   C is the letter, and VALUE is a constant value.
   Return 1 if VALUE is in the range specified by C.  */

/*   For MIPS, `I' is used for the range of constants an insn
                   can actually contain (16 bits signed integers).
               `J' is used for the range which is just zero (since that is
	           available as $R0).
*/

#define SMALL_INT(X) ((unsigned) (INTVAL (X) + 0x10000) < 0x20000)

#define CONST_OK_FOR_LETTER_P(VALUE, C)					\
  ((C) == 'I' ? (unsigned) ((VALUE) + 0x10000) < 0x20000		\
   : (C) == 'J' ? (VALUE) == 0						\
   : 0)

/* Similar, but for floating constants, and defining letters G and H.
   Here VALUE is the CONST_DOUBLE rtx itself.  */

				/* DEFINED FLOATING CONSTANT CLASSES:
				**
				** 'G'     : Floating point 0
				 */
#define CONST_DOUBLE_OK_FOR_LETTER_P(VALUE, C)				\
  ((C) == 'G' && XINT (VALUE, 0) == 0 && XINT (VALUE, 1) == 0)

/* Given an rtx X being reloaded into a reg required to be
   in class CLASS, return the class of reg to actually use.
   In general this is just CLASS; but on some machines
   in some cases it is preferable to use a more restrictive class.  */

#define PREFERRED_RELOAD_CLASS(X,CLASS)					\
    (((GET_MODE(X) == SFmode) || (GET_MODE(X) == DFmode))? FP_REGS  :	\
     ((GET_MODE(X) == VOIDmode) ? GR_REGS :(CLASS)))

/* Same but Mode has been extracted already
*/

#define PREFERRED_RELOAD_CLASS_FM(X,CLASS)				\
    ((((X) == SFmode) || ((X) == DFmode))? FP_REGS  :			\
     (((X) == VOIDmode) ? GR_REGS :(CLASS)))

/* Return the maximum number of consecutive registers
   needed to represent mode MODE in a register of class CLASS.  */

#define CLASS_MAX_NREGS(CLASS, MODE)					\
 ((((MODE) == DFmode) || ((MODE) == SFmode)) ? 2			\
  : ((MODE) == VOIDmode)? ((CLASS) == FP_REGS ? 2 :1)			\
  : ((GET_MODE_SIZE (MODE) + UNITS_PER_WORD - 1) / UNITS_PER_WORD))


/* Stack layout; function entry, exit and calling.  */

/* Define this if pushing a word on the stack
   makes the stack pointer a smaller address.  */
#define STACK_GROWS_DOWNWARD

/* Define this if the nominal address of the stack frame
   is at the high-address end of the local variables;
   that is, each additional local variable allocated
   goes at a more negative offset in the frame.  */
#define FRAME_GROWS_DOWNWARD

/* Offset within stack frame to start allocating local variables at.
   If FRAME_GROWS_DOWNWARD, this is the offset to the END of the
   first local allocated.  Otherwise, it is the offset to the BEGINNING
   of the first local allocated.  */
#define STARTING_FRAME_OFFSET -4

/* If we generate an insn to push BYTES bytes,
   this says how many the stack pointer really advances by.
   On the vax, sp@- in a byte insn really pushes a word.  */

/* #define PUSH_ROUNDING(BYTES) 0 */


/* Offset of first parameter from the argument pointer register value.  */
#define FIRST_PARM_OFFSET(FNDECL) 0

/* Offset from top-of-stack address to location to store the
   function parameter if it can't go in a register.
   Addresses for following parameters are computed relative to this one.  */
#define FIRST_PARM_CALLER_OFFSET(FNDECL) 0

/* When a parameter is passed in a register, stack space is still
   allocated for it.  */
/* For the MIPS, stack space must be allocated, cf Asm Lang Prog Guide
   page 7-8

   BEWARE that some space is also allocated for non existing arguments
   in register. In case an argument list is of form
   GF used registers are a0 (a2,a3), but we should push over a1... not
    used..
*/
#define REG_PARM_STACK_SPACE
   /* Align stack frames on 64 bits (Double Word )
   */

#define STACK_BOUNDARY 64

/* For the MIPS, there seems to be a minimum to the amount of stack space
   used... for varargs using functions.
   evidence comes from the dis-assembled version of printf:

 cc (cc)
        Mips Computer Systems 1.31
 /usr/lib/cpp1.31


printf:
  [printf.c:  14] 0x400510:     27bdffe8        addiu   sp,sp,-24
  [printf.c:  14] 0x400514:     afbf0014        sw      ra,20(sp)
  [printf.c:  14] 0x400518:     afa5001c        sw      a1,28(sp)
  [printf.c:  14] 0x40051c:     afa60020        sw      a2,32(sp)
  [printf.c:  14] 0x400520:     afa70024        sw      a3,36(sp)
  [printf.c:  18] 0x400524:     27a5001c        addiu   a1,sp,28

  it is however OK for functions that do not take arguments to have 0 size
  frames.

*/

#define STACK_ARGS_ADJUST(SIZE)						\
{									\
  SIZE.constant += 4;							\
  if (SIZE.var)								\
    {									\
      rtx size1 = ARGS_SIZE_RTX (SIZE);					\
      rtx rounded = gen_reg_rtx (SImode);				\
      rtx label = gen_label_rtx ();					\
      emit_move_insn (rounded, size1);					\
      /* Needed: insns to jump to LABEL if ROUNDED is < 16.  */		\
      abort ();								\
      emit_move_insn (rounded, gen_rtx (CONST_INT, VOIDmode, 16));	\
      emit_label (label);						\
      SIZE.constant = 0;						\
      SIZE.var = (tree) rounded;					\
    }									\
  else if (SIZE.constant < 16)						\
    SIZE.constant = 16;							\
}

/* Value is 1 if returning from a function call automatically
   pops the arguments described by the number-of-args field in the call.
   FUNTYPE is the data type of the function (as a tree),
   or for a library call it is an identifier node for the subroutine name.  */

#define RETURN_POPS_ARGS(FUNTYPE) 0


/* Define how to find the value returned by a function.
   VALTYPE is the data type of the value (as a tree).
   If the precise function being called is known, FUNC is its FUNCTION_DECL;
   otherwise, FUNC is 0.  */

#define FUNCTION_VALUE(VALTYPE, FUNC)					\
  gen_rtx (REG, TYPE_MODE (VALTYPE),					\
  (TYPE_MODE (VALTYPE) == SFmode) ||(TYPE_MODE (VALTYPE) == DFmode)?32 : 2)

/* Define how to find the value returned by a library function
   assuming the value has mode MODE.  */


#define LIBCALL_VALUE(MODE)  gen_rtx (REG, MODE,			\
   ((MODE) == DFmode || ( MODE) == SFmode) ? 32 : 2)

/* 1 if N is a possible register number for a function value.
   On the MIPS, R2 R3 and F0 F2 are the only register thus used.  */
/* Currently, R2 and F0 are only implemented  here ( C has no complex type)
*/

#define FUNCTION_VALUE_REGNO_P(N) ((N) == 2 || (N) == 32)

/* 1 if N is a possible register number for function argument passing.
    */

#define FUNCTION_ARG_REGNO_P(N) (((N) < 8 && (N) > 3)			\
                                ||((N) < 48 && (N) > 44 && (0 == (N) % 2)))

/* Define a data type for recording info about an argument list
   during the scan of that argument list.  This data type should
   hold all necessary information about the function itself
   and about the args processed so far, enough to enable macros
   such as FUNCTION_ARG to determine where the next arg should go.
*/
				/* On MIPS the following automaton decides */
				/* where to put things. */
				/* If you dont believe it, look at Gerry Kane*/
				/* 's book page D-22 */

#define CUMULATIVE_ARGS struct    { enum arg_state arg_rec_state;int restype,arg_num;}

enum arg_state     { ARG_STA_INIT =0,
		     ARG_STA_F    =1, /* $f12 */
		     ARG_STA_FF   =2, /* $f12 $f14 */
		     ARG_STA_FG   =3, /* $f12 $6   */
		     ARG_STA_FGG  =4, /* $f12 $6 $7 */
		     ARG_STA_FGF  =5, /* $f12 $6 STACK */
		     ARG_STA_G    =6, /* $4 */
		     ARG_STA_GF   =7, /* $4  ($6,$7) */
		     ARG_STA_GG   =8, /* $4 $5 */
		     ARG_STA_GGF  =9, /* $4 $5 ($6,$7) */
		     ARG_STA_GGG  =10,/* $4 $5 $6 */
		     ARG_STA_GGGF =11,/* $4 $5 $6 STACK */
		     ARG_STA_GGGG =12 /* $4 $5 $6 $7 */
		     };
#define ARG_STA_AUTOMA							\
{									\
  {ARG_STA_F,ARG_STA_G,44,4        },   /* ARG_STA_INIT */		\
  {ARG_STA_FF,ARG_STA_FG,46,6      },   /* ARG_STA_F    */		\
  {ARG_STA_FF,ARG_STA_FF,-1,-1     },   /* ARG_STA_FF   */		\
  {ARG_STA_FGF,ARG_STA_FGG,-1,7    },   /* ARG_STA_FG   */		\
  {ARG_STA_FGG,ARG_STA_FGG,-1,-1   },   /* ARG_STA_FGG  */		\
  {ARG_STA_FGF,ARG_STA_FGF,-1,-1   },   /* ARG_STA_FGF  */		\
  {ARG_STA_GF,ARG_STA_GG,-2,5      },   /* ARG_STA_G    */		\
  {ARG_STA_GF,ARG_STA_GF,-1,-1     },   /* ARG_STA_GF   */		\
  {ARG_STA_GGF,ARG_STA_GGG,-2,6    },   /* ARG_STA_GG   */		\
  {ARG_STA_GGF,ARG_STA_GGF,-1,-1   },   /* ARG_STA_GGF  */		\
  {ARG_STA_GGGF,ARG_STA_GGGG,-1,7  },   /* ARG_STA_GGG  */		\
  {ARG_STA_GGGF,ARG_STA_GGGF,-1,-1 },   /* ARG_STA_GGGF */		\
  {ARG_STA_GGGG,ARG_STA_GGGG,-1,-1 }    /* ARG_STA_GGGG */		\
}

/* Initialize a variable CUM of type CUMULATIVE_ARGS
   for a call to a function whose data type is FNTYPE.
   For a library call, FNTYPE is 0.

*/

#define INIT_CUMULATIVE_ARGS(CUM,FNTYPE) ((CUM.arg_rec_state) = ARG_STA_INIT,\
   (CUM.arg_num) = 0, (CUM.restype = (int)VOIDmode))

/* Update the data in CUM to advance over an argument
   of mode MODE and data type TYPE.
   (TYPE is null for libcalls where that information may not be available.)  */

#define FUNCTION_ARG_ADVANCE(CUM, MODE, TYPE, NAMED)			\
			    ( function_arg_advance(&CUM,MODE,TYPE));

extern  enum arg_state function_arg_advance();

/* Determine where to put an argument to a function.
   Value is zero to push the argument on the stack,
   or a hard register in which to store the argument.

   MODE is the argument's machine mode.
   TYPE is the data type of the argument (as a tree).
    This is null for libcalls where that information may
    not be available.
   CUM is a variable of type CUMULATIVE_ARGS which gives info about
    the preceding args and about the function being called.
   NAMED is nonzero if this argument is a named parameter
    (otherwise it is an extra parameter matching an ellipsis).  */



#define FUNCTION_ARG(CUM, MODE, TYPE, NAMED)				\
     ( (rtx) function_arg(&CUM,MODE,TYPE,NAMED))

/* For an arg passed partly in registers and partly in memory,
   this is the number of registers used.
   For args passed entirely in registers or entirely in memory, zero.
*/

#define FUNCTION_ARG_PARTIAL_NREGS(CUM, MODE, TYPE, NAMED) (0)


/* This macro generates the assembly code for function entry.
   FILE is a stdio stream to output the code to.
   SIZE is an int: how many units of temporary storage to allocate.
   Refer to the array `regs_ever_live' to determine which registers
   to save; `regs_ever_live[I]' is nonzero if register number I
   is ever used in the function.  This macro is responsible for
   knowing which registers should not be saved even if used.  */


/* ALIGN FRAMES on double word boundaries */

#define AL_ADJUST_ALIGN(LOC) (((LOC)+7) & 0xfffffff8)


/* The problem of Varargs  comes from the register passing conventions
   for Floating Point data. There is a conflict when we send registers
   back  to stack between registers $4,$5 $6,$7 and $f12, $f14.

   The current implementation:
      a/ tries to figure out if the current routines uses varargs.(It becomes
         ``suspect''.) This is currently done by looking for a special
	 static character string constant.

      b/when a function is suspected of using varags,  a larger reg
        save_area is allocated which will hold regs f12 and f14. The varargs
	macros then have to find where is the argument they are looking for.
	This is made easier by a modification in stack frame layout for
        these functions:the  stack frame-size is accessible on stack at
        location 4($30).

        Total overhead in PROLOGUE: 2 inns to put stacksize on stack
                                    2 sw.d to save floating registers.
        (Only when Varargs suspected)

        The only problem with ``thinking'', is that when functions are
        thought using varargs and dont do it, they get the above entry
        overhead.However the current method is quite precise, and is *safe*.


   See va-mips.h for more information on varargs

*/
extern int varargs_suspect;
extern int  this_varargs_suspect ;

#define VARARGS_SUSPECT(COND) varargs_suspect |= (COND)
#define VARARGS_NOTSUSPECT    varargs_suspect = 0
#define VARARGS_SUSPECTED    (varargs_suspect)

#define THIS_VARARGS_SUSPECT(COND) this_varargs_suspect |= (COND)
#define THIS_VARARGS_NOTSUSPECT    this_varargs_suspect = 0
#define THIS_VARARGS_SUSPECTED    (this_varargs_suspect)


#define FUNCTION_PROLOGUE(FILE, SIZE)					\
{ register int regno;							\
  register int mask = 0, fmask=0;					\
  static char dont_save_regs[] = CALL_USED_REGISTERS;			\
  register int push_loc = 0,tsize = SIZE+4;				\
  char *fp_str;								\
  extern char *reg_numchar[];						\
  extern int  current_function_total_framesize;				\
  this_varargs_suspect = VARARGS_SUSPECTED  ;				\
  fp_str = TARGET_NAME_REGS ? reg_names[STACK_POINTER_REGNUM]		\
    : reg_numchar[STACK_POINTER_REGNUM];				\
  for (regno = 0; regno < 32; regno++)					\
    if (  MUST_SAVE_REG_LOGUES						\
	|| (regs_ever_live[regno] && !dont_save_regs[regno]))		\
      {tsize += 4; mask |= 1 << regno;}					\
  for (regno = 32; regno < FIRST_PSEUDO_REGISTER; regno += 2)		\
    if (regs_ever_live[regno] && !dont_save_regs[regno])		\
      {tsize += 8; fmask |= 1 << (regno-32);}				\
  if (THIS_VARARGS_SUSPECTED) tsize += 16;				\
  fprintf (FILE," #PROLOGUE\n");					\
  regno = STACK_POINTER_REGNUM;						\
  tsize = AL_ADJUST_ALIGN (tsize);					\
									\
  if (!frame_pointer_needed)						\
    fprintf (FILE,"#define __0__gcc  %d\n",				\
	     (!( regs_ever_live[29] || regs_ever_live[30]		\
		|| fmask || mask					\
		|| (SIZE > 0)))						\
	     ? 0:tsize);						\
									\
  push_loc = 0; current_function_total_framesize = tsize;		\
  fprintf (FILE, " #\t.mask\t0x%x\n", mask);				\
  if (frame_pointer_needed || regs_ever_live[29] || regs_ever_live[30]	\
      || fmask || mask							\
      || (SIZE > 0))							\
    fprintf (FILE,"\tsubu\t%s,%d\t#temp=%5d,saveregs=%5d, sfo=%5d\n",	\
	     TARGET_NAME_REGS ? reg_names[29]				\
	     :reg_numchar[29],tsize,SIZE,tsize-SIZE,			\
	     STARTING_FRAME_OFFSET);					\
  else fprintf (FILE," #NO STACK PUSH:\tSP %sused, FP %sused, FP %sneeded\n",\
		regs_ever_live[29]? "":"un",				\
		regs_ever_live[30]? "":"un",				\
	       frame_pointer_needed ?"" : "not ");			\
  for  (regno = 31; regno >= 30; regno--)				\
    {									\
      if (MUST_SAVE_REG_LOGUES						\
	  || (regs_ever_live[regno] && !dont_save_regs[regno]))		\
	{								\
	  push_loc += 4;						\
	  fprintf (FILE,"\tsw\t%s,%d(%s)\n",				\
		   TARGET_NAME_REGS ? reg_names[regno]			\
		   : reg_numchar[regno],push_loc,fp_str);		\
	}								\
    }									\
  if (THIS_VARARGS_SUSPECTED)						\
    { int fregno;							\
      fprintf (FILE,"\taddi\t%s,$0,%d\t#Varargs suspicion\n",		\
	       TARGET_NAME_REGS ? reg_names[9]				\
	       : reg_numchar[9],tsize);					\
      fprintf (FILE,"\tsw\t%s,-4(%s)\t#Varargs suspicion\n",		\
	       TARGET_NAME_REGS ? reg_names[9]				\
	       : reg_numchar[9],					\
             TARGET_NAME_REGS ? reg_names[29]				\
	       : reg_numchar[29]);					\
      for (fregno = 44; fregno< 48; fregno += 2)			\
	{push_loc += 8;							\
	 fprintf (FILE,"\ts.d\t%s,%d(%s)\t#Varargs Suspicion\n",	\
		  ( (TARGET_NAME_REGS)					\
		   ?reg_names[fregno]: reg_numchar[fregno]),		\
		  push_loc,fp_str);}					\
    }									\
  for  (regno = 29; regno >= 0; regno--)				\
    {									\
      if (   MUST_SAVE_REG_LOGUES					\
	  || (regs_ever_live[regno] && !dont_save_regs[regno]))		\
	{								\
	  push_loc += 4;						\
	  fprintf (FILE, "\tsw\t%s,%d(%s)\n",				\
		   TARGET_NAME_REGS ? reg_names[regno]			\
		   : reg_numchar[regno],push_loc,fp_str);		\
	}								\
    }									\
  fprintf (FILE, " #\t.fmask\t0x%x\n", fmask);				\
  for  (regno = 32; regno < FIRST_PSEUDO_REGISTER; regno += 2)		\
    if (regs_ever_live[regno] && !dont_save_regs[regno])		\
      {push_loc += 8;							\
       fprintf (FILE,"\ts.d\t%s,%d(%s)\n",				\
		( (TARGET_NAME_REGS) ? reg_names[regno]			\
		 : reg_numchar[regno]),push_loc,fp_str);		\
     }									\
  if (frame_pointer_needed)						\
    fprintf (FILE,"\taddiu\t%s,%s,%d\t#Establish FramePTR\n",		\
	     (TARGET_NAME_REGS ? reg_names[FRAME_POINTER_REGNUM]  :	\
	      reg_numchar[FRAME_POINTER_REGNUM]),			\
	     (TARGET_NAME_REGS ? reg_names[29]  : reg_numchar[29]),	\
	     tsize);							\
  fprintf (FILE," #END PROLOGUE\n");					\
}

/* Output assembler code to FILE to increment profiler label # LABELNO
   for profiling a function entry.  */

#define FUNCTION_PROFILER(FILE, LABELNO)				\
   fprintf (FILE, "ERROR\t profiler LP%d,r0\n", (LABELNO));

/* EXIT_IGNORE_STACK should be nonzero if, when returning from a function,
   the stack pointer does not matter.  The value is tested only in
   functions that have frame pointers.
   No definition is equivalent to always zero.  */

extern int may_call_alloca;
extern int current_function_pretend_args_size;

#define EXIT_IGNORE_STACK 0


/* This declaration is needed due to traditional/ANSI
   incompatibilities which cannot be #ifdefed away
   because they occur inside of macros.  Sigh.  */


extern union tree_node *current_function_decl;
extern char *current_function_name;

/* Tell prologue and epilogue if Register containing return
   address should be saved / restored
*/

#define MUST_SAVE_REG_LOGUES (( frame_pointer_needed && (regno == 30))	\
                              ||( (regno == 31) && regs_ever_live[31])	\
                                 )


/* This macro generates the assembly code for function exit,
   on machines that need it.  If FUNCTION_EPILOGUE is not defined
   then individual return instructions are generated for each
   return statement.  Args are same as for FUNCTION_PROLOGUE.  */


#define FUNCTION_EPILOGUE(FILE, SIZE)					\
{ register int regno;							\
  register int mask = 0;						\
  register int fmask = 0;						\
  char *fp_str;								\
  char *sp_str;								\
  static char dont_save_regs[] = CALL_USED_REGISTERS;			\
  register int push_loc ;						\
  extern char *reg_numchar[];						\
  extern char *current_function_name;					\
  extern int  current_function_total_framesize;				\
  push_loc = 0;								\
  regno = STACK_POINTER_REGNUM;						\
  sp_str = TARGET_NAME_REGS ? reg_names[STACK_POINTER_REGNUM]		\
    : reg_numchar[STACK_POINTER_REGNUM];				\
  fp_str = TARGET_NAME_REGS ? reg_names[8]				\
    :reg_numchar[8];							\
  fprintf (FILE," #EPILOGUE\n");					\
  if (!frame_pointer_needed)						\
    fprintf (FILE,"#undef __0__gcc\n");					\
  else									\
    fprintf (FILE,"\taddu\t%s,$0,%s\t# sp not trusted  here \n",	\
	     fp_str,							\
	     TARGET_NAME_REGS ? reg_names[FRAME_POINTER_REGNUM]		\
	     :reg_numchar[FRAME_POINTER_REGNUM]				\
	     );								\
  for  (regno = 0; regno < 32; regno++)					\
    if  ( MUST_SAVE_REG_LOGUES						\
	 || (regs_ever_live[regno] && !dont_save_regs[regno]))		\
      mask |= 1 << regno;						\
  fprintf  (FILE, " #\t.mask\t0x%x\n", mask);				\
  for  (regno = 31; regno >= 0; regno--)				\
    { if  ( MUST_SAVE_REG_LOGUES					\
	   || (regs_ever_live[regno] && !dont_save_regs[regno]))	\
	{								\
	  push_loc += 4;						\
	  fprintf (FILE,"\tlw\t%s,%d(%s)\n",				\
		   TARGET_NAME_REGS ? reg_names[regno]			\
		   : reg_numchar[regno],				\
		   (frame_pointer_needed ?				\
		    push_loc - current_function_total_framesize:	\
		    push_loc),						\
		   (frame_pointer_needed ? fp_str :sp_str));		\
	}								\
      if ( THIS_VARARGS_SUSPECTED &&  (regno == 30)) push_loc += 16;	\
    }									\
  for  (regno = 32; regno < FIRST_PSEUDO_REGISTER; regno += 2)		\
    if  (regs_ever_live[regno] && !dont_save_regs[regno])		\
      fmask |= 1 <<  (regno-32);					\
  fprintf  (FILE, " #\t.fmask\t0x%x\n", fmask);				\
    for  (regno = 32; regno < FIRST_PSEUDO_REGISTER; regno += 2)	\
    {									\
      if  (regs_ever_live[regno] && !dont_save_regs[regno])		\
	{								\
	  push_loc += 8;						\
	  fprintf (FILE,"\tl.d\t%s,%d(%s)\n",				\
		   ( ( TARGET_NAME_REGS) ? reg_names[regno]		\
		    : reg_numchar[regno]),				\
		   (frame_pointer_needed ?				\
		    push_loc - current_function_total_framesize		\
		    : push_loc),					\
		   (frame_pointer_needed ? fp_str :sp_str));		\
	}								\
    }									\
  if (frame_pointer_needed)						\
    fprintf (FILE,"\taddu\t%s,$0,%s\t# sp not trusted  here \n",	\
	     TARGET_NAME_REGS ? reg_names[STACK_POINTER_REGNUM]		\
	     :reg_numchar[STACK_POINTER_REGNUM],			\
	     TARGET_NAME_REGS ? reg_names[8]				\
	     :reg_numchar[8]						\
	     );								\
  else									\
    if (regs_ever_live[29]|| regs_ever_live[30]				\
	|| fmask || mask						\
	||  (SIZE > 0))			\
      fprintf (FILE,"\taddu\t%s,%d\t\n",TARGET_NAME_REGS ? reg_names[29]\
	       :reg_numchar[29],current_function_total_framesize);	\
  fprintf (FILE,"\tj\t$31\n");						\
  fprintf (FILE," #END EPILOGUE\n");					\
  fprintf (FILE," \t.end\t%s\n",current_function_name);			\
  THIS_VARARGS_NOTSUSPECT; VARARGS_NOTSUSPECT;}

/* If the memory Address ADDR is relative to the frame pointer,
   correct it to be relative to the stack pointer. This is for
   when we don't use a frame pointer.
   ADDR should be a variable name.  */

#define FIX_FRAME_POINTER_ADDRESS(ADDR,DEPTH)				\
{ rtx newaddr;								\
    int frame_offset = -1;						\
    /* fprintf(stderr,"FIX_FRAME depth=%d\n",DEPTH); */			\
  if(ADDR == frame_pointer_rtx)						\
    frame_offset = 0;							\
  else									\
      if (GET_CODE(ADDR) == PLUS)					\
          if(XEXP(ADDR,0) == frame_pointer_rtx)				\
             if(GET_CODE(XEXP(ADDR,1)) == CONST_INT)			\
	       frame_offset = INTVAL(XEXP(ADDR,1));			\
             else abort_with_insn(ADDR,"Unable to FIX");		\
          else if (XEXP(ADDR,1) == frame_pointer_rtx)			\
             if(GET_CODE(XEXP(ADDR,0)) == CONST_INT)			\
	       frame_offset = INTVAL(XEXP(ADDR,0));			\
             else abort_with_insn(ADDR,"Unable to FIX");		\
	  else;								\
   if (frame_offset >= 0)						\
    { newaddr=gen_rtx(PLUS,Pmode,stack_pointer_rtx,			\
                      gen_rtx(PLUS,Pmode,				\
                          gen_rtx(CONST_INT,VOIDmode,frame_offset+(DEPTH)),\
			      gen_rtx(SYMBOL_REF,SImode,"__0__gcc")));	\
      ADDR = newaddr;							\
    }									\
  }



/* Addressing modes, and classification of registers for them.  */

/* #define HAVE_POST_INCREMENT */
/* #define HAVE_POST_DECREMENT */

/* #define HAVE_PRE_DECREMENT */
/* #define HAVE_PRE_INCREMENT */

/* These assume that REGNO is a hard or pseudo reg number.
   They give nonzero only if REGNO is a hard reg of the suitable class
   or a pseudo reg currently allocated to a suitable hard reg.
   These definitions are NOT overridden anywhere.  */

#define REGNO_OK_FOR_INDEX_P(regno)					\
((regno) < FIRST_PSEUDO_REGISTER || reg_renumber[regno] >= 0)
#define REGNO_OK_FOR_BASE_P(regno)					\
((regno) < FIRST_PSEUDO_REGISTER || reg_renumber[regno] >= 0)
#define REGNO_OK_FOR_FP_P(REGNO)					\
(((REGNO) ^ 0x20) < 32 || (unsigned) (reg_renumber[REGNO] ^ 0x20) < 32)

/* The macros REG_OK_FOR..._P assume that the arg is a REG rtx
   and check its validity for a certain class.
   We have two alternate definitions for each of them.
   The usual definition accepts all pseudo regs; the other rejects them all.
   The symbol REG_OK_STRICT causes the latter definition to be used.

   Most source files want to accept pseudo regs in the hope that
   they will get allocated to the class that the insn wants them to be in.
   Some source files that are used after register allocation
   need to be strict.  */

#ifndef REG_OK_STRICT

/* Nonzero if X is a hard reg that can be used as an index or if
   it is a pseudo reg.  */
#define REG_OK_FOR_INDEX_P(X) 1
/* Nonzero if X is a hard reg that can be used as a base reg
   of if it is a pseudo reg.  */
#define REG_OK_FOR_BASE_P(X) 1

#else

/* Nonzero if X is a hard reg that can be used as an index.  */
#define REG_OK_FOR_INDEX_P(X) REGNO_OK_FOR_INDEX_P (REGNO (X))
/* Nonzero if X is a hard reg that can be used as a base reg.  */
#define REG_OK_FOR_BASE_P(X) REGNO_OK_FOR_BASE_P (REGNO (X))

#endif

#define REG_OK_FOR_CLASS_P(X, C) 0

#define REGNO_OK_FOR_CLASS_P(X, C)  0

#define ADDRESS_REG_P(X)						\
  (GET_CODE (X) == REG )

/* 1 if X is an fp register.  */

#define FP_REG_P(X) (REG_P (X) && REGNO_OK_FOR_FP_P (REGNO (X)))

/* Maximum number of registers that can appear in a valid memory address.  */

#define MAX_REGS_PER_ADDRESS 1

/* GO_IF_LEGITIMATE_ADDRESS recognizes an RTL expression
   that is a valid memory address for an instruction.
   The MODE argument is the machine mode for the MEM expression
   that wants to use this address.

   The other macros defined here are used only in GO_IF_LEGITIMATE_ADDRESS,
   except for CONSTANT_ADDRESS_P which is actually machine-independent.  */

/* 1 if X is an address that we could indirect through.  */
#define INDIRECTABLE_ADDRESS_P(X)					\
  (CONSTANT_ADDRESS_P (X)						\
   || (GET_CODE (X) == REG && REG_OK_FOR_BASE_P (X))			\
   || (GET_CODE (X) == PLUS						\
       && GET_CODE (XEXP (X, 0)) == REG					\
       && REG_OK_FOR_BASE_P (XEXP (X, 0))				\
       && CONSTANT_ADDRESS_P (XEXP (X, 1))))



/* 1 if X is an address which is (+ (reg) (+ (const_int) (symbol_ref)  )) */
#define FIXED_FRAME_PTR_REL_P(X)					\
  (    (GET_CODE(X) == PLUS)						\
   &&  (GET_CODE(XEXP((X),0)) == REG)					\
   &&  (GET_CODE(XEXP((X),1)) == PLUS)					\
   &&  (GET_CODE(XEXP(XEXP((X),1),0)) == CONST_INT)			\
   &&   (GET_CODE(XEXP(XEXP((X),1),1)) == SYMBOL_REF))

/* Go to ADDR if X is a valid address not using indexing.
   (This much is the easy part.)  */
#define GO_IF_LEGITIMATE_ADDRESS(MODE, X, ADDR)				\
{ register rtx xfoob = (X);						\
  if (GET_CODE (xfoob) == REG) goto ADDR;				\
  if (INDIRECTABLE_ADDRESS_P (xfoob))   goto ADDR;			\
  if (FIXED_FRAME_PTR_REL_P (xfoob))    goto ADDR;                      \
  }




#define CONSTANT_ADDRESS_P(X)						\
  (GET_CODE (X) == LABEL_REF || GET_CODE (X) == SYMBOL_REF		\
   || GET_CODE (X) == CONST_INT						\
   || GET_CODE (X) == CONST)

/* Nonzero if the constant value X is a legitimate general operand.
   It is given that X satisfies CONSTANT_P or is a CONST_DOUBLE.

   Anything but a CONST_DOUBLE can be made to work.  */

#define LEGITIMATE_CONSTANT_P(X)					\
 (GET_CODE (X) != CONST_DOUBLE)

/* Try machine-dependent ways of modifying an illegitimate address
   to be legitimate.  If we find one, return the new, valid address.
   This macro is used in only one place: `memory_address' in explow.c.

   OLDX is the address as it was before break_out_memory_refs was called.
   In some cases it is useful to look at this to decide what needs to be done.

   MODE and WIN are passed so that this macro can use
   GO_IF_LEGITIMATE_ADDRESS.

   It is always safe for this macro to do nothing.  It exists to recognize
   opportunities to optimize the output.

   For the MIPS (so far ..), nothing needs to be done.

   ACHTUNG this is actually used by the FLOW analysis to get rid
   of statements....

*/

#define LEGITIMIZE_ADDRESS(X,OLDX,MODE,WIN)  {}

/* Go to LABEL if ADDR (a legitimate address expression)
   has an effect that depends on the machine mode it is used for.
*/

				/* See if this is of any use here */

#define GO_IF_MODE_DEPENDENT_ADDRESS(ADDR,LABEL)			\
{ }


/* Specify the machine mode that this machine uses
   for the index in the tablejump instruction.  */
#define CASE_VECTOR_MODE SImode

/* Define this if the tablejump instruction expects the table
   to contain offsets from the address of the table.
   Do not define this if the table should contain absolute addresses.  */
/* #define CASE_VECTOR_PC_RELATIVE */

/* Specify the tree operation to be used to convert reals to integers.  */
#define IMPLICIT_FIX_EXPR FIX_ROUND_EXPR

/* This is the kind of divide that is easiest to do in the general case.  */
#define EASY_DIV_EXPR TRUNC_DIV_EXPR

/* Define this as 1 if `char' should by default be signed; else as 0.  */
#define DEFAULT_SIGNED_CHAR 1

/* Max number of bytes we can move from memory to memory
   in one reasonably fast instruction.  */
#define MOVE_MAX 4

/* Nonzero if access to memory by bytes is slow and undesirable.  */
#define SLOW_BYTE_ACCESS 0

/* On Sun 4, this limit is 2048.  We use 1500 to be safe,
   since the length can run past this up to a continuation point.  */
#define DBX_CONTIN_LENGTH 1500

/* We assume that the store-condition-codes instructions store 0 for false
   and some other value for true.  This is the value stored for true.  */

#define STORE_FLAG_VALUE 1

/* Define this if zero-extension is slow (more than one real instruction).  */
#define SLOW_ZERO_EXTEND

/* Define if shifts truncate the shift count
   which implies one can omit a sign-extension or zero-extension
   of a shift count.

   Only 5 bits are used in SLLV and SRLV
*/
#define SHIFT_COUNT_TRUNCATED


/* Value is 1 if truncating an integer of INPREC bits to OUTPREC bits
   is done just by pretending it is already truncated.  */
#define TRULY_NOOP_TRUNCATION(OUTPREC, INPREC) 1

/* Specify the machine mode that pointers have.
   After generation of rtl, the compiler makes no further distinction
   between pointers and any other objects of this machine mode.  */
#define Pmode SImode

/* A function address in a call instruction
   is a word address (for indexing purposes)
   so give the MEM rtx a words's mode.  */

#define FUNCTION_MODE SImode

/* Compute the cost of computing a constant rtl expression RTX
   whose rtx-code is CODE.  The body of this macro is a portion
   of a switch statement.  If the code is computed here,
   return it with a return statement.  Otherwise, break from the switch.  */

#define CONST_COSTS(RTX,CODE)						\
  case CONST_INT:							\
    /* Constant zero is super cheap due to register 0.  */		\
    if (RTX == const0_rtx) return 0;					\
    if ((INTVAL (RTX) < 0x7fff) && (- INTVAL(RTX) < 0x7fff)) return 1;	\
  case CONST:								\
  case LABEL_REF:							\
  case SYMBOL_REF:							\
    return 3;								\
  case CONST_DOUBLE:							\
    return 5;

/* Tell final.c how to eliminate redundant test instructions.  */

/* Here we define machine-dependent flags and fields in cc_status
   (see `conditions.h').  No extra ones are needed for the vax.  */
/* Tell final.c how to eliminate redundant test instructions.  */

/* Tell final.c how to eliminate redundant test instructions.  */

/* Here we define machine-dependent flags and fields in cc_status
   (see `conditions.h').  No extra ones are needed for the vax.  */

/* Store in cc_status the expressions
   that the condition codes will describe
   after execution of an instruction whose pattern is EXP.
   Do not alter them if the instruction would not alter the cc's.  */

#define NOTICE_UPDATE_CC(EXP, INSN)					\
  CC_STATUS_INIT;


/* Here we define machine-dependent flags and fields in cc_status
   (see `conditions.h').   */


/* Control the assembler format that we output.  */

/* Output at beginning of assembler file.  */

#define ASM_FILE_START(FILE)						\
{									\
  if (TARGET_NAME_REGS)							\
    fprintf (FILE, "#include <regdef.h>\n\t.verstamp\t%s\n", TARGET_VERSNUM);\
  else fprintf (FILE, " #\t.verstamp\t%s\n", TARGET_VERSNUM);		\
/* print_options(FILE);  */						\
  if (TARGET_GP_OPT)							\
    fprintf (FILE, "#ifdef %sRESCAN_GCC\n", "__x_");			\
}

/* Output to assembler file text saying following lines
   may contain character constants, extra white space, comments, etc.  */

#define ASM_APP_ON " #APP\n"

/* Output to assembler file text saying following lines
   no longer contain unusual constructs.  */

#define ASM_APP_OFF " #NO_APP\n"

/* Output before read-only data.  */

#define TEXT_SECTION_ASM_OP ".text"

/* Output before writable data.  */

#define DATA_SECTION_ASM_OP ".data"

#define  ASM_OUTPUT_MIPS_SECTIONS
#define  OUTPUT_MIPS_SECTION_THRESHOLD  ((mips_section_threshold >= 0 )?\
					mips_section_threshold : mips_section_get())

/* Output before writable  short data.  */

#define SDATA_SECTION_ASM_OP ".sdata"

/* How to refer to registers in assembler output.
   This sequence is indexed by compiler's hard-register-number (see above).  */

#define REGISTER_NAMES							\
{"$0", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0",			\
 "t1", "t2", "t3", "t4", "t5", "t6", "t7","s0",				\
 "s1","s2","s3","s4","s5","s6","s7","t8","t9",				\
 "k0","k1","gp","sp","fp","ra",						\
 "$f0","$f1","$f2","$f3","$f4","$f5","$f6","$f7","$f8","$f9",		\
"$f10","$f11","$f12","$f13","$f14","$f15","$f16","$f17","$f18","$f19",	\
"$f20","$f21","$f22","$f23","$f24","$f25","$f26","$f27","$f28","$f29",	\
"$f30","$f31"								\
}
#define REGISTER_NUMCHAR						\
{									\
"$0","$1","$2","$3","$4","$5","$6","$7","$8","$9",			\
"$10","$11","$12","$13","$14","$15","$16","$17","$18","$19",		\
"$20","$21","$22","$23","$24","$25","$26","$27","$28","$29",		\
"$30","$31",								\
"$f0","$f1","$f2","$f3","$f4","$f5","$f6","$f7","$f8","$f9",		\
"$f10","$f11","$f12","$f13","$f14","$f15","$f16","$f17","$f18","$f19",	\
"$f20","$f21","$f22","$f23","$f24","$f25","$f26","$f27","$f28","$f29",	\
"$f30","$f31"								\
}


/* How to renumber registers for dbx and gdb.
   MIPS needs no change in the numeration.  */

#define DBX_REGISTER_NUMBER(REGNO) (REGNO)

/* Define results of standard character escape sequences.  */
#define TARGET_BELL 007
#define TARGET_BS 010
#define TARGET_TAB 011
#define TARGET_NEWLINE 012
#define TARGET_VT 013
#define TARGET_FF 014
#define TARGET_CR 015

				/*  LIST OF PRINT OPERAND CODES


				** 'x'  X is CONST_INT, prints 16 bits in
				**      Hexadecimal format = "0x%4x",
				** 'd'  output integer constant in decimal,
				** 'u'  Prints an 'u' if flag -mnofixed-ovfl
				**      has been set, thus selecting addu
				**      instruction instead of add.
				*/


/* Print an instruction operand X on file FILE.
   CODE is the code from the %-spec that requested printing this operand;
   if `%z3' was used to print operand 3, then CODE is 'z'.
   CODE is used as follows:

				    LIST OF PRINT OPERAND CODES


				   'x'  X is CONST_INT, prints 16 bits in
				**      Hexadecimal format = "0x%4x",
				** 'd'  output integer constant in decimal,
				** ':'  Prints an 'u' if flag -mnofixed-ovfl
				**      has been set, thus selecting addu
				**      instruction instead of add.
				*/

#define PRINT_OPERAND_PUNCT_VALID_P(CODE)				\
  ((CODE) == ':')

#define PRINT_OPERAND(FILE, X, CODE)					\
{ if ((CODE) == ':')							\
    {if (TARGET_NOFIXED_OVFL)fprintf(FILE,"u");}			\
  else if (GET_CODE (X) == REG)						\
    { extern char *reg_numchar[];					\
      fprintf (FILE, "%s", TARGET_NAME_REGS ?reg_names[REGNO (X)]	\
	       :reg_numchar[REGNO (X) ]);				\
    }									\
  else									\
    {									\
      if (GET_CODE (X) == MEM)						\
	output_address (XEXP (X, 0));					\
      else if (GET_CODE (X) == CONST_DOUBLE)				\
	{ union { double d; int i[2]; } u;				\
	  union { float f; int i; } u1;					\
	  u.i[0] = CONST_DOUBLE_LOW (X);				\
	  u.i[1] = CONST_DOUBLE_HIGH (X);				\
	  u1.f = u.d;							\
	  if (GET_MODE (X) == SFmode)					\
	    u.d = u1.f;							\
	  fprintf (FILE, "%.20e", u.d); }				\
      else								\
	{ if ((CODE == 'x') && (GET_CODE(X) == CONST_INT))		\
	    fprintf(FILE,"0x%x",0xffff & (INTVAL(X)));			\
	  else { if ((CODE == 'd') && (GET_CODE(X) == CONST_INT))	\
		   fprintf(FILE,"%d",(INTVAL(X)));			\
	         else							\
		   {							\
		      if ((CODE) == 'd') abort();			\
		      else output_addr_const (FILE, X);}		\
		}}}}

/* Print a memory operand whose address is X, on file FILE.  */

#define PRINT_OPERAND_ADDRESS(FILE, ADDR)				\
{ register rtx reg1, reg2, breg, ireg;					\
  register rtx addr = ADDR;						\
  rtx offset;								\
  extern char *reg_numchar[];						\
/*	my_print_rtx(addr);*/						\
 retry:									\
  switch (GET_CODE (addr))						\
    {									\
    case REG:								\
      fprintf (FILE, "0(%s)", TARGET_NAME_REGS ? reg_names [REGNO (addr)]\
	       : reg_numchar[REGNO(addr)]);				\
      break;								\
    case MEM:								\
    case PRE_DEC:							\
    case POST_INC:							\
      abort();								\
      break;								\
    case PLUS:								\
      if(   (GET_CODE (XEXP(addr,0)) == REG)				\
         && (GET_CODE (XEXP(addr,1)) == PLUS)				\
         && (GET_CODE (XEXP(XEXP(addr,1),1)) == SYMBOL_REF)		\
         && (GET_CODE (XEXP(XEXP(addr,1),0)) == CONST_INT))		\
	{output_address(XEXP(XEXP(addr,1),0));				\
         fprintf(FILE,"+");						\
         output_address(XEXP(XEXP(addr,1),1));				\
         breg = XEXP(addr,0);						\
	 fprintf(FILE,"(%s)", TARGET_NAME_REGS ?			\
		   reg_names[REGNO (breg)]: reg_numchar[REGNO(breg)]);	\
	 break;								\
        }								\
									\
      reg1 = 0;	reg2 = 0;						\
      ireg = 0;	breg = 0;						\
      offset = 0;							\
        /*fprintf(stderr,"PRINT_OPERAND_ADDRESS"); */			\
      if (CONSTANT_ADDRESS_P (XEXP (addr, 0))				\
	  || GET_CODE (XEXP (addr, 0)) == MEM)				\
	{								\
	  offset = XEXP (addr, 0);					\
	  addr = XEXP (addr, 1);					\
	}								\
      else if (CONSTANT_ADDRESS_P (XEXP (addr, 1))			\
	       || GET_CODE (XEXP (addr, 1)) == MEM)			\
	{								\
	  offset = XEXP (addr, 1);					\
	  addr = XEXP (addr, 0);					\
	}								\
      if (GET_CODE (addr) != PLUS) ;					\
      else if (GET_CODE (XEXP (addr, 0)) == MULT)			\
	{								\
	  reg1 = XEXP (addr, 0);					\
	  addr = XEXP (addr, 1);					\
	}								\
      else if (GET_CODE (XEXP (addr, 1)) == MULT)			\
	{								\
	  reg1 = XEXP (addr, 1);					\
	  addr = XEXP (addr, 0);					\
	}								\
      else if (GET_CODE (XEXP (addr, 0)) == REG)			\
	{								\
	  reg1 = XEXP (addr, 0);					\
	  addr = XEXP (addr, 1);					\
	}								\
      else if (GET_CODE (XEXP (addr, 1)) == REG)			\
	{								\
	  reg1 = XEXP (addr, 1);					\
	  addr = XEXP (addr, 0);					\
	}								\
      if (GET_CODE (addr) == REG || GET_CODE (addr) == MULT)		\
	{ if (reg1 == 0) reg1 = addr; else reg2 = addr; addr = 0; }	\
      if (offset != 0) { if (addr != 0) abort (); addr = offset; }	\
      if (reg1 != 0 && GET_CODE (reg1) == MULT)				\
	{ breg = reg2; ireg = reg1; }					\
      else if (reg2 != 0 && GET_CODE (reg2) == MULT)			\
	{ breg = reg1; ireg = reg2; }					\
      else if (reg2 != 0 || GET_CODE (addr) == MEM)			\
	{ breg = reg2; ireg = reg1; }					\
      else								\
	{ breg = reg1; ireg = reg2; }					\
      if (addr != 0)							\
	output_address (offset);					\
      if (breg != 0)							\
	{ if (GET_CODE (breg) != REG) abort ();				\
	  fprintf (FILE, "(%s)", TARGET_NAME_REGS ?			\
		   reg_names[REGNO (breg)]: reg_numchar[REGNO(breg)]); }\
      if (ireg != 0)							\
	{ if (GET_CODE (ireg) == MULT) ireg = XEXP (ireg, 0);		\
	  if (GET_CODE (ireg) != REG) abort ();				\
	  fprintf (FILE, "[%s]",  TARGET_NAME_REGS ?			\
		   reg_names[REGNO (ireg)]: reg_numchar[REGNO(ireg)]); }\
      break;								\
    default:								\
      output_addr_const (FILE, addr);					\
    }}


/* This is how to output a note to DBX telling it the line number
   to which the following sequence of instructions corresponds.

   This is needed for SunOS 4.0, and should not hurt for 3.2
   versions either.  */
#define ASM_OUTPUT_SOURCE_LINE(file, line)				\
  { static int sym_lineno = 1;						\
    fprintf (file, " #.stabn 68,0,%d,LM%d\nLM%d:\n",			\
	     line, sym_lineno, sym_lineno);				\
    sym_lineno += 1; }

/* This is how to output the definition of a user-level label named NAME,
   such as the label on a static function or variable NAME.  */

#define ASM_OUTPUT_LABEL(FILE,NAME)					\
  do { assemble_name (FILE, NAME); fputs (":\n", FILE); } while (0)

/* This is how to output a command to make the user-level label named NAME
   defined for reference from other files.  */

#define ASM_GLOBALIZE_LABEL(FILE,NAME)					\
  do { fputs ("\t.globl ", FILE); assemble_name (FILE, NAME);		\
       fputs ("\n", FILE);						\
       if(TARGET_GP_OPT) {fputs ("#define _gccx__",FILE);		\
       assemble_name(FILE,NAME);					\
       fputs ("\n", FILE);						\
       }								\
     } while (0)

#define ASM_DECLARE_FUNCTION_NAME(FILE,NAME,DECL)			\
  fprintf(FILE,"\t.ent\t%s\n",NAME);					\
  current_function_name = NAME;						\
  ASM_OUTPUT_LABEL(FILE,NAME);

/* This is how to output a reference to a user-level label named NAME.
   `assemble_name' uses this.  */

#define ASM_OUTPUT_LABELREF(FILE,NAME)					\
  fprintf (FILE, "%s", NAME)

/* This is how to output an internal numbered label where
   PREFIX is the class of label and NUM is the number within the class.  */

#define ASM_OUTPUT_INTERNAL_LABEL(FILE,PREFIX,NUM)			\
  fprintf (FILE, "%s%d:\n", PREFIX, NUM)

/* This is how to store into the string LABEL
   the symbol_ref name of an internal numbered label where
   PREFIX is the class of label and NUM is the number within the class.
   This is suitable for output with `assemble_name'.  */

#define ASM_GENERATE_INTERNAL_LABEL(LABEL,PREFIX,NUM)			\
  sprintf (LABEL, "*%s%d", PREFIX, NUM)

/* This is how to output an assembler line defining a `double' constant.  */

#define ASM_OUTPUT_DOUBLE(FILE,VALUE)					\
  fprintf (FILE, "\t.double %.20e\n", (VALUE))

/* This is how to output an assembler line defining a `float' constant.  */

#define ASM_OUTPUT_FLOAT(FILE,VALUE)					\
  fprintf (FILE, "\t.float %.12e\n", (VALUE))

/* This is how to output an assembler line defining an `int' constant.  */

#define ASM_OUTPUT_INT(FILE,VALUE)					\
( fprintf (FILE, "\t.word "),						\
  output_addr_const (FILE, (VALUE)),					\
  fprintf (FILE, "\n"))

/* Likewise for `char' and `short' constants.  */

#define ASM_OUTPUT_SHORT(FILE,VALUE)					\
( fprintf (FILE, "\t.half "),						\
  output_addr_const (FILE, (VALUE)),					\
  fprintf (FILE, "\n"))

#define ASM_OUTPUT_CHAR(FILE,VALUE)					\
( fprintf (FILE, "\t.byte "),						\
  output_addr_const (FILE, (VALUE)),					\
  fprintf (FILE, "\n"))

/* This is how to output an assembler line for a numeric constant byte.  */

#define ASM_OUTPUT_BYTE(FILE,VALUE)					\
  fprintf (FILE, "\t.byte 0x%x\n", (VALUE))

/* This is how to output an element of a case-vector that is absolute.  */

#define ASM_OUTPUT_ADDR_VEC_ELT(FILE, VALUE)				\
  fprintf (FILE, "\t.word L%d\n", VALUE)

/* This is how to output an element of a case-vector that is relative.
   (We  do not use such vectors,
   but we must define this macro anyway.)  */

#define ASM_OUTPUT_ADDR_DIFF_ELT(FILE, VALUE, REL)			\
  fprintf (FILE, "\t.word L%d-L%d\n", VALUE, REL)

/* This is how to output an assembler line
   that says to advance the location counter
   to a multiple of 2**LOG bytes.  */

#define ASM_OUTPUT_ALIGN(FILE,LOG)					\
    fprintf (FILE, "\t.align %d\n", (LOG))

#define ASM_OUTPUT_SKIP(FILE,SIZE)					\
  fprintf (FILE, "\t.space %d\n", (SIZE))

/* The support of .comm and .extern  below permits to take advantage
   of the SDATA/SBSS sections supported by the MIPS ASSEMBLER and LOADER
   However some problems have to be solved
        a/  externs should be included ONCE
	b/  the same external cannot appear both on an extern and .comm stmt
	    in the same assembly
	c/  for the whole scheme to bring some benefit, .comm should appear
	    in front of the source asm -- whereas GCC put them at the end
*/


				/* ALL THESE PROBLEMS ARE PRESENTLY SOLVED   */
				/* USING CONDITIONAL ASSEMBLY + FILE RESCAN  */

#define EXTRA_SECTIONS in_sdata

/* Define the additional functions to select our additional sections.  */

       /* on the MIPS it is not a good idea to put constants  in the
	  text section, since this defeats the sdata/data mechanism. This
	  is especially true when -O2 is used. In this case an effort is
	  made to address with faster (gp) register relative addressing,
	  which can only get at sdata and sbss items (there is no stext !!)
       */
#define EXTRA_SECTION_FUNCTIONS						\
void									\
sdata_section ()							\
{									\
  if (in_section != in_sdata)						\
    {									\
      fprintf (asm_out_file, "%s\n", SDATA_SECTION_ASM_OP);		\
      in_section = in_sdata;						\
    }								\
}

/* Given a decl node or constant node, choose the section to output it in
   and select that section.  */

       /* following takes  care of constants  emitted from
	  the hash table entries (see above comment)
       */
#define SELECT_SECTION_MODE(MODE,RTX)					\
{									\
  extern int mips_section_threshold;					\
  if (( GET_MODE_SIZE(MODE)/ BITS_PER_UNIT)				\
	      <= OUTPUT_MIPS_SECTION_THRESHOLD)				\
	    sdata_section();						\
	  else								\
	    data_section ();						\
}									\

#define SELECT_SECTION(DECL)						\
{									\
  extern int mips_section_threshold;					\
	  if (int_size_in_bytes (TREE_TYPE (DECL))			\
	      <= OUTPUT_MIPS_SECTION_THRESHOLD)				\
	    sdata_section ();						\
	  else								\
	    data_section ();						\
}

/* This says how to output an assembler line
   to define a global common symbol.  */

#define ASM_OUTPUT_COMMON(FILE, NAME, SIZE, ROUNDED)			\
( ((TARGET_GP_OPT)?							\
   fprintf((FILE),"\n#else"),0 :0),					\
 fputs ("\n\t.comm ", (FILE)),						\
 assemble_name ((FILE), (NAME)),					\
 fprintf ((FILE), ",%d\n", (ROUNDED)),					\
 (TARGET_GP_OPT ? (fputs("\n#define _gccx__",(FILE)),			\
		   assemble_name((FILE),NAME),0):0),			\
 ((TARGET_GP_OPT)?							\
  fprintf((FILE),"\n#endif\n#ifdef %sRESCAN_GCC","__x_"),0 :0)		\
)


/* This says how to output an external                                      */
/* It would be possible not to output anything and let undefined            */
/* symbol become external. However the assembler uses length  information on*/
/* externals to allocate in data/sdata bss/sbss, thereby saving exec time   */

#define ASM_OUTPUT_EXTERNAL(FILE,DECL,NAME)				\
        mips_output_external(FILE,DECL,NAME)


/* This says how to output an assembler line
   to define a local common symbol.  */

#define ASM_OUTPUT_LOCAL(FILE, NAME, SIZE, ROUNDED)			\
( fputs ("\n\t.lcomm\t", (FILE)),					\
  assemble_name ((FILE), (NAME)),					\
  fprintf ((FILE), ",%d\n", (ROUNDED)))

/* This says what to print at the end of the assembly file */
#define ASM_FILE_END(FILE)						\
       mips_asm_file_end(FILE)

/* Store in OUTPUT a string (made with alloca) containing
   an assembler-name for a local static variable named NAME.
   LABELNO is an integer which is different for each call.  */

#define ASM_FORMAT_PRIVATE_NAME(OUTPUT, NAME, LABELNO)			\
( (OUTPUT) = (char *) alloca (strlen ((NAME)) + 10),			\
  sprintf ((OUTPUT), "%s.%d", (NAME), (LABELNO)))

#define ASM_OUTPUT_REG_POP(FILE,REGNO)					\
  (fprintf (FILE,"ERROR: ASM_OUTPUT_REG_POP\n"))
#define ASM_OUTPUT_REG_PUSH(FILE,REGNO)					\
  (fprintf (FILE,"ERROR: ASM_OUTPUT_REG_PUSH\n"))

				/*  The following macro is taken from the    */
				/*  C-text of varasm.c. It has been modified */
				/*  to handle the VARARG_SUSPECTED hack      */
#define  ASM_OUTPUT_ASCII(FILE, P , SIZE)				\
{  int i;								\
	  fprintf ((FILE), "\t.ascii \"");				\
          VARARGS_SUSPECT( 0 == strncmp((P),"__%%VARARGS",11));		\
	  for (i = 0; i < (SIZE); i++)					\
	    {								\
	      register int c = (P)[i];					\
	      if (i != 0 && (i / 200) * 200 == i)			\
		fprintf ((FILE), "\"\n\t.ascii \"");			\
	      if (c == '\"' || c == '\\')				\
		putc ('\\', (FILE));					\
	      if (c >= ' ' && c < 0177)					\
		putc (c, (FILE));					\
	      else							\
		{							\
		  fprintf ((FILE), "\\%o", c);				\
		  /* After an octal-escape, if a digit follows,		\
		     terminate one string constant and start another.	\
		     The Vax assembler fails to stop reading the escape	\
		     after three digits, so this is the only way we	\
		     can get it to parse the data properly.  */		\
		  if (i < (SIZE) - 1 && (P)[i + 1] >= '0' && (P)[i + 1] <= '9')\
		    fprintf ((FILE), "\"\n\t.ascii \"");		\
		}							\
	    }								\
	  fprintf ((FILE), "\"\n");					\
 }



/* Define the parentheses used to group arithmetic operations
   in assembler code.  */

#define ASM_OPEN_PAREN "("
#define ASM_CLOSE_PAREN ")"

/* Specify what to precede various sizes of constant with
   in the output file.  */

#define ASM_INT_OP ".word "
#define ASM_SHORT_OP ".half "
#define ASM_CHAR_OP ".byte "


#define DEBUG_LOG_INSN(X)  {						\
            extern rtx al_log_insn_debug;				\
            al_log_insn_debug=(X); }
