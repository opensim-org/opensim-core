/*******************************************************************************

   SIMMKEYS.C

   Author: Kenny Smith

   Date: 9-Nov-98

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.
   This file is copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#include <string.h>
#include <ctype.h>

#include "universal.h"
#include "functions.h"

typedef struct {
   const char* keyname;
   int         keyid;
} KeyDef;

static KeyDef simmkeys[] = {
{"null_key", 0},
{"key_released", 0},
{"key_pressed", 1},
{"backspace_key", 8},
{"tab_key", 9},
{"return_key", 13},
{"enter_key", 13},
{"escape_key", 27},
{"space_key", 32},
{"bang_key", 33},
{"double_quote_key", 34},
{"hash_key", 35},
{"pound_sign_key", 35},
{"dollar_key", 36},
{"percent_key", 37},
{"ampersand_key", 38},
{"single_quote_key", 39},
{"open_paren_key", 40},
{"left_paren_key", 40},
{"close_paren_key", 41},
{"right_paren_key", 41},
{"asterisk_key", 42},
{"plus_key", 43},
{"comma_key", 44},
{"dash_key", 45},
{"minus_key", 45},
{"period_key", 46},
{"slash_key", 47},
{"zero_key", 48},
{"one_key", 49},
{"two_key", 50},
{"three_key", 51},
{"four_key", 52},
{"five_key", 53},
{"six_key", 54},
{"seven_key", 55},
{"eight_key", 56},
{"nine_key", 57},
{"colon_key", 58},
{"semicolon_key", 59},
{"less_than_key", 60},
{"equals_key", 61},
{"greater_than_key", 62},
{"question_mark_key", 63},
{"at_sign_key", 64},
{"A_key", 65},
{"B_key", 66},
{"C_key", 67},
{"D_key", 68},
{"E_key", 69},
{"F_key", 70},
{"G_key", 71},
{"H_key", 72},
{"I_key", 73},
{"J_key", 74},
{"K_key", 75},
{"L_key", 76},
{"M_key", 77},
{"N_key", 78},
{"O_key", 79},
{"P_key", 80},
{"Q_key", 81},
{"R_key", 82},
{"S_key", 83},
{"T_key", 84},
{"U_key", 85},
{"V_key", 86},
{"W_key", 87},
{"X_key", 88},
{"Y_key", 89},
{"Z_key", 90},
{"left_bracket_key", 91},
{"open_bracket_key", 91},
{"backslash_key", 92},
{"right_bracket_key", 93},
{"close_bracket_key", 93},
{"carat_key", 94},
{"underscore_key", 95},
{"back_quote_key", 96},
{"virgule_key", 96},
{"a_key", 97},
{"b_key", 98},
{"c_key", 99},
{"d_key", 100},
{"e_key", 101},
{"f_key", 102},
{"g_key", 103},
{"h_key", 104},
{"i_key", 105},
{"j_key", 106},
{"k_key", 107},
{"l_key", 108},
{"m_key", 109},
{"n_key", 110},
{"o_key", 111},
{"p_key", 112},
{"q_key", 113},
{"r_key", 114},
{"s_key", 115},
{"t_key", 116},
{"u_key", 117},
{"v_key", 118},
{"w_key", 119},
{"x_key", 120},
{"y_key", 121},
{"z_key", 122},
{"left_brace_key", 123},
{"open_brace_key", 123},
{"vertical_bar_key", 124},
{"right_brace_key", 125},
{"close_brace_key", 125},
{"tilde_key", 126},
{"delete_key", 127},
{"leftmouse_button", 128},
{"middlemouse_button", 129},
{"rightmouse_button", 130},
{"select_button", 128},
{"mouse_motion", 131},
{"window_shut", 132},
{"window_quit", 133},
{"input_change", 134},
{"depth_change", 135},
{"window_thaw", 136},
{"window_freeze", 137},
{"f1_key", 138},
{"f2_key", 139},
{"f3_key", 140},
{"f4_key", 141},
{"f5_key", 142},
{"f6_key", 143},
{"f7_key", 144},
{"f8_key", 145},
{"f9_key", 146},
{"f10_key", 147},
{"f11_key", 148},
{"f12_key", 149},
{"left_arrow_key", 150},
{"up_arrow_key", 151},
{"right_arrow_key", 152},
{"down_arrow_key", 153},
{"page_up_key", 154},
{"page_down_key", 155},
{"home_key", 156},
{"end_key", 157},
{"insert_key", 158},
{"shift_key", 159},
{"control_key", 160},
{"alt_key", 161},
{"caps_lock_key", 162}
};


int lookup_simm_key (const char* keyname)
{
   /* First check for an all-integer 'keyname'.  If the joint file included
    * simmkeys.h, then acpp will have already substituted integer key ids
    * in place of symbolic key names.
    */
   const char* p = keyname;
   
   for ( ; *p; p++)
     if ( ! isdigit(*p))
       break;
   
   /* If 'keyname' has no alphabetic characters (ie. all digits), then
    * convert it directly to an integer value.
    */
   if (*p == '\0')
      return atoi(keyname);
   
   /* Otherwise lookup the symbolic keyname and return its value.
    */
   {
      int i, n = sizeof(simmkeys) / sizeof(KeyDef);
   
      for (i = 0; i < n; i++)
      {
         if (strcmp(keyname, simmkeys[i].keyname) == 0)
            return simmkeys[i].keyid;
      }
   }
   return null_key;
}

