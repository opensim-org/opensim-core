/*******************************************************************************

   SIMMKEYS.H

   Author: Peter Loan

   Date: 29-JUL-98

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.
   This file is copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef SIMM_KEYS_H
#define SIMM_KEYS_H

#define null_key 0

#define key_released 0
#define key_pressed 1

// For some reason, CTRL-C and CTRL-V come through as
// ASCII codes 3 (end of text) and 22 (synchronous idle)
// instead of c_key and v_key with the CTRL modifier turned on.
// This is the case for both upper and lower case c and v.
#define copy_key 3
#define paste_key 22

#define FIRST_ASCII_KEY 8
#define backspace_key 8
#define tab_key 9
#define return_key 13
#define enter_key 13
#define escape_key 27

#define space_key 32
#define bang_key 33
#define double_quote_key 34
#define hash_key 35
#define pound_sign_key 35
#define dollar_key 36
#define percent_key 37
#define ampersand_key 38
#define single_quote_key 39
#define open_paren_key 40
#define left_paren_key 40
#define close_paren_key 41
#define right_paren_key 41
#define asterisk_key 42
#define plus_key 43
#define comma_key 44
#define dash_key 45
#define minus_key 45
#define period_key 46
#define slash_key 47
#define zero_key 48
#define one_key 49
#define two_key 50
#define three_key 51
#define four_key 52
#define five_key 53
#define six_key 54
#define seven_key 55
#define eight_key 56
#define nine_key 57
#define colon_key 58
#define semicolon_key 59
#define less_than_key 60
#define equals_key 61
#define greater_than_key 62
#define question_mark_key 63
#define at_sign_key 64
#define A_key 65
#define B_key 66
#define C_key 67
#define D_key 68
#define E_key 69
#define F_key 70
#define G_key 71
#define H_key 72
#define I_key 73
#define J_key 74
#define K_key 75
#define L_key 76
#define M_key 77
#define N_key 78
#define O_key 79
#define P_key 80
#define Q_key 81
#define R_key 82
#define S_key 83
#define T_key 84
#define U_key 85
#define V_key 86
#define W_key 87
#define X_key 88
#define Y_key 89
#define Z_key 90
#define left_bracket_key 91
#define open_bracket_key 91
#define backslash_key 92
#define right_bracket_key 93
#define close_bracket_key 93
#define carat_key 94
#define underscore_key 95
#define back_quote_key 96
#define virgule_key 96
#define a_key 97
#define b_key 98
#define c_key 99
#define d_key 100
#define e_key 101
#define f_key 102
#define g_key 103
#define h_key 104
#define i_key 105
#define j_key 106
#define k_key 107
#define l_key 108
#define m_key 109
#define n_key 110
#define o_key 111
#define p_key 112
#define q_key 113
#define r_key 114
#define s_key 115
#define t_key 116
#define u_key 117
#define v_key 118
#define w_key 119
#define x_key 120
#define y_key 121
#define z_key 122
#define left_brace_key 123
#define open_brace_key 123
#define vertical_bar_key 124
#define right_brace_key 125
#define close_brace_key 125
#define tilde_key 126
#define delete_key 127
#define LAST_ASCII_KEY 127

#define FIRST_MOUSE_BUTTON 128
#define leftmouse_button 128
#define middlemouse_button 129
#define rightmouse_button 130
#define select_button 128
#define mouse_motion 131
#define LAST_MOUSE_BUTTON 131

#define FIRST_WINDOW_ACTION_KEY 132
#define window_shut 132
#define window_quit 133
#define input_change 134
#define depth_change 135
#define window_thaw 136
#define window_freeze 137
#define LAST_WINDOW_ACTION_KEY 137

#define FIRST_SPECIAL_KEY 138
#define f1_key 138
#if 0
#define f2_key 139
#define f3_key 140
#define f4_key 141
#else
#define f2_key leftmouse_button
#define f3_key middlemouse_button
#define f4_key rightmouse_button
#endif
#define f5_key 142
#define f6_key 143
#define f7_key 144
#define f8_key 145
#define f9_key 146
#define f10_key 147
#define f11_key 148
#define f12_key 149

#define left_arrow_key 150
#define up_arrow_key 151
#define right_arrow_key 152
#define down_arrow_key 153
#define page_up_key 154
#define page_down_key 155
#define home_key 156
#define end_key 157
#define insert_key 158
#define LAST_SPECIAL_KEY 158

#define FIRST_MODIFIER_KEY 159
#define shift_key 159
#define control_key 160
#define alt_key 161
#define caps_lock_key 162
#define LAST_MODIFIER_KEY 162

#define max_simm_keys 163

#define IS_ASCII_KEY(I)         ((I) >= FIRST_ASCII_KEY && (I) <= LAST_ASCII_KEY)
#define IS_SPECIAL_KEY(I)       ((I) >= FIRST_SPECIAL_KEY && (I) <= LAST_SPECIAL_KEY)
#define IS_MODIFIER_KEY(I)      ((I) >= FIRST_MODIFIER_KEY && (I) <= LAST_MODIFIER_KEY)
#define IS_MOUSE_BUTTON(I)      ((I) >= FIRST_MOUSE_BUTTON && (I) <= LAST_MOUSE_BUTTON)
#define IS_WINDOW_ACTION_KEY(I) ((I) >= FIRST_WINDOW_ACTION_KEY && (I) <= LAST_WINDOW_ACTION_KEY)

#endif /* SIMM_KEYS_H */
