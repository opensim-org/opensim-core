 /*******************************************************************************

   WINDOWROOT.H

   Author: Peter Loan

   Date: 13-APR-89

   Copyright (c) 1992 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef WINDOWROOT_H
#define WINDOWROOT_H

#define NO_WINDOW_CHANGE 0
#define CHANGED_ORIGIN 1
#define CHANGED_SIZE 2
#define CHANGED_SIZE_AND_ORIGIN 3

#define WINDOW_ICONIC 0
#define WINDOW_OPEN 1
#define WINDOW_KILLED 2

#define POP_WINDOW 0
#define PUSH_WINDOW 1

#ifdef WIN32
  #define TITLE_BAR_HEIGHT 20
#else
  #define TITLE_BAR_HEIGHT 30
#endif

ENUM {
   ADD,                              /* add new window to the list */
   DELETE,                           /* delete window from the list */
   PUSH,                             /* push window to bottom of the list */
   POP                               /* pop window to top of the list */
} WindowAction;                      /* possible window actions */


ENUM {
   MODEL,                            /*  */
   PLOT,                             /*  */
   PLOTKEY,                          /*  */
   TOOL,                             /*  */
   NOTYPE                            /*  */
} WindowType;                        /*  */


STRUCT {
   int id;                           /* identifier returned by glueOpenWindow() */
   IntBox vp;                        /* viewport */
   Ortho ortho;                      /* values for ortho() call */
   int minwidth;                     /* minimum allowable width */
   int minheight;                    /* minimum allowable height */
   int xo;                           /* x coord of window origin */
   int yo;                           /* y coord of window origin */
   int xsize;                        /* width of window in pixels */
   int ysize;                        /* height of window in pixels */
   char* name;                       /* name of the window */
} WindowParams;                      /* window properties */


STRUCT {
   unsigned int event_code;           /* type of event */
   void* struct_ptr;                  /* user data */
   int field1;                        /*  */
   int field2;                        /* value */
   int mouse_x;                       /* mouse X position */
   int mouse_y;                       /* mouse Y position */
   int key_modifiers;                 /* shift, ctrl, alt key states */
   int window_id;                     /* id of window event happened in */
} SimmEvent;

STRUCT {
   SBoolean used;
   char* name;
   unsigned int simm_event_mask;
   void* tool_struct;
   void (*simm_event_handler)(SimmEvent);
   void (*command_handler)(char*);
   SBoolean (*query_handler)(QueryType, void*);
} ToolStruct;


UNION {
   ToolStruct* tool;                 /*  */
   ModelStruct* model;               /*  */
   PlotStruct* plot;                 /*  */
   PlotKeyStruct* plotkey;           /*  */
} WinUnion;


STRUCT {
   WindowType type;                  /*  */
   int reference_number;             /*  */
   int state;                        /* whether window is iconic or not */
   WinUnion* win_struct;             /*  */
   WindowParams* win_parameters;     /*  */
   void (*input_handler)(WindowParams*, WinUnion*, SimmEvent);
   void (*update_function)(WindowParams*, WinUnion*);
   void (*display_function)(WindowParams*, WinUnion*);
} WindowList;


STRUCT {
   char jointfilepath[150];           /*  */
   char bonefilepath[150];            /*  */
   char plotfilepath[150];            /*  */
   char outputfilepath[150];          /*  */
   SBoolean set_tools_to_new_model;   /*  */
} GlobalPrefStruct;                   /*  */


STRUCT {
   int num_primaries;
   int num_secondaries;
   int num_misc;
   int max_tool_colors;
   int num_tool_colors;
   int first_tool_color_index;
   ColorRGB cmap[COLORBUFFER];
} ColorDatabase;

STRUCT {
   char* name;
   SBoolean defined_yet;
} ElementStruct;

STRUCT {
   long max_screen_x;
   long max_screen_y;
} GLStruct;


STRUCT {
    WindowParams* win_params;
    void*         struct_ptr;
    ModelStruct*  ms;
    PlotStruct*   ps;
} TitleAreaCBParams;

typedef void (*title_area_cb)(int selector, TitleAreaCBParams* params);

#if ! OPENSIM_BUILD
STRUCT {
   IntBox surgwin;                  /*  */
   int basewindow;                   /*  */
   int nummodels;                    /*  */
   int numplots;                     /*  */
   int numtools;                     /*  */
   int numwindows;                   /*  */
   int toptool;                      /*  */
   int currentmodel;                 /*  */
   int currentplot;                  /*  */
   int currentwindow;                /*  */
   int modelcount;                   /* total number of models added so far */
   int numbitplanes;                 /*  */
   int numworldobjects;              /*  */
   long modelmenu;                   /*  */
   long plotmenu;                    /*  */
   long gravityMenu;                 /* menu for choosing direction of gravity */
   WindowList window[200];           /*  */
   GlobalFonts gfont;                /*  */
   Menu confirm_menu;                /*  */
   int events_in_queue;              /*  */
   int event_queue_length;           /*  */
   SimmEvent* simm_event_queue;      /*  */
   GlobalPrefStruct pref;            /*  */
   Menu model_selector;              /*  */
   Menu plot_selector;               /*  */
   Menu help_selector;               /*  */
   SBoolean confirm_window_open;     /*  */
   void (*confirm_callback)(SBoolean); /*  */
   ColorDatabase color;              /*  */
   char* simm_base_dir;              /* directory where SIMM executable is */
   char* simm_dir;                   /* resources directory under SIMM */
   char* color_dir;                  /*  */
   char* help_dir;                   /*  */
   char* mocap_dir;                  /* base directory of mocap import */
   char* mocap_model;                /* joint file used as MOCAP_MODEL */
   GLStruct gldesc;                  /*  */
   const glutSysInfo* ginfo;         /*  */
   int num_commands;                 /* number of commands in command list */
   char* command[500];               /* list of commands to be parsed */
   HelpStruct messages;              /*  */
   SBoolean multiple_screens;        /* support for allowing tools to move beyond main SIMM window */
} RootStruct;                        /*  */
#endif

#endif /*WINDOWROOT_H*/
