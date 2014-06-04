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
   PLOT,                             /*  */
   PLOTKEY,                          /*  */
   TOOL,                             /*  */
   SCENE,                            /*  */
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
   int model_index;                   /* index of model associated with the event */
   void* struct_ptr1;                 /* user data */
   void* struct_ptr2;                 /* user data */
   int field1;                        /* user data */
   int field2;                        /* user data */
   int mouse_x;                       /* mouse X position */
   int mouse_y;                       /* mouse Y position */
   int key_modifiers;                 /* shift, ctrl, alt key states */
   int window_id;                     /* id of window the event happened in */
   PickIndex object;                  /* for identifying any object in a model */
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
   Scene* scene;                     /*  */
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
   char* name;
   char* value;
} SimmPreference;

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

/* data for handling selecting an object from a popup menu */
typedef struct {
    WindowParams* win_parameters;
    int menu;
    int* submenu;
    int num_submenus;
} ObjectSelectMenuData;

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
   Menu model_selector;              /*  */
   Menu plot_selector;               /*  */
   Menu help_selector;               /*  */
   SBoolean confirm_window_open;     /*  */
   void (*confirm_callback)(SBoolean); /*  */
   ColorDatabase color;              /*  */
   GLStruct gldesc;                  /*  */
   const glutSysInfo* ginfo;         /*  */
   int num_commands;                 /* number of commands in command list */
   char* command[COMMAND_BUFFER];    /* list of commands to be parsed */
   HelpStruct messages;              /*  */
   int num_preferences;
   int preference_array_size;
   SimmPreference* preference;
} RootStruct;                        /*  */
#endif

#endif /*WINDOWROOT_H*/
