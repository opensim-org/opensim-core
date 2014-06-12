/*******************************************************************************

   BASIC.H

   Author: Peter Loan

   Date: 13-APR-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Basic #defines, enums, and structures needed by most/all ".c" files.

*******************************************************************************/

#ifndef BASIC_H
#define BASIC_H

#define STRUCT typedef struct
#define UNION typedef union
#define ENUM typedef enum

#define public

#define SELECT_BUTTON RIGHTMOUSE

#define DUMMY_LONG 0
#define DUMMY_INT 0

#define ZERO 0 /* used like NULL, but is an int */
#define NO 0
#define YES 1

#define SPACE ' '
#define TAB '\t'
#define CARRIAGE_RETURN '\n'
#define STRING_TERMINATOR '\0'
#define LINE_FEED 13

#ifdef WIN32
  #define DIR_SEP_CHAR   '\\'
  #define DIR_SEP_STRING "\\"
  
  #define COPY_COMMAND   "copy"
#else
  #define DIR_SEP_CHAR   '/'
  #define DIR_SEP_STRING "/"
  
  #define COPY_COMMAND   "cp"
#endif

#define COMMAND_BUFFER 500

#define PIXELSPERINCH 96
#define PIXELSPERINCHF 96.0

#define MENU_ITEM_HEIGHT 24
#define FORM_FIELD_HEIGHT 21
#define FORM_FIELD_YSPACING 27
#define COMBO_ITEM_HEIGHT 20
#define COMBO_ITEM_SIZE 135

#define MAX_MGROUP_MENU_HEIGHT 540
#define MGROUP_MENU_SPACER 20

#define MAX_CHAR_WIDTH 13

#define MAX_NUM_MATERIALS 64

#define FORMITEM_XMARGIN 4
#define FORMITEM_YMARGIN 6

#define FORM_ITEM_STRING_MAX 256
#define FORM_BEGINNING 0
#define FORM_END 1
#define FORM_CURSOR 2
#define FORM_INSERT 3
#define FORM_NAVIGATE 4

#define MODEL_SELECTOR_X 20
#define MODEL_SELECTOR_Y -40

#define END_OF_ARRAY -1

#define NO_WINDOW -1
#define NO_SELECTION -1

#define FILLED 1
#define EMPTY 0
#define DID_NOT_PLACE_MENU 0
#define PLACED_MENU 1

#define NOITEM -1
#define TITLEBOX -2

#define TITLE_BAR -2
#define TITLE_AREA_HEIGHT 65

#define FUNCTION_ARRAY_INCREMENT 24 // might cause crashing, setting to 1 fixes this
#define MOTION_ARRAY_INCREMENT 4
#define DEFAULT_ARRAY_INCREMENT 10

#define CHARBUFFER 4096
#if ENGINE || CORTEX_PLUGIN
  #define MODELBUFFER 4
  #define PLOTBUFFER 1
  #define TOOLBUFFER 1
  #define SCENEBUFFER 4
#else
  #define MODELBUFFER 32
  #define PLOTBUFFER 24
  #define TOOLBUFFER 16
  #define SCENEBUFFER 32
#endif
#define COLORBUFFER 256

#define GENBUFFER 64         /* used only for gencoord help text in MV */
#define GROUPBUFFER 96       /* used by JO and PM model options structs */
#define COLUMNBUFFER 50      /* used by tools which place muscle menus */

#define FIXED_SEGMENT 0
#define MAX_MUSCLE_POINTS 200
#define MAX_PLOT_CURVES 60
#define MAXMDOUBLE 99999999.999
#define MINMDOUBLE -99999999.999
#define TINY_NUMBER 0.0000001
#define ERROR_DOUBLE -999999.3
#define ROUNDOFF_ERROR 0.000000001
#define KEEPOLDDOUBLE -999999.4
#define DONT_CHECK_DOUBLE -999999.5
#define ERRORINT -32760
#define MAXIMUMERROR 0.01
#define DEG_TO_RAD 0.017453292519943
#define RAD_TO_DEG 57.295779513082323
#define DTOR DEG_TO_RAD
#define RTOD RAD_TO_DEG
#define DOUBLE_NOT_DONE -99999.73
#define MAX_MATRIX_SIZE 20
#define MAX_UINT ((unsigned int)~((unsigned int)0))
#define MAX_INT ((int)(MAX_UINT >> 1))

#define UNDEFINED_USERFUNCNUM -9999
#define INVALID_FUNCTION -1
#define INVALID_GENCOORD -1

#define STRING_DONE 0
#define STRING_NOT_DONE 1
#define KEEP_OLD_STRING 2
#define NULL_STRING_ENTERED 3

#define CHECKBOX_XSIZE 18
#define CHECKBOX_YSIZE 18

#define UPDATE_WINDOW 1
#define DONT_UPDATE_WINDOW 0

#define HELP_WINDOW_TEXT_Y_SPACING 20
#define HELP_WINDOW_X_MARGIN 40

#define HIGHLIGHT_TEXT 1
#define CENTER_JUSTIFY 2
#define NO_NEW_LINE 4
#define OVERWRITE_LAST_LINE 8
#define OVERWRITABLE 16
#define SPECIAL_COLOR 32
#define DEFAULT_MESSAGE_X_OFFSET 10

#define SHOW_MODEL 0x00000001
#define SHOW_PLOT  0x00000002
#define SHOW_HELP  0x00000004

#define NULL_SELECTED 0
#define MODEL_SELECTED 1
#define PLOT_SELECTED 2
#define HELP_SELECTED 3

#define NUM_DRAW_MODES 7

#ifndef FALSE
   enum {FALSE, TRUE};
#endif
enum {OFF, ON};

enum {XX, YY, ZZ, WW};
enum {RD, GR, BL};

/* to go along with "typedef float Matrix[4][4]" in gl.h */
typedef double DMatrix[4][4];

typedef double DCoord[3];
typedef double Quat[4];

typedef unsigned __int64 PickIndex;

STRUCT {
   double x1;                         /* minimum x */
   double y1;                         /* minimum y */
   double x2;                         /* maximum x */
   double y2;                         /* maximum y */
} Ortho;                             /* values for ortho() commands */


STRUCT {
   double x;                         /* x coordinate */
   double y;                         /* y coordinate */
} XYCoord;                           /* an (x,y) coordinate pair */


STRUCT {
   int x;                            /* x coordinate */
   int y;                            /* y coordinate */
} XYIntCoord;                        /* an (x,y) integer coordinate pair */


STRUCT {
   int x1;                           /* minimum x */
   int y1;                           /* minimum y */
   int x2;                           /* maximum x */
   int y2;                           /* maximum y */
} IntBox;                            /* a box in integer coordinates */


STRUCT {
   double x1;
   double x2;
   double y1;
   double y2;
   double z1;
   double z2;
} BoundingCube;


STRUCT {
   long x1;                          /* minimum x */
   long y1;                          /* minimum y */
   long x2;                          /* maximum x */
   long y2;                          /* maximum y */
} LongBox;                           /* a box in long coordinates */

STRUCT {
   GLfloat rgb[3];
} ColorRGB;                          /* an rgb triplet defining a color */

ENUM {
   defining_element,                 /* defining an element's properties */
   declaring_element,                /* using the element name, not defining it */
   just_checking_element             /* just checking to see if already defined */
} EnterMode;                         /* modes when entering/defining model elements */

ENUM {
   type_int,                         /* integer */
   type_double,                      /* double */
   type_char,                        /* single character */
   type_string                       /* string */
} VariableType;                      /* variable types that can be read from string */


ENUM {
   up_obj,                           /* object appears to come out of screen */
   down_obj                          /* object appears to go down into screen */
} GUIObjectMode;


ENUM {
   code_fine,                        /* fine, no error was encountered */
   code_bad                          /* bad, an error was encountered */
} ReturnCode;                        /* error condition values */


ENUM {
   recover,                          /* recover from error */
   abort_action,                     /* abort action; critical error */
   exit_program,                     /* exit program; fatal error */
   none                              /* no action; no error */
} ErrorAction;                       /* error recovery actions */


ENUM {
   horizontal_slider,                /*  */
   vertical_slider                   /*  */
} SliderType;                        /*  */


ENUM {
   zeroth,                           /* zeroth derivative */
   first,                            /* first derivative */
   second                            /* second derivative */
} Derivative;                        /* function derivative values */


ENUM {
   FORWARD,                          /* forward for converting a frame/point */
   INVERSE                           /* inverse for converting a frame/point */
} Direction;                         /* directions used in joint traversal */


ENUM {
   from_ground,
   to_ground
} GroundDirection;

ENUM {
   no,                               /* no */
   yes                               /* yes */
} SBoolean;                          /* conventional SBooleanean */


ENUM
{
   off,
   on
} OnOffSwitch;


ENUM
{
   right,
   left
} Justification;


ENUM {
   valid,                            /* clean, still valid */
   invalid                           /* dirty, no longer valid */
} Condition;                         /* conditions for calculated values */


ENUM {
   gouraud_shading=1,
   flat_shading,
   solid_fill,
   wireframe,
   outlined_polygons,
   bounding_box,
   no_surface
} DrawingMode;


ENUM {
   normal_menu,
   toggle_menu
} MenuType;


ENUM {
   normal_checkbox,
   radio_checkbox
} CheckBoxType;

ENUM {
   no_field_action,
   goto_previous_field,
   goto_next_field
} TextFieldAction;

ENUM {
   left_arrow,
   up_arrow,
   right_arrow,
   down_arrow
} ArrowDirection;

STRUCT {
   ArrowDirection direction;
   SBoolean pressed;
   int color;
   int pressed_color;
   IntBox bounding_box;
   XYIntCoord tip;
   XYIntCoord base1;
   XYIntCoord base2;
} ArrowButton;

STRUCT {
   SBoolean visible;
   SBoolean active;
   SliderType type;
   char label[3];
   double value;
   double min_value;
   double max_value;
   double arrow_step;
   int thumb_thickness;
   int thumb_dist;
   IntBox shaft;
   ArrowButton decrease_arrow;
   ArrowButton increase_arrow;
   SBoolean thumb_pressed;
   int thumb_color;
   int background_color;
   int border_color;
   int pressed_thumb_color;
   void* data;                       /* user-supplied data to identify slider */
} Slider;

STRUCT {
   int numsliders;
   Slider* sl;
   XYIntCoord origin;
} SliderArray;

STRUCT {
   SBoolean visible;
   SBoolean active;
   char start_label[10];
   char end_label[10];
   double start_value;
   double end_value;
   double min_value;
   double max_value;
   double arrow_step;
   int thumb_thickness;
   int thumb_dist;
   int thumb_x1;
   int thumb_x2;
   IntBox bounding_box;
   IntBox shaft;
   ArrowButton start_increase_arrow;
   ArrowButton start_decrease_arrow;
   ArrowButton end_increase_arrow;
   ArrowButton end_decrease_arrow;
   SBoolean thumb_pressed;
   int thumb_color;
   int background_color;
   int border_color;
   int pressed_thumb_color;
   void* data;                       /* user-supplied data to identify slider */
   XYIntCoord origin;
} CropSlider;


STRUCT {
   IntBox box;                       /* position of option box */
   char* name;                       /* name of option */
   SBoolean active;                  /* is this menu item active? */
   SBoolean visible;
} MenuItem;                          /* an Menu menu option */


STRUCT {
   MenuType type;                    /* toggle menu or normal menu */
   int numoptions;                   /* number of options */
   IntBox titlebox;                  /* position of title box */
   char* title;                      /* title of menu */
   IntBox bbox;                      /* bounding box of entire menu */
   MenuItem *option;                 /* list of menu-option structures */
   XYIntCoord origin;                /* origin of menu */
} Menu;                              /* properties of an Menu menu */


STRUCT {
   IntBox box;                          /* position of option box */
   char* name;                          /* name of option */
   char valuestr[FORM_ITEM_STRING_MAX]; /* string to hold current value */
   SBoolean justify;                    /* used for lining-up numbers */
   SBoolean active;                     /* whether or not this field is active */
   SBoolean visible;                    /* whether of not this field is visible */
   SBoolean editable;                   /* whether or not this field can be edited */
   SBoolean use_alternate_colors;       /* use alternate colors for display? */
   int decimal_places;                  /* how many places to right of number */
   void* data;                          /* user-supplied data to identify form item */
   int firstVisible;                    /* first character that is visible in the box */
   int lastVisible;                     /* last character that is visible in the box */
} FormItem;                             /* a Form menu option */


STRUCT {
   int numoptions;                   /* number of options */
   char* title;                      /* title of form */
   IntBox bbox;                      /* bounding box of entire form */
   FormItem *option;                 /* list of form-option structures */
   XYIntCoord origin;                /* origin of menu */
   int selected_item;                /* the selected option, if any */
   int cursor_position;              /* where the cursor is in the selected string */
   int highlight_start;              /* first character of selected text in string */
} Form;                              /* properties of a Form */


STRUCT {
   IntBox box;                       /* position of option box */
   char* name;                       /* name of option */
   OnOffSwitch state;                /* state of checkbox (yes,no) */
   Justification just;               /* left or right justified text */
   SBoolean active;                  /* whether or not this checkbox is active */
   SBoolean visible;
   SBoolean use_alternate_colors;//dkb
} CheckBox;                          /* a checkbox */


STRUCT {
   CheckBoxType type;                /* normal (boxes) or radio button (diamonds) */
   int numoptions;                   /* number of options */
   char* title;                      /* title of menu */
   IntBox bbox;                      /* bounding box of entire checkbox region */
   CheckBox *checkbox;               /* list of checkboxes */
   XYIntCoord origin;                /* origin of menu */
} CheckBoxPanel;                     /* holds panel of checkboxes */


STRUCT {
   IntBox box;                       /* position of combobox */
   char* defaultName;                /* name to show when no selected option */
   const char* currentName;          /* name of currently selected option */
   int currentMenuIndex;             /* index in menu of currently selected option */
   SBoolean active;                  /* is this combobox active? */
   SBoolean visible;                 /* is this combobox visible? */
   long popupMenu;                   /* the popup menu of options to choose from */
   void (*menuCallback)(int menuValue, void* userData); /* callback for popup menu */
   void* userData;                   /* user data for popup menu callback */
} ComboBox;                          /* a combobox for choosing one of several options */

STRUCT {
   char *title;
   int numoptions;
   IntBox bbox;                     /* bounding box of entire combobox region */
   ComboBox *combobox;                 /* list of combo boxes */
   XYIntCoord origin;               /* origin of menu */
   int yPosition;
} ComboBoxPanel;

STRUCT {
   OnOffSwitch state;                /* whether this menu is on or off */
   int xo;                           /* x coordinate of menu origin */
   int yo;                           /* y coordinate of menu origin */
   int starting_column;              /* first column this menu occupied, when on */
} MuscleMenu;                        /* state and position of group menus */


STRUCT {
   int block_size;                   /*  */
   int free_ptr;                     /*  */
   void* memory;                     /*  */
} MEMORYBLOCK;                       /*  */


STRUCT {
   double xyz[3];                    /* x, y, and z coordinates */
   float normal[3];                  /* vertex normal (float because of n3f()*/
} Vertex;                            /* description of a polygon vertex */


STRUCT {
   int numpoints;                    /* number of vertices in polygon */
   int pts[25];                      /* list of vertices in polygon */
   float normal[3];                  /* polygon normal */
} SimmPolygon;                       /* description of a polygon */


STRUCT {
   char* text;
   int format;
   int xoffset;
} TextLine;


STRUCT {
   TextLine* line;
   int num_lines_malloced;
   int num_lines;
   int window_id;
   int window_width;
   int window_height;
   Slider sl;
   int lines_per_page;
   int starting_line;
   int background_color;
} HelpStruct;


STRUCT {
   void* defaultfont;         /*  */
   void* largefont;           /*  */
   void* smallfont;           /*  */
   void* italics;             /*  */
   void* bold;                /*  */
} GlobalFonts;                /*  */


/* TOOL QUERIES (sent to a tools query_handler routine) */
ENUM
{
   GET_TOOL_MODEL,
   GET_TOOL_PLOT
} QueryType;

#endif /*BASIC_H*/

