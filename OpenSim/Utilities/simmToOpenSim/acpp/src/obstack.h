/* obstack.h - object stack macros
   Copyright (C) 1988 Free Software Foundation, Inc.

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


/* Summary:

All the apparent functions defined here are macros. The idea
is that you would use these pre-tested macros to solve a
very specific set of problems, and they would run fast.
Caution: no side-effects in arguments please!! They may be
evaluated MANY times!!

These macros operate a stack of objects.  Each object starts life
small, and may grow to maturity.  (Consider building a word syllable
by syllable.)  An object can move while it is growing.  Once it has
been "finished" it never changes address again.  So the "top of the
stack" is typically an immature growing object, while the rest of the
stack is of mature, fixed size and fixed address objects.

These routines grab large chunks of memory, using a function you
supply, called `obstack_chunk_alloc'.  On occasion, they free chunks,
by calling `obstack_chunk_free'.  You must define them and declare
them before using any obstack macros.

Each independent stack is represented by a `struct obstack'.
Each of the obstack macros expects a pointer to such a structure
as the first argument.

One motivation for this package is the problem of growing char strings
in symbol tables.  Unless you are "fascist pig with a read-only mind"
[Gosper's immortal quote from HAKMEM item 154, out of context] you
would not like to put any arbitrary upper limit on the length of your
symbols.

In practice this often means you will build many short symbols and a
few long symbols.  At the time you are reading a symbol you don't know
how long it is.  One traditional method is to read a symbol into a
buffer, realloc()ating the buffer every time you try to read a symbol
that is longer than the buffer.  This is beaut, but you still will
want to copy the symbol from the buffer to a more permanent
symbol-table entry say about half the time.

With obstacks, you can work differently.  Use one obstack for all symbol
names.  As you read a symbol, grow the name in the obstack gradually.
When the name is complete, finalize it.  Then, if the symbol exists already,
free the newly read name.

The way we do this is to take a large chunk, allocating memory from
low addresses.  When you want to build a symbol in the chunk you just
add chars above the current "high water mark" in the chunk.  When you
have finished adding chars, because you got to the end of the symbol,
you know how long the chars are, and you can create a new object.
Mostly the chars will not burst over the highest address of the chunk,
because you would typically expect a chunk to be (say) 100 times as
long as an average object.

In case that isn't clear, when we have enough chars to make up
the object, THEY ARE ALREADY CONTIGUOUS IN THE CHUNK (guaranteed)
so we just point to it where it lies.  No moving of chars is
needed and this is the second win: potentially long strings need
never be explicitly shuffled. Once an object is formed, it does not
change its address during its lifetime.

When the chars burst over a chunk boundary, we allocate a larger
chunk, and then copy the partly formed object from the end of the old
chunk to the beginning of the new larger chunk.  We then carry on
accreting characters to the end of the object as we normally would.

A special macro is provided to add a single char at a time to a
growing object.  This allows the use of register variables, which
break the ordinary 'growth' macro.

Summary:
    We allocate large chunks.
    We carve out one object at a time from the current chunk.
    Once carved, an object never moves.
    We are free to append data of any size to the currently
      growing object.
    Exactly one object is growing in an obstack at any one time.
    You can run one obstack per control block.
    You may have as many control blocks as you dare.
    Because of the way we do it, you can `unwind' a obstack
      back to a previous state. (You may remove objects much
      as you would with a stack.)
*/


/* Don't do the contents of this file more than once.  */

#ifndef __OBSTACKS__
#define __OBSTACKS__

/* We use subtraction of (char *)0 instead of casting to int
   because on word-addressable machines a simple cast to int
   may ignore the byte-within-word field of the pointer.  */

#ifndef __PTR_TO_INT
#define __PTR_TO_INT(P) ((P) - (char *)0)
#endif

#ifndef __INT_TO_PTR
#define __INT_TO_PTR(P) ((P) + (char *)0)
#endif

struct _obstack_chunk       /* Lives at front of each chunk. */
{
  char  *limit;         /* 1 past end of this chunk */
  struct _obstack_chunk *prev;  /* address of prior chunk or NULL */
  char  contents[4];        /* objects begin here */
};

struct obstack      /* control current object in current chunk */
{
  long  chunk_size;     /* preferred size to allocate chunks in */
  struct _obstack_chunk* chunk; /* address of current struct obstack_chunk */
  char  *object_base;       /* address of object we are building */
  char  *next_free;     /* where to add next char to current object */
  char  *chunk_limit;       /* address of char after current chunk */
  int   temp;           /* Temporary for some macros.  */
  int   alignment_mask;     /* Mask of alignment for each object. */
  struct _obstack_chunk *(*chunkfun) (); /* User's fcn to allocate a chunk.  */
  void (*freefun) ();       /* User's function to free a chunk.  */
};

#ifdef __STDC__

/* Do the function-declarations after the structs
   but before defining the macros.  */

void obstack_init (struct obstack *obstack);

void * obstack_alloc (struct obstack *obstack, int size);

void * obstack_copy (struct obstack *obstack, void *address, int size);
void * obstack_copy0 (struct obstack *obstack, void *address, int size);

void obstack_free (struct obstack *obstack, void *block);

void obstack_blank (struct obstack *obstack, int size);

void obstack_grow (struct obstack *obstack, void *data, int size);
void obstack_grow0 (struct obstack *obstack, void *data, int size);

void obstack_1grow (struct obstack *obstack, int data_char);
void obstack_ptr_grow (struct obstack *obstack, void *data);
void obstack_int_grow (struct obstack *obstack, int data);

void * obstack_finish (struct obstack *obstack);

int obstack_object_size (struct obstack *obstack);

int obstack_room (struct obstack *obstack);
void obstack_1grow_fast (struct obstack *obstack, int data_char);
void obstack_ptr_grow_fast (struct obstack *obstack, void *data);
void obstack_int_grow_fast (struct obstack *obstack, int data);
void obstack_blank_fast (struct obstack *obstack, int size);

void * obstack_base (struct obstack *obstack);
void * obstack_next_free (struct obstack *obstack);
int obstack_alignment_mask (struct obstack *obstack);
int obstack_chunk_size (struct obstack *obstack);

#endif /* __STDC__ */

/* Non-ANSI C cannot really support alternative functions for these macros,
   so we do not declare them.  */

/* Pointer to beginning of object being allocated or to be allocated next.
   Note that this might not be the final address of the object
   because a new chunk might be needed to hold the final size.  */

#define obstack_base(h) ((h)->object_base)

/* Size for allocating ordinary chunks.  */

#define obstack_chunk_size(h) ((h)->chunk_size)

/* Pointer to next byte not yet allocated in current chunk.  */

#define obstack_next_free(h)    ((h)->next_free)

/* Mask specifying low bits that should be clear in address of an object.  */

#define obstack_alignment_mask(h) ((h)->alignment_mask)

#define obstack_init(h) \
  _obstack_begin ((h), 0, 0, obstack_chunk_alloc, obstack_chunk_free)

#define obstack_begin(h, size) \
  _obstack_begin ((h), (size), 0, obstack_chunk_alloc, obstack_chunk_free)

#define obstack_1grow_fast(h,achar) (*((h)->next_free)++ = achar)

#define obstack_blank_fast(h,n) ((h)->next_free += (n))

#if defined (__GNUC__) && defined (__STDC__)

/* For GNU C, if not -traditional,
   we can define these macros to compute all args only once
   without using a global variable.
   Also, we can avoid using the `temp' slot, to make faster code.  */

#define obstack_object_size(OBSTACK)                    \
  ({ struct obstack *__o = (OBSTACK);                   \
     (unsigned) (__o->next_free - __o->object_base); })

#define obstack_room(OBSTACK)                       \
  ({ struct obstack *__o = (OBSTACK);                   \
     (unsigned) (__o->chunk_limit - __o->next_free); })

#define obstack_grow(OBSTACK,where,length)              \
({ struct obstack *__o = (OBSTACK);                 \
   int __len = (length);                        \
   ((__o->next_free + __len > __o->chunk_limit)             \
    ? _obstack_newchunk (__o, __len) : 0);              \
   bcopy (where, __o->next_free, __len);                \
   __o->next_free += __len;                     \
   (void) 0; })

#define obstack_grow0(OBSTACK,where,length)             \
({ struct obstack *__o = (OBSTACK);                 \
   int __len = (length);                        \
   ((__o->next_free + __len + 1 > __o->chunk_limit)         \
    ? _obstack_newchunk (__o, __len + 1) : 0),              \
   bcopy (where, __o->next_free, __len),                \
   __o->next_free += __len,                     \
   *(__o->next_free)++ = 0;                     \
   (void) 0; })

#define obstack_1grow(OBSTACK,datum)                    \
({ struct obstack *__o = (OBSTACK);                 \
   ((__o->next_free + 1 > __o->chunk_limit)             \
    ? _obstack_newchunk (__o, 1) : 0),                  \
   *(__o->next_free)++ = (datum);                   \
   (void) 0; })

/* These assume that the obstack alignment is good enough for pointers or ints,
   and that the data added so far to the current object
   shares that much alignment.  */
   
#define obstack_ptr_grow(OBSTACK,datum)                 \
({ struct obstack *__o = (OBSTACK);                 \
   ((__o->next_free + sizeof (void *) > __o->chunk_limit)       \
    ? _obstack_newchunk (__o, sizeof (void *)) : 0),            \
   *((void **)__o->next_free)++ = ((void *)datum);          \
   (void) 0; })

#define obstack_int_grow(OBSTACK,datum)                 \
({ struct obstack *__o = (OBSTACK);                 \
   ((__o->next_free + sizeof (int) > __o->chunk_limit)          \
    ? _obstack_newchunk (__o, sizeof (int)) : 0),           \
   *((int *)__o->next_free)++ = ((int)datum);               \
   (void) 0; })

#define obstack_ptr_grow_fast(h,aptr) (*((void **)(h)->next_free)++ = (void *)aptr)
#define obstack_int_grow_fast(h,aint) (*((int *)(h)->next_free)++ = (int)aint)

#define obstack_blank(OBSTACK,length)                   \
({ struct obstack *__o = (OBSTACK);                 \
   int __len = (length);                        \
   ((__o->next_free + __len > __o->chunk_limit)             \
    ? _obstack_newchunk (__o, __len) : 0);              \
   __o->next_free += __len;                     \
   (void) 0; })

#define obstack_alloc(OBSTACK,length)                   \
({ struct obstack *__h = (OBSTACK);                 \
   obstack_blank (__h, (length));                   \
   obstack_finish (__h); })

#define obstack_copy(OBSTACK,where,length)              \
({ struct obstack *__h = (OBSTACK);                 \
   obstack_grow (__h, (where), (length));               \
   obstack_finish (__h); })

#define obstack_copy0(OBSTACK,where,length)             \
({ struct obstack *__h = (OBSTACK);                 \
   obstack_grow0 (__h, (where), (length));              \
   obstack_finish (__h); })

#define obstack_finish(OBSTACK)                     \
({ struct obstack *__o = (OBSTACK);                 \
   void *value = (void *) __o->object_base;             \
   __o->next_free                           \
     = __INT_TO_PTR ((__PTR_TO_INT (__o->next_free)+__o->alignment_mask)\
             & ~ (__o->alignment_mask));            \
   ((__o->next_free - (char *)__o->chunk                \
     > __o->chunk_limit - (char *)__o->chunk)               \
    ? (__o->next_free = __o->chunk_limit) : 0);             \
   __o->object_base = __o->next_free;                   \
   value; })

#define obstack_free(OBSTACK, OBJ)                  \
({ struct obstack *__o = (OBSTACK);                 \
   void *__obj = (OBJ);                         \
   if (__obj >= (void *)__o->chunk && __obj < (void *)__o->chunk_limit) \
     __o->next_free = __o->object_base = __obj;             \
   else (obstack_free) (__o, __obj); })

#else /* not __GNUC__ or not __STDC__ */

/* The non-GNU macros copy the obstack-pointer into this global variable
   to avoid multiple evaluation.  */

extern struct obstack *_obstack;

#define obstack_object_size(h) \
 (unsigned) (_obstack = (h), (h)->next_free - (h)->object_base)

#define obstack_room(h)     \
 (unsigned) (_obstack = (h), (h)->chunk_limit - (h)->next_free)

#define obstack_grow(h,where,length)                    \
( (h)->temp = (length),                         \
  (((h)->next_free + (h)->temp > (h)->chunk_limit)          \
   ? _obstack_newchunk ((h), (h)->temp) : 0),               \
  bcopy (where, (h)->next_free, (h)->temp),             \
  (h)->next_free += (h)->temp)

#define obstack_grow0(h,where,length)                   \
( (h)->temp = (length),                         \
  (((h)->next_free + (h)->temp + 1 > (h)->chunk_limit)          \
   ? _obstack_newchunk ((h), (h)->temp + 1) : 0),               \
  bcopy (where, (h)->next_free, (h)->temp),             \
  (h)->next_free += (h)->temp,                      \
  *((h)->next_free)++ = 0)

#define obstack_1grow(h,datum)                      \
( (((h)->next_free + 1 > (h)->chunk_limit)              \
   ? _obstack_newchunk ((h), 1) : 0),                   \
  *((h)->next_free)++ = (datum))

#define obstack_ptr_grow(h,datum)                   \
( (((h)->next_free + sizeof (char *) > (h)->chunk_limit)        \
   ? _obstack_newchunk ((h), sizeof (char *)) : 0),         \
  *((char **)(h)->next_free)++ = ((char *)datum))

#define obstack_int_grow(h,datum)                   \
( (((h)->next_free + sizeof (int) > (h)->chunk_limit)           \
   ? _obstack_newchunk ((h), sizeof (int)) : 0),            \
  *((int *)(h)->next_free)++ = ((int)datum))

#define obstack_ptr_grow_fast(h,aptr) (*((char **)(h)->next_free)++ = (char *)aptr)
#define obstack_int_grow_fast(h,aint) (*((int *)(h)->next_free)++ = (int)aint)

#define obstack_blank(h,length)                     \
( (h)->temp = (length),                         \
  (((h)->next_free + (h)->temp > (h)->chunk_limit)          \
   ? _obstack_newchunk ((h), (h)->temp) : 0),               \
  (h)->next_free += (h)->temp)

#define obstack_alloc(h,length)                     \
 (obstack_blank ((h), (length)), obstack_finish ((h)))

#define obstack_copy(h,where,length)                    \
 (obstack_grow ((h), (where), (length)), obstack_finish ((h)))

#define obstack_copy0(h,where,length)                   \
 (obstack_grow0 ((h), (where), (length)), obstack_finish ((h)))

#define obstack_finish(h)                       \
( (h)->temp = __PTR_TO_INT ((h)->object_base),              \
  (h)->next_free                            \
    = __INT_TO_PTR ((__PTR_TO_INT ((h)->next_free)+(h)->alignment_mask) \
            & ~ ((h)->alignment_mask)),             \
  (((h)->next_free - (char *)(h)->chunk                 \
    > (h)->chunk_limit - (char *)(h)->chunk)                \
   ? ((h)->next_free = (h)->chunk_limit) : 0),              \
  (h)->object_base = (h)->next_free,                    \
  __INT_TO_PTR ((h)->temp))

#ifdef __STDC__
#define obstack_free(h,obj)                     \
( (h)->temp = (char *)(obj) - (char *) (h)->chunk,          \
  (((h)->temp >= 0 && (h)->temp < (h)->chunk_limit - (char *) (h)->chunk)\
   ? (int) ((h)->next_free = (h)->object_base               \
        = (h)->temp + (char *) (h)->chunk)              \
   : ((obstack_free) ((h), (h)->temp + (char *) (h)->chunk), 0)))
#else
#define obstack_free(h,obj)                     \
( (h)->temp = (char *)(obj) - (char *) (h)->chunk,          \
  (((h)->temp >= 0 && (h)->temp < (h)->chunk_limit - (char *) (h)->chunk)\
   ? (int) ((h)->next_free = (h)->object_base               \
        = (h)->temp + (char *) (h)->chunk)              \
   : (int) _obstack_free ((h), (h)->temp + (char *) (h)->chunk)))
#endif

#endif /* not __GNUC__ or not __STDC__ */

#endif /* not __OBSTACKS__ */

