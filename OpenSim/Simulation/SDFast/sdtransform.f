      subroutine sdtransform(frombdy,fvec,tobdy,tvec)

c  This subroutine transforms a vector from one reference frame to another.
c   The transformation includes translation as well as rotation. This is a
c   function which should be included in SD/FAST but is not.

c==============================================================================
c  Revision record:
c   2/10/93  1:30 pm
c==============================================================================

c  Input:
c   frombdy = body number in which the original vector is defined
c      fvec = vector to be transformed
c     tobdy = body number to which the vector will be transformed
c      tvec = transformed vector
c______________________________________________________________________________

      implicit none

#ifndef UNIX
      interface to subroutine sdpos
     & [C,ALIAS:'_sdpos_'] (aBody,aPoint,rPos)
	integer*4 aBody [REFERENCE]
	real*8 aPoint(3)
	real*8 rPos(3)
	end

      interface to subroutine sdtrans
     & [C,ALIAS:'_sdtrans_'] (aBody1,aVec1,aBody2,rVec2)
	integer*4 aBody1 [REFERENCE]
	real*8 aVec1(3)
	integer*4 aBody2 [REFERENCE]
	real*8 rVec2(3)
	end
#endif

      integer frombdy,tobdy
      real*8 fvec(*),tvec(*),zerovec(3),orgvec(3),ivec(3),dvec(3)


      data zerovec/0.d0,0.d0,0.d0/


c  Get the coordinates of the original vector in the inertial frame:
      call sdpos(frombdy,fvec,ivec)

c  Get the coordinates of the origin of the new frame in the inertial frame:
      call sdpos(tobdy,zerovec,orgvec)

c  Calculate the difference in these two vectors:
      dvec(1) = ivec(1) - orgvec(1)
      dvec(2) = ivec(2) - orgvec(2)
      dvec(3) = ivec(3) - orgvec(3)

c  Rotate this vector to the new frame:
      call sdtrans(0,dvec,tobdy,tvec)


      return
      end
