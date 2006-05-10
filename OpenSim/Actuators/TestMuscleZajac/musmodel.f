c______________________________________________________________________________
c
c  This file is saved under the name "musmodel.f"
c______________________________________________________________________________

c==============================================================================
c  Revision record:
c   11/18/93  9:10 pm
c==============================================================================




      subroutine mmspec(mussrq,nmus,mpt,fom,lom,lst,pennation,slow,mass)   

c  This subroutine provides the user with a standard set of muscle parameters
c   for the muscles specified.

c______________________________________________________________________________
c
c  Input:
c   mussrq = an assumed length character string indicating which muscle set is
c             requested.
c     nmus = number of muscles (>0)
c      mpt = a vector indicating which muscle data from the specified set are
c             requested.
c
c  Output:
c         fom = maximum isometric strength [Newton]
c         lom = optimal fiber length [meter]
c         lst = tendon slack length [meter]
c   pennation = pennation angle [deg]
c______________________________________________________________________________

      implicit none

      integer i,nmus,mmax,nload

      parameter (mmax=101)

      integer mpt(*)
      real*8 fom(*),lom(*),lst(*),pennation(*),slow(*),mass(*)
      real*8 mspfom(mmax),msplom(mmax),msplst(mmax),msppen(mmax)
      real*8 mspslow(mmax),mspmass(mmax)
      real*8 mifom,milom,milst,mipen,mislow,mimass

      logical ex

      character*4 keyword
      character*256 musset
      character*(*) mussrq



c  Error checking -----

c   Check for none or too many muscles:
      if(nmus .lt. 1)then
       write(6,200)
       write(6,300)
       stop

      elseif(nmus .gt. mmax)then
       write(6,200)
       write(6,400)mmax
       stop

      endif

c ---------------------



c  Load the specified muscle set -----

c  This check is to allow for backward compatibility:

      i = index(mussrq,'/')

      if(i .eq. 0)then
       musset = '/usr/people/BIN/MUSCLE/MUSDAT/'//mussrq
      else
       musset = mussrq
      endif


      inquire(file=musset,exist=ex)

      if(.not. ex)then
       write(6,200)
       write(6,700)musset
       stop
      endif



      open(4,file=musset)

      nload = 0

10    read(4,100,end=50)keyword
      call mmlcase(keyword)
      if(keyword .eq. 'data')go to 30
      go to 10

50    write(6,200)
      write(6,800)mussrq
      stop


30    read(4,*,end=40)mifom,milom,milst,mipen,mislow

      nload = nload + 1
      if(nload .gt. mmax)then
       write(6,600)mmax
       stop
      endif

c  ---------------------------------------------------------------------
c  NOTE- muscle strength was scaled up by 13% 12/12/97 because the model
c        did not have the strength to jump as high as the subjects.
c        Scaling needs to occur here so that
c           1. Jumping and Walking are both scaled
c           2. Muscle mass is properly calculated
c  ---------------------------------------------------------------------
      mspfom(nload) = 1.13d0*mifom
      msplom(nload) = milom
      msplst(nload) = milst
      msppen(nload) = mipen
      mspslow(nload) = mislow
      mspmass(nload) = 1000.d0*mifom*milom/25.d4

      go to 30

40    close(4)

c ------------------------------------


c  More error checking -----

c   Check for bad muscle specification:
      do i = 1,nmus
       if((mpt(i) .lt. 1) .or. (mpt(i) .gt. nload))then
        write(6,200)
        write(6,500)i
        stop
       endif
      enddo

c --------------------------


c  Assign the specified muscle parameters -----

      do i = 1,nmus
       fom(i) = mspfom(mpt(i))
       lom(i) = msplom(mpt(i))
       lst(i) = msplst(mpt(i))
       pennation(i) = msppen(mpt(i))
       slow(i) = mspslow(mpt(i))
       mass(i) = mspmass(mpt(i))
      enddo

c ---------------------------------------------


      return


100   format(a)
200   format(/,1x,'** ERROR **')
300   format(1x,'At least one muscle must be specified.')
400   format(1x,'Too many muscles have been specified. The maximum allow
     &ed = ',i3,'.')
500   format(1x,'Invalid muscle specification for muscle number ',i3,'.'
     &)
600   format(1x,'Too many muscles in input file. The maximum allowed = '
     & ,i3,'.')
700   format(1x,'Cannot access muscle set ',a)
800   format(1x,'Unable to find data in file ',a,/,
     &       1x,'The keyword "data" must appear in the first column imme
     &diately before the',/,
     &1x,'muscle parameters.')

      end





      subroutine mmlcase(string)

c  This subroutine converts a string to all lower case.

      implicit none

      integer i,strlen,ascnum
      character*(*) string


      strlen = len(string)

      do 5 i = 1,strlen

       ascnum = ichar(string(i:i))

       if((ascnum .gt. 64) .and. (ascnum .lt. 91))then
        string(i:i) = char(ascnum+32)
       endif

5     continue

      return
      end







      subroutine mmintl(nmus,fom,lom,lst,pennation,slow,mass)

c  This subroutine initializes the muscle model parameters.

c______________________________________________________________________________
c
c   Argument List -
c          nmus = number of muscles (>0)
c           lom = optimal muscle fiber length [meters]
c           lst = tendon slack length [meters]
c     pennation = pennation angle of muscle [deg]
c
c   Common Blocks - These values are initialized by this routine.
c         mmlom = optimal muscle fiber length [meters]
c         mmlst = tendon slack length [meters]
c      muswidth = muscle width [meters]  = mmlom * sin(pennation angle)
c    muswidthsq = square of the muscle width [meters^2] = muswidth^2
c         reckt = reciprocal of the tendon elastic spring constant [meters]
c        reclom = reciprocal of optimal muscle fiber length (mmlom) [1/meters]
c      recktlom = reckt * reclom
c
c  Other -
c           eat = elastic modulus of tendon
c
c______________________________________________________________________________


      implicit none

      integer i,nmus
      real*8 dtr,eat
      real*8 fom(*),lom(*),lst(*),pennation(*),slow(*),mass(*)

c______________________________________________________________________________

      integer mmax

      parameter (mmax=101)

      real*8 mmfom,mmlom,mmlst,mmslo,mmfas
      real*8 muswidth,muswidthsq,mmmas,reclom,reckt,recktlom

      common /mmbasic1/mmfom(mmax),mmlom(mmax),mmlst(mmax)
      common /mmbasic2/mmslo(mmax),mmfas(mmax)

      common /mmparm1/muswidth(mmax),muswidthsq(mmax),mmmas(mmax)
      common /mmparm2/reclom(mmax),reckt(mmax),recktlom(mmax)

c______________________________________________________________________________


      parameter (eat=37.5d0)


      if(nmus .gt. mmax)then
       print 200,mmax
       stop
      endif


      dtr = dasin(1.d0) / 90.d0


      do i = 1,nmus

c  Load the basic constants into the appropriate vectors:

       if(lom(i) .le. 0.d0)then
        print 400,i,lom(i)
        stop

       elseif(lst(i) .lt. 0.d0)then
        print 500,i,lst(i)
        stop

       endif

       mmfom(i) = fom(i)
       mmlom(i) = lom(i)
       mmlst(i) = lst(i)
       mmslo(i) = slow(i)
       mmfas(i) = 1.d0 - slow(i)
       mmmas(i) = mass(i)


c  Calculate the remaining dependent constants:

       reckt(i) = mmlst(i) / eat
       reclom(i) = 1.d0 / mmlom(i)
       recktlom(i) = reckt(i) * reclom(i)

       muswidth(i) = mmlom(i) * dsin(pennation(i) * dtr)
       muswidthsq(i) = muswidth(i) * muswidth(i)

      enddo


      return


200   format(/,1x,'** ERROR **  Too many muscles. The maximum number all
     &owed = ',i3,/)

400   format(/,1x,'** ERROR **  Illegal value for the optimal fiber leng
     &th of muscle ',i3,
     &      //,1x,'Specified value = ',d13.6,/)

500   format(/,1x,'** ERROR **  Illegal value for the tendon slack lengt
     &h of muscle ',i3,
     &      //,1x,'Specified value = ',d13.6,/)


      end





      subroutine mmfdot(nmus,actlen,actsv,fmus,atv,exc,fdot,edot)

c  This subroutine is the standard muscle model code. It returns the normalized
c   time rate of change of muscle force [fdot].

c______________________________________________________________________________
c
c  Input:
c
c   Argument List -
c    actlen = actuator length [meters]
c     actsv = actuator shortening velocity [meters/sec]
c              Note: lengthing is positive, shortening is negative.
c      fmus = normalized muscle force
c       atv = activation level (0.02 - 1.00)
c       exc = excitation level (0.01 - 1.00)
c      nmus = number of muscles (>0)
c
c   Common Blocks -
c         mmlom = optimal muscle fiber length [meters]
c         mmlst = tendon slack length [meters]
c      muswidth = muscle width [meters]  = mmlom * sin(pennation angle)
c    muswidthsq = square of the muscle width [meters^2] = muswidth^2
c         reckt = reciprocal of the tendon elastic spring constant [meters]
c        reclom = reciprocal of optimal muscle fiber length (mmlom) [1/meters]
c      recktlom = reckt * reclom
c
c  flcoef = coefficients of a quintic spline which defines the force-length
c            curve of muscle.
c  fvcoef = coefficients of a quintic spline which defines the force-velocity
c            curve of muscle.
c
c  Values for "flcoef" and "fvcoef" can be found in a block data subprogram at
c   the end of this file.
c
c
c  Output:
c     fdot = normalized time rate of change of muscle force [1/sec] 
c
c  Other:
c   crvmax = transition point of the force-length curve. If lmnorm is greater
c             than this value, then fdot is determined from an equation which
c             assumes that the actuator force is due entirely to the passive
c             elements.
c______________________________________________________________________________

      implicit none

      integer i,j,ifv,nmus

      real*8 actlen(*),actsv(*),fmus(*),atv(*),exc(*),fdot(*),edot(*)    

      real*8 lm,lt,lmnorm,temp
      real*8 dlm,dpb,pol,xcdot,vbar,pbar
      real*8 zeta,term1,term3,cossq,co,recco,sigma
      real*8 const1,const2,conpe1,sigma0,Vmax,crvmax


c______________________________________________________________________________

      integer mmax

      parameter (mmax=101)

      real*8 mmfom,mmlom,mmlst,mmslo,mmfas
      real*8 muswidth,muswidthsq,mmmas,reclom,reckt,recktlom
      real*8 flcoef,fvcoef,vfcoef
      real*8 xslow,xfast,maintl
      real*8 as,af
      real*8 ms,mf
      real*8 adot,mdot,sdot,wdot,ldot,ddot
 
      external xslow,xfast,maintl

      common /mmbasic1/mmfom(mmax),mmlom(mmax),mmlst(mmax)
      common /mmbasic2/mmslo(mmax),mmfas(mmax)

      common /mmparm1/muswidth(mmax),muswidthsq(mmax),mmmas(mmax)
      common /mmparm2/reclom(mmax),reckt(mmax),recktlom(mmax)

      common /mmflv/flcoef(6,30),fvcoef(6,28),vfcoef(6,30)

c______________________________________________________________________________


      parameter (Vmax=10.0d0, const1=18.0d0, const2=0.020833d0)
      parameter (conpe1=0.1000d0, sigma0=0.0100d0)
      parameter (crvmax=1.60d0)
      parameter (as=40.d0,af=133.d0)
      parameter (ms=74.d0,mf=111.d0)


      do i = 1,nmus


c  Tendon and Passive Elastic element:

c   lt = tendon length at fmus.
      lt = reckt(i) * fmus(i) + mmlst(i)



c   lm = muscle length. When lm is too short, adjust it.
      temp = dmax1( 0.0d0 , (actlen(i) - lt) )
      lm = dsqrt( muswidthsq(i) + temp*temp )

      write(*,9) 'actlen(i) =', actlen(i)
 9    format (A,F)
      write(*,8) 'temp =', temp
 8    format (A,F)


      write(*,10) 'lm =', lm
10    format (A,F)

c   muscle length must be greater than its width.
      lm = dmax1( lm, muswidth(i)+1.0d-4 )

c   normalized muscle-fiber length.
      lmnorm = lm * reclom(i)

      write(*,11) 'lmnorm =', lmnorm
11    format (A,F)


c  Check the length of the muscle fiber to determine how to evaluate the time
c   rate of change of muscle force [fdot].  If the length is greater than 
c    crvmax*lmnorm, then assume the force is due entirely to the passive
c    elements.  Otherwise, use the standard muscle model, which includes the
c    force-length and force-velocity curves, to find fdot.


c   sigma = normalized parallel elastic spring force.
      sigma = sigma0 * dexp( (lmnorm - 1.0d0) / conpe1 )


      if(lmnorm .gt. crvmax)then

c  The muscle fiber length is too long.

       fdot(i) = actsv(i)/(reckt(i)*(1.d0+conpe1/(sigma*recktlom(i))))
      write(*,17) 'lmnorm =', lmnorm
17    format (A,F)

      else

c  The muscle fiber length is within the range the force-length curve.

c  Force-length curve:

       ifv = min0( 30 , (int( lmnorm*15.0d0 ) + 1) )
       dlm = lmnorm - dble(ifv-1) / 15.d0

c   pol = normalized sarcomere muscle force.

       pol = (((( flcoef(6,ifv)  *dlm + flcoef(5,ifv) )*dlm
     &          + flcoef(4,ifv) )*dlm + flcoef(3,ifv) )*dlm
     &          + flcoef(2,ifv) )*dlm + flcoef(1,ifv)


c   co = cos(alpha).
       co = dsqrt( 1.0d0 - muswidthsq(i)/(lm*lm) )
       recco = 1.0d0 / co


c  Find Xcdot, the shortening velocity of muscle.
c   normalized muscle force = normalized tendon force / cos(alpha)

       pbar =  (fmus(i)*recco - sigma) / (pol * atv(i))

       pbar = dmax1(0.d0,pbar)

       ifv = min0( 28 , (int( pbar*16.0d0 ) + 1) )
       dpb = pbar - dble(ifv-1) / 16.d0

       vbar = (((( fvcoef(6,ifv)  *dpb + fvcoef(5,ifv) )*dpb
     &           + fvcoef(4,ifv) )*dpb + fvcoef(3,ifv) )*dpb
     &           + fvcoef(2,ifv) )*dpb + fvcoef(1,ifv) + 2.1220d-03

       vbar = dmin1(2.d0,vbar)




       Xcdot = Vmax*vbar




c  Find fdot, uses the force velocity curve for muscle.

       cossq = co * co
       zeta = actsv(i) * reclom(i)

      write(*,117) 'zeta =', zeta
117   format (A,F)

       term1 = fmus(i) * ( const1 + (1.0d0 - cossq)/cossq )
     &         + co * ( const1*const2 + (1.0d0/conpe1 - const1)*sigma )
       term3 = const1 * ( fmus(i)*recco - sigma + const2 )

      write(*,118) 'term1 =', term1
118   format (A,F)
      write(*,119) 'term3 =', term3
119   format (A,F)


       fdot(i) = (zeta*term1-Xcdot*term3) / (recco+recktlom(i)*term1)
      write(*,121) 'recco =', recco
121   format (A,F)
      write(*,122) 'recktlom(i) =', recktlom(i)
122   format (A,F)
      write(*,120) 'fdot(i) =', fdot(i)
120   format (A,F)

      endif


c  Muscle Metabolism
c   Activation Rate
        adot = mmmas(i)*
     &         (as*mmslo(i)*xslow(exc(i))+af*mmfas(i)*xfast(exc(i)))  
c   Maintenance Rate
ccc        mdot = 10.81d0*mmmas(i)*maintl(lmnorm)*
ccc     &         (ms*mmslo(i)*xslow(atv(i))+mf*mmfas(i)*xfast(atv(i)))
        mdot = mmmas(i)*maintl(lmnorm)*
     &         (ms*mmslo(i)*xslow(atv(i))+mf*mmfas(i)*xfast(atv(i)))
c   Work Rate
        wdot = -1.d0 * (mmfom(i)*fmus(i)/co - sigma) * mmlom(i)*Xcdot
        if (wdot.lt.0.d0) then
ccc          wdot = -0.5*wdot
          wdot = 0.0d0
        endif
c   Shortening Rate
        sdot = 0.25d0 * wdot 
c   Dissipation Rate
        ldot = (actlen(i)-lt)*(actsv(i)-mmfom(i)*fdot(i)*reckt(i))/lm
        ddot = 200.d0 * ldot**2 
c   Total
c        edot(i) = adot
c        edot(i+nmus) = mdot
c        edot(i+2*nmus) = sdot
c        edot(i+3*nmus) = wdot 
c        edot(i+4*nmus) = 0.0d0

      enddo



      return
      end




      real*8 function maintl(l)
c================================================================
c Maintenance heat rate function for length dependancy 
c================================================================
      real*8 l
      maintl = 0.0d0
      if (l.le.0.5d0) then
        maintl = 0.5d0
      elseif ((l.gt.0.5d0).and.(l.le.1.d0)) then
        maintl = l
      elseif ((l.gt.1.d0).and.(l.le.1.5d0)) then
        maintl = -2.d0*l + 3.d0
      elseif (l.gt.1.5d0) then
        maintl = 0.0d0
      endif
      return
      end
c================================================================


      real*8 function xslow(x)
c================================================================
c Attempt at taking into account size principle
c================================================================
      real*8 x,pi
      parameter(pi=3.14159265359d0)
      xslow = dsin(0.5d0*pi*x) 
      return
      end
c================================================================


      real*8 function xfast(x)
c================================================================
c Attempt at taking into account size principle
c================================================================
      real*8 x,pi
      parameter(pi=3.14159265359d0)
      xfast = 1.d0 - dcos(0.5d0*pi*x) 
      return
      end
c================================================================







      subroutine mmatvdot(nmus,ut,atv,k1,k2,atvdot)

c  This subroutine calculates time rate of change of a muscle's activation
c   based on the its current excitation and activation.

c______________________________________________________________________________
c
c  Input:
c   nmus = number of muscles
c     ut = muscle excitation (controls)
c    atv = muscle activation
c     k1 = rise time constant
c     k2 = decay time constant
c
c  Output:
c   atvdot = time rate of change of muscle activation
c
c
c  Excitation  Dynamics  Equation  of  Muscle:
c
c    da(t)/dt = k1*u(t)^2 + k2*u(t) - (k1*u(t) + k2)*a(t)
c
c       where
c         amin =< a(t) <= amax: amin=0.0 , amax=1.0
c         umin =< u(t) <= umax: umin=0.02, umax=1.0
c______________________________________________________________________________


      implicit none
       
      integer i,nmus
      real*8 k1,k2,ut(*),atv(*),atvdot(*)


      do i = 1,nmus
       atvdot(i) = k1*ut(i)*ut(i) + k2*ut(i) - (k1*ut(i)+k2) * atv(i)
      enddo
      write(*,120) 'atvdot =', atvdot
120   format (A,F)

      return
      end





      real*8 function mslm(i,atv,ax,bx,tol,actlen,actsv)

c  This routine calculates a zero of a function in the interval (ax,bx).
c   This routine was formerly known as "zeroin".

      integer i

      real*8 atv,ax,bx,tol,actlen,actsv
      real*8 mszero,eps
      real*8 a,b,c,d,e,fa,fb,fc,toli,xm,p,q,r,s
      
      external mszero
      
      parameter (eps=6.93889d-18)    ! machine precision in r*8
c      parameter (eps=2.98e-08)       ! machine precision in r*4
 

c______________________________________________________________________________ 
c
c  Input:  All inputs are scalars
c
c        i = specific muscle number
c      atv = activation level
c       ax = left endpoint of initial interval
c       bx = right endpoint of initial interval
c      tol = desired length of the interval of uncertainty of the final
c             result ( >= 0.0 )
c   actlen = total actuator length [meters]
c    actsv = actuator shortening velocity [meters/sec]
c
c 
c  Output:
c 
c    mslm = abcissa approximating a zero of mszero in the interval (ax,bx).
c
c
c  Other:
c
c   mszero = function subprogram which finds the difference in actuator length
c             and the sum of muscle fiber length and tendon length
c             [lmt - (lm+lt)]. This will become zero when the appropriate lm
c             is determined, which is the purpose of this routine.
c 
c 
c  It is assumed that mszero(ax) and mszero(bx) have opposite signs without
c   a check.  mslm returns a zero x in the given interval (ax,bx) to within
c   a tolerance 4*macheps*abs(x) + tol, where macheps is the relative
c   machine precision.
c
c  This function subprogram is a slightly modified translation of the algol 60
c   procedure "zero" given in Richard Brent, Algorithms for Minimization
c   Without Derivatives, Prentice - Hall, Inc. (1973).
c______________________________________________________________________________
 
 
c  Initialization
 
      a = ax
      b = bx

      write(*,1) 'a =', a
1     format (A,F)

      write(*,3) 'b =', b
3     format (A,F)


      fa = mszero(i,atv,a,actlen,actsv)
      fb = mszero(i,atv,b,actlen,actsv)

      write(*,120) 'fa =', fa
120   format (A,F)

      write(*,130) 'fb =', fb
130   format (A,F)
 
      write(*,122) 'voor loop a =', a
122   format (A,F)

 
c  Begin step
      write(*,123) 'in  loop a =', a
123   format (A,F)

  20  c = a
      fc = fa
      d = b - a
      e = d
  
      write(*,140) 'c =', c
140   format (A,F)

      write(*,150) 'fc =', fc
150   format (A,F)

      write(*,160) 'e =', e
160   format (A,F)


  30  if (dabs(fc) .ge. dabs(fb)) go to 40
      a = b
      b = c
      c = a
      fa = fb
      fb = fc
      fc = fa
      write(*,161) '30a =', a
161   format (A,F) 
      write(*,170) '30b =', b
170   format (A,F) 
      write(*,180) '30c =', c
180   format (A,F) 
      write(*,190) '30fa =', fa
190   format (A,F) 
      write(*,200) '30fb =', fb
200   format (A,F) 
      write(*,210) '30fc =', fc
210   format (A,F) 



c  Convergence test
 
  40  toli = 20.d0*eps*dabs(b) + 0.5d0*tol
      xm = 0.5d0*(c - b)

      write(*,213) '40c =', c
213   format (A,F) 
      write(*,214) '40b =', b
214   format (A,F) 

      write(*,211) '40xm =', xm
211   format (A,F) 
      write(*,212) '40toli =', toli
212   format (A,F) 

      if (dabs(xm) .le. toli) go to 90
      if (fb .eq. 0.d0) go to 90
 
c  Is bisection necessary?
 
      if (dabs(e) .lt. toli) go to 70
      if (dabs(fa) .le. dabs(fb)) go to 70
 
c  Is quadratic interpolation possible?
 
      if (a .ne. c) go to 50
 
c  Linear interpolation
 
      s = fb/fa
      p = 2.d0*xm*s
      q = 1.d0 - s
      go to 60
 
c  Inverse quadratic interpolation
 
  50  q = fa/fc
      r = fb/fc
      s = fb/fa
      p = s*(2.d0*xm*q*(q - r) - (b - a)*(r - 1.d0))
      q = (q - 1.d0)*(r - 1.d0)*(s - 1.d0)
 
c  Adjust signs
 
  60  if (p .gt. 0.d0) q = -q
      p = dabs(p)
 
c  Is interpolation acceptable?
 
      write(*,361) '30a =', s
361   format (A,F) 
      write(*,370) '30b =', p
370   format (A,F) 
      write(*,380) '30c =', q
380   format (A,F) 

      if ((2.d0*p) .ge. (3.d0*xm*q - dabs(toli*q))) go to 70
      if (p .ge. dabs(0.5d0*e*q)) go to 70
      e = d
      d = p/q
      go to 80
 
c  Bisection
 
  70  d = xm
      e = d
 
c  Complete step
 
  80  a = b
      fa = fb
      if (dabs(d) .gt. toli) b = b + d
      if (dabs(d) .le. toli) b = b + dsign(toli,xm)
      
      fb = mszero(i,atv,b,actlen,actsv)

       write(*,461) '80a =', a
461   format (A,F) 
      write(*,470) '80b =', b
470   format (A,F) 
      write(*,480) '80c =', c
480   format (A,F) 
      write(*,490) '80fa =', fa
490   format (A,F) 
      write(*,400) '80fb =', fb
400   format (A,F) 
      write(*,410) '80fc =', fc
410   format (A,F) 
     
      if ((fb*(fc/dabs(fc))) .gt. 0.d0) go to 20
      go to 30
 
c  Done
 
  90  mslm = b
      write(*,171) '90mslm =', mslm
171   format (A,F)  
      return
      end
 
 
 
 
 
      real*8 function mszero(i,atv,lm,actlen,actsv)
 
      implicit none
 
      integer i
      real*8 atv,lm,actlen,actsv
      real*8 fmus,lt,msforce,temp

c______________________________________________________________________________

      integer mmax

      parameter (mmax=101)

      real*8 mmfom,mmlom,mmlst,mmslo,mmfas
      real*8 muswidth,muswidthsq,mmmas,reclom,reckt,recktlom

      common /mmbasic1/mmfom(mmax),mmlom(mmax),mmlst(mmax)
      common /mmbasic2/mmslo(mmax),mmfas(mmax)

      common /mmparm1/muswidth(mmax),muswidthsq(mmax),mmmas(mmax)
      common /mmparm2/reclom(mmax),reckt(mmax),recktlom(mmax)

c______________________________________________________________________________

 
      external msforce
 
 
c  muscle length must be greater than the width.
      lm = dmax1( lm, muswidth(i)+1.0d-4 )
       fmus = msforce(i,atv,lm,actsv)
 
c  find the length of tendon at the normalized muscle force "fmus".

      lt = mmlst(i) + fmus*reckt(i)
 
c  evaluate the error in this function.

      temp = dmax1( 0.d0 , (actlen - lt) )
      mszero = temp*temp - lm*lm + muswidthsq(i)

      write(*,140) 'mszero =', mszero
140   format (A,F)  
      return
      end
 
 


 
 
 
      real*8 function msforce(i,atv,lm,actsv)

c  This routine calculates the normalized muscle force.
 
      implicit none
 
      integer i,ifv
 
      real*8 atv,lm,actsv,lmnorm
      real*8 dlm,dpb,pol,co,recco,fmus,vbar,pbar,sigma
      real*8 conpe1,sigma0,Vmax,crvmax

c______________________________________________________________________________

      integer mmax,temp

      parameter (mmax=101)

      real*8 mmfom,mmlom,mmlst,mmslo,mmfas
      real*8 muswidth,muswidthsq,mmmas,reclom,reckt,recktlom
      real*8 flcoef,fvcoef,vfcoef

      common /mmbasic1/mmfom(mmax),mmlom(mmax),mmlst(mmax)
      common /mmbasic2/mmslo(mmax),mmfas(mmax)

      common /mmparm1/muswidth(mmax),muswidthsq(mmax),mmmas(mmax)
      common /mmparm2/reclom(mmax),reckt(mmax),recktlom(mmax)
      
      common /mmflv/flcoef(6,30),fvcoef(6,28),vfcoef(6,30)

c______________________________________________________________________________

 
      parameter (conpe1=0.1000d0, sigma0=0.0100d0, Vmax=10.d0)
      parameter (crvmax=1.60d0)
 
 
 
c  Sigma = normalized parallel elastic spring force.
      lmnorm = lm * reclom(i)
   

      sigma = sigma0 * dexp( (lmnorm - 1.d0) / conpe1 )

      if(lmnorm .gt. crvmax)then
       msforce = sigma
       return
      endif
 

c  co = cos(pennation angle).

      co = dsqrt( 1.d0 - muswidthsq(i)/(lm*lm) )
  
      recco = 1.d0 / co
 
c Force-length curve:

      ifv = min0( 30 , (int( lmnorm*15.d0 ) + 1) )

      dlm = lmnorm - dble(ifv-1) / 15.d0
 

      pol = (((( flcoef(6,ifv)  *dlm + flcoef(5,ifv) )*dlm
     &         + flcoef(4,ifv) )*dlm + flcoef(3,ifv) )*dlm
     &         + flcoef(2,ifv) )*dlm + flcoef(1,ifv)



       write(*,150) 'pol =', pol
150   format (A,F) 
   
c Find the change in force due to the shortening velocity of muscle.
c  For simplicity, assume that the shortening velocity of muscle is the same
c  as that of the actuator. In other words, assume that the shortening velocity
c  of tendon is zero.
 
      vbar = actsv * reclom(i) * co / Vmax
 
      vbar = dmax1( -1.d0 , vbar)
      vbar = dmin1(  2.d0 , vbar)

 
  
  
      ifv = min0( 30 , (int( vbar*10.d0 ) + 11) )
      dpb = vbar - dble(ifv-11) * 0.1d0
 
      pbar = (((( vfcoef(6,ifv)  *dpb + vfcoef(5,ifv) )*dpb
     &          + vfcoef(4,ifv) )*dpb + vfcoef(3,ifv) )*dpb
     &          + vfcoef(2,ifv) )*dpb + vfcoef(1,ifv) + 2.1220d-03
 
      pbar = dmax1(0.d0 , pbar)


      msforce = co*sigma + pol*co*atv*pbar

      return
      end
 
 
 
 
 
 
      real*8 function msatv(i,fmus,actlen,actsv)
 
c  This routine returns the activation of a muscle given the required
c   conditions.
 
      implicit none
 
      integer i,j,ifv
 
      real*8 fmus,actlen,actsv
      real*8 lm,lt,lmnorm,temp,dlm,dpb,pol,vbar,pbar,co
      real*8 conpe1,sigma0,sigma,Vmax,crvmax

c______________________________________________________________________________

      integer mmax

      parameter (mmax=101)

      real*8 mmfom,mmlom,mmlst,mmslo,mmfas
      real*8 muswidth,muswidthsq,mmmas,reclom,reckt,recktlom
      real*8 flcoef,fvcoef,vfcoef

      common /mmbasic1/mmfom(mmax),mmlom(mmax),mmlst(mmax)
      common /mmbasic2/mmslo(mmax),mmfas(mmax)

      common /mmparm1/muswidth(mmax),muswidthsq(mmax),mmmas(mmax)
      common /mmparm2/reclom(mmax),reckt(mmax),recktlom(mmax)

      common /mmflv/flcoef(6,30),fvcoef(6,28),vfcoef(6,30)

c______________________________________________________________________________


 
      parameter (conpe1=0.1000d0, sigma0=0.0100d0, Vmax=10.d0)
      parameter (crvmax=1.60d0)
 
 
 
c  Tendon and passive elastic element:

c   lt = tendon length at fmus.
      lt = reckt(i) * fmus + mmlst(i)

c   lm = muscle length. when lm is too short, adjust it.
      temp = dmax1( 0.0d0 , (actlen - lt) )
      lm = dsqrt( muswidthsq(i) + temp*temp )

c   muscle length must be greater than the width.
      lm = dmax1( lm, muswidth(i)+1.0d-4 )


c  Normalized muscle fiber length.
      lmnorm = lm * reclom(i)

      if(lmnorm .gt. crvmax)then
       msatv = 0.5d0
       return
      endif


c   sigma = normalized parallel elastic spring force.
      sigma = sigma0 * dexp( (lmnorm - 1.0d0) / conpe1 )
 
 
c  Force-length curve:
 
      ifv = min0( 30 , (int( lmnorm*15.0d0 ) + 1) )
      dlm = lmnorm - dble(ifv-1) / 15.d0
 
      pol = (((( flcoef(6,ifv)  *dlm + flcoef(5,ifv) )*dlm
     &         + flcoef(4,ifv) )*dlm + flcoef(3,ifv) )*dlm
     &         + flcoef(2,ifv) )*dlm + flcoef(1,ifv)
 
 
c   co = cos(pennation angle).
      co = dsqrt( 1.0d0 - muswidthsq(i)/(lm*lm) )
 
 
c Find the change in force due to the shortening velocity of muscle.
c  for simplicity, assume that the shortening velocity of muscle is the same
c  as that of the actuator. in other words, assume that the shortening velocity
c  of tendon is zero.
 
      vbar = actsv * reclom(i) * co / Vmax
      vbar = dmax1( -1.d0 , vbar)
      vbar = dmin1(  2.d0 , vbar)
 
      ifv = min0( 30 , (int( vbar*10.d0 ) + 11) )
      dpb = vbar - dble(ifv-11) * 0.1d0
 
      pbar = (((( vfcoef(6,ifv)  *dpb + vfcoef(5,ifv) )*dpb
     &          + vfcoef(4,ifv) )*dpb + vfcoef(3,ifv) )*dpb
     &          + vfcoef(2,ifv) )*dpb + vfcoef(1,ifv) + 2.1220d-03
 
      pbar = dmax1(0.d0 , pbar)
 
      pol = pol*pbar
 
 
      msatv = (fmus/co - sigma) / pol
 
 
      return
      end





      real*8 function musfrc(mus,atv,actlen,actsv)

c  This function calculates the muscle force given the muscle's activation,
c   length, and shortening velocity.

      implicit none

c______________________________________________________________________________

      integer mus,mmax
      parameter (mmax=101)

      real*8 mslm,msforce,lm,atv,muswidth,muswidthsq,mmmas
      real*8 mw,actlen,actsv

      common /mmparm1/muswidth(mmax),muswidthsq(mmax),mmmas(mmax)
c______________________________________________________________________________


      external mslm
      external msforce


      mw = muswidth(mus)

c  Find the contractile element length of the muscle:


      write(*,120) 'mw =', mw
120    format (A,F)
      write(*,130) 'actlen =', actlen
130    format (A,F)
      write(*,140) 'actsv =', actsv
140    format (A,F)



      lm = mslm(mus,atv,mw,1.d0,0.00001d0,actlen,actsv)
c  Clay Anderson changed the tolerance to speed computation 2003_05_13
c      lm = mslm(mus,atv,mw,1.d0,0.00005d0,actlen,actsv)
      write(*,100) 'lm =', lm
100   format (A,F)	
c  Get the maximum allowed muscle force:
      musfrc = msforce(mus,atv,lm,actsv)
      write(*,101) 'musfrc =', musfrc
101   format (A,F)	

      return
      end







      block data mmflvdat

c   This file contains data for the normalized force-length, velocity-force,
c    and force-velocity curves.

c  Note: On the SUN, this block data subprogram must be compiled with the
c          -Nl60 option. This is because the data statements have sixty (60)
c          continuation lines.

      implicit none

c______________________________________________________________________________

      integer mmax

      parameter (mmax=101)

      real*8 mmfom,mmlom,mmlst,mmslo,mmfas
      real*8 muswidth,muswidthsq,mmmas,reclom,reckt,recktlom
      real*8 flcoef,fvcoef,vfcoef

      common /mmbasic1/mmfom(mmax),mmlom(mmax),mmlst(mmax)
      common /mmbasic2/mmslo(mmax),mmfas(mmax)

      common /mmparm1/muswidth(mmax),muswidthsq(mmax),mmmas(mmax)
      common /mmparm2/reclom(mmax),reckt(mmax),recktlom(mmax)

      common /mmflv/flcoef(6,30),fvcoef(6,28),vfcoef(6,30)

c______________________________________________________________________________



      data flcoef/ 0.29999999E-01, 0.24252515E-01, 0.83434001E-01,
     &             0.00000000E+00, 0.00000000E+00, 0.93781357E+01,
     &             0.32000002E-01, 0.36303282E-01, 0.11122108E+00,
     &             0.41680610E+00, 0.31260455E+01,-0.75772491E+02,
     &             0.35000000E-01, 0.52911408E-01, 0.53432360E-01,
     &            -0.21172471E+01,-0.22131449E+02, 0.22750180E+03,
     &             0.37999999E-01, 0.28045202E-01,-0.28610954E+00,
     &             0.20922267E+01, 0.53702496E+02,-0.21170753E+03,
     &             0.39999999E-01, 0.60531657E-01, 0.93712115E+00,
     &             0.70036678E+01,-0.16866692E+02, 0.43728191E+02,
     &             0.50000001E-01, 0.26319203E+00, 0.20176411E+01,
     &             0.44493580E+01,-0.22906296E+01, 0.16808680E+04,
     &             0.79999998E-01, 0.75483233E+00, 0.78267779E+01,
     &             0.78543747E+02, 0.55799872E+03,-0.57448760E+04,
     &             0.19183697E+00, 0.29395888E+01, 0.21393633E+02,
     &            -0.27984400E+02,-0.13569598E+04, 0.58263604E+04,
     &             0.45546898E+00, 0.43861423E+01,-0.31255426E+01,
     &            -0.13089108E+03, 0.58515930E+03,-0.59455225E+03,
     &             0.70598048E+00, 0.28589883E+01,-0.15461145E+02,
     &            -0.12730635E+01, 0.38697504E+03,-0.11397749E+04,
     &             0.83362931E+00, 0.11265945E+01,-0.87735367E+01,
     &             0.51263630E+02, 0.70501127E+01,-0.66309363E+03,
     &             0.88419741E+00, 0.58316940E+00,-0.29753122E+00,
     &             0.23672834E+02,-0.21398106E+03, 0.57005804E+03,
     &             0.92529106E+00, 0.66183126E+00, 0.41993487E+00,
     &            -0.80528736E+01,-0.23961737E+02, 0.64066841E+02,
     &             0.96850455E+00, 0.58837938E+00,-0.16397921E+01,
     &            -0.11595254E+02,-0.26061234E+01, 0.71818405E+02,
     &             0.99704933E+00, 0.21914154E+00,-0.38155444E+01,
     &            -0.90982914E+01, 0.21333344E+02, 0.22131031E+01,
     &             0.99242932E+00,-0.38540557E+00,-0.50597558E+01,
     &            -0.33110399E+01, 0.22071045E+02,-0.34520439E+02,
     &             0.94365722E+00,-0.10814385E+01,-0.52356844E+01,
     &             0.10403328E+01, 0.10564221E+02, 0.69808632E+02,
     &             0.84890056E+00,-0.17462429E+01,-0.45390658E+01,
     &             0.69600554E+01, 0.33833744E+02,-0.20842517E+03,
     &             0.71476674E+00,-0.22391374E+01,-0.28623760E+01,
     &             0.67190380E+01,-0.35641376E+02, 0.43575076E+03,
     &             0.55463004E+00,-0.25304043E+01,-0.11778954E+01,
     &             0.16581345E+02, 0.10960875E+03,-0.10678167E+04,
     &             0.38637313E+00,-0.24419289E+01, 0.18973714E+01,
     &            -0.16482311E+01,-0.24633047E+03, 0.23621240E+04,
     &             0.22976723E+00,-0.22695737E+01, 0.19977926E+01,
     &             0.37646801E+02, 0.54104352E+03,-0.41256050E+04,
     &             0.10375030E+00,-0.12674731E+01, 0.11730966E+02,
     &            -0.14353340E+01,-0.83415924E+03, 0.41862578E+04,
     &             0.59999999E-01,-0.29765821E+00, 0.16033850E+01,
     &            -0.37822041E+02, 0.56125873E+03,-0.24013950E+04,
     &             0.44000000E-01,-0.16014636E+00, 0.18906338E+01,
     &             0.51182389E+01,-0.23920692E+03, 0.11255194E+04,
     &             0.39999999E-01,-0.12160726E-01,-0.12969553E+00,
     &            -0.86472273E+01, 0.13596582E+03,-0.55925067E+03,
     &             0.37999999E-01,-0.38839772E-01, 0.10957568E+00,
     &             0.27547290E+01,-0.50451214E+02, 0.21464819E+03,
     &             0.35999998E-01,-0.26094167E-01,-0.48849393E-01,
     &            -0.11590136E+01, 0.21098116E+02,-0.88555939E+02,
     &             0.34000002E-01,-0.31802028E-01, 0.19576361E-01,
     &             0.53132880E+00,-0.84205570E+01, 0.31915079E+02,
     &             0.32000002E-01,-0.28935270E-01,-0.41427822E-02,
     &            -0.29570580E+00, 0.22177913E+01,-0.66533685E+01/



      data fvcoef/-0.99995804E+00,  0.50985093E+01, -0.12840916E+02, 
     &             0.00000000E+00,  0.00000000E+00,  0.24692949E+03, 
     &            -0.73122555E+00,  0.35122344E+01, -0.12238060E+02, 
     &             0.96456804E+01,  0.77165466E+02, -0.28950901E+02, 
     &            -0.55601108E+00,  0.21686606E+01, -0.86916122E+01, 
     &             0.27806149E+02,  0.68118301E+02, -0.63705542E+03, 
     &            -0.44720095E+00,  0.14259807E+01, -0.34367480E+01, 
     &             0.19950750E+02, -0.13096150E+03,  0.33955386E+03, 
     &            -0.36830568E+00,  0.11281989E+01, -0.19364038E+01, 
     &             0.47419998E+00, -0.24850937E+02,  0.20577457E+03, 
     &            -0.30542454E+00,  0.88313645E+00, -0.19275559E+01, 
     &             0.22995307E+01,  0.39453590E+02, -0.20102814E+03, 
     &            -0.25678629E+00,  0.69233119E+00, -0.10624915E+01, 
     &             0.43102646E+01, -0.23367722E+02,  0.52318295E+02, 
     &            -0.21692032E+00,  0.59120220E+00, -0.67426765E+00, 
     &             0.51201826E+00, -0.70182557E+01,  0.46904053E+02, 
     &            -0.18254140E+00,  0.50964373E+00, -0.62824291E+00, 
     &             0.58964401E+00,  0.76392608E+01, -0.35213955E+02, 
     &            -0.15291579E+00,  0.44279686E+00, -0.42461100E+00, 
     &             0.11239138E+01, -0.33651037E+01,  0.35358219E+01, 
     &            -0.12667322E+00,  0.39987490E+00, -0.28411430E+00, 
     &             0.42075619E+00, -0.22601600E+01,  0.10132744E+02, 
     &            -0.10271295E+00,  0.36785719E+00, -0.23345713E+00, 
     &             0.25152662E+00,  0.90632361E+00, -0.15478793E+01, 
     &            -0.80560051E-01,  0.34238961E+00, -0.16883321E+00, 
     &             0.41764352E+00,  0.42261115E+00, -0.23799617E+01, 
     &            -0.59714071E-01,  0.32641086E+00, -0.86430468E-01, 
     &             0.43032908E+00, -0.32112655E+00, -0.83081322E+01, 
     &            -0.39558768E-01,  0.31970248E+00, -0.33553813E-01, 
     &             0.25510818E-01, -0.29174187E+01,  0.22803499E+02, 
     &            -0.19724984E-01,  0.31469795E+00, -0.41474979E-01, 
     &             0.18691723E+00,  0.42086711E+01, -0.24812429E+01, 
     &            -0.11086452E-03,  0.31562474E+00,  0.86154908E-01, 
     &             0.11421622E+01,  0.34332817E+01, -0.65490707E+02, 
     &             0.20220956E-01,  0.33813503E+00,  0.22088873E+00, 
     &            -0.55773860E+00, -0.17032518E+02,  0.10814697E+03, 
     &             0.41924343E-01,  0.35082772E+00, -0.18856723E-01, 
     &            -0.59137255E+00,  0.16763449E+02,  0.34940689E+02, 
     &             0.63922137E-01,  0.36057681E+00,  0.34845823E+00, 
     &             0.49643559E+01,  0.27682409E+02, -0.36143289E+03, 
     &             0.89109048E-01,  0.46176851E+00,  0.10456773E+01, 
     &            -0.22335031E+01, -0.85265312E+02,  0.43564413E+03, 
     &             0.12062342E+00,  0.51627433E+00, -0.30792767E+00, 
     &            -0.65324678E+01,  0.50873634E+02,  0.46494720E+03, 
     &             0.15131254E+00,  0.48638496E+00,  0.79470813E+00, 
     &             0.24347912E+02,  0.19616957E+03, -0.19854454E+04, 
     &             0.19186006E+00,  0.91114461E+00,  0.51103878E+01, 
     &            -0.41661053E+01, -0.42428186E+03,  0.18109329E+04, 
     &             0.26300505E+00,  0.12249467E+01, -0.11936491E+01, 
     &            -0.39496960E+02,  0.14163533E+03,  0.30391128E+04, 
     &             0.33031818E+00,  0.98306721E+00,  0.21399465E+01, 
     &             0.11462707E+03,  0.10913577E+04, -0.49505034E+04, 
     &             0.44003567E+00,  0.32819295E+01,  0.37125008E+02, 
     &             0.19408749E+03, -0.45567386E+03, -0.29708398E+03, 
     &             0.83032483E+00,  0.97293663E+01,  0.62111267E+02, 
     &             0.68564049E+02, -0.54851270E+03,  0.17552415E+04/
 


      data vfcoef/ 0.00000000E+00,  0.19490500E+00,  0.11604053E+00,
     &             0.00000000E+00,  0.00000000E+00,  0.17118695E+01,
     &             0.20667715E-01,  0.21896866E+00,  0.13315850E+00,
     &             0.17118214E+00,  0.85592270E+00, -0.28726175E+01,
     &             0.44123858E-01,  0.25272262E+00,  0.20714137E+00,
     &             0.22629277E+00, -0.58036578E+00,  0.77730355E+01,
     &             0.71713097E-01,  0.30250385E+00,  0.31793433E+00,
     &             0.77143145E+00,  0.33060975E+01, -0.76533756E+01,
     &             0.10616777E+00,  0.39862949E+00,  0.67119020E+00,
     &             0.13285358E+01, -0.52053666E+00,  0.69259363E+00,
     &             0.15402523E+00,  0.57098478E+00,  0.10454397E+01,
     &             0.11895815E+01, -0.17424472E+00,  0.56420404E+00,
     &             0.22275476E+00,  0.81534141E+00,  0.13974965E+01,
     &             0.11763034E+01,  0.10785336E+00,  0.51013916E+02,
     &             0.31995946E+00,  0.11560618E+01,  0.22669711E+01,
     &             0.63206925E+01,  0.25614452E+02, -0.11930303E+03,
     &             0.46592188E+00,  0.18418704E+01,  0.45069957E+01,
     &             0.46363611E+01, -0.34036224E+02,  0.23893833E+01,
     &             0.69643164E+00,  0.27473993E+01,  0.38796613E+01,
     &            -0.87390070E+01, -0.32841553E+02, -0.62542439E+01,
     &             0.99787807E+00,  0.31266701E+01, -0.77498108E+00,
     &            -0.22500851E+02, -0.35968632E+02,  0.29059869E+03,
     &             0.12796004E+01,  0.22980924E+01, -0.67773333E+01,
     &            -0.78290467E+01,  0.10932867E+03, -0.13840590E+03,
     &             0.14433545E+01,  0.10758772E+01, -0.39504786E+01,
     &             0.22061613E+02,  0.40126709E+02, -0.43857874E+03,
     &             0.15331247E+01,  0.88884521E+00,  0.68984365E+00,
     &            -0.57445703E+01, -0.17915956E+03,  0.77121613E+03,
     &             0.16129586E+01,  0.52345812E+00, -0.40709386E+01,
     &            -0.28794375E+00,  0.20644308E+03, -0.72966547E+03,
     &             0.16376544E+01,  0.16156906E+00,  0.93256897E+00,
     &             0.93236351E+01, -0.15838451E+03,  0.48042551E+03,
     &             0.16614262E+01,  0.23446929E+00, -0.96913165E+00,
     &            -0.59880857E+01,  0.81824844E+02, -0.22818460E+03,
     &             0.16750942E+01,  0.74207850E-01, -0.13792819E+00,
     &             0.39235618E+01, -0.32265957E+02,  0.86415871E+02,
     &             0.16826967E+01,  0.78473255E-01, -0.32656990E-01,
     &            -0.34129956E+00,  0.10941332E+02, -0.36170631E+02,
     &             0.16906085E+01,  0.87382466E-01,  0.15972510E+00,
     &             0.41820762E+00, -0.71437464E+01,  0.18469364E+02,
     &             0.17008324E+01,  0.11253329E+00,  0.41258883E-01,
     &            -0.59236693E+00,  0.20907981E+01, -0.50816641E+01,
     &             0.17120640E+01,  0.10883658E+00, -0.61818872E-01,
     &            -0.26421174E+00, -0.45000064E+00,  0.42816858E+01,
     &             0.17220629E+01,  0.88887662E-01, -0.12526552E+00,
     &            -0.16053502E-01,  0.16908103E+01, -0.70726466E+01,
     &             0.17297813E+01,  0.66580139E-01, -0.99359214E-01,
     &            -0.46984266E-01, -0.18454661E+01,  0.11912358E+02,
     &             0.17353332E+01,  0.43873392E-01, -0.10506071E+00,
     &             0.40604055E+00,  0.41106238E+01, -0.22952692E+02,
     &             0.17392575E+01,  0.40008530E-01,  0.33862963E-01,
     &            -0.24494012E+00, -0.73655701E+01,  0.38545902E+02,
     &             0.17430009E+01,  0.29243873E-01, -0.96097238E-01,
     &             0.66335094E+00,  0.11907093E+02, -0.58248142E+02,
     &             0.17462358E+01,  0.48428640E-01,  0.23485379E+00,
     &            -0.39853555E+00, -0.17216595E+02,  0.67698494E+02,
     &             0.17519840E+01,  0.48426915E-01, -0.24071532E+00,
     &            -0.51542318E+00,  0.16632149E+02, -0.47319916E+02,
     &             0.17550941E+01,  0.27689446E-01,  0.12938190E+00,
     &             0.14054815E+01, -0.70274992E+01,  0.14055183E+02/


      end
