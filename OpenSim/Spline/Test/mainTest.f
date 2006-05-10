c==============================================================================
      program mainTest
c==============================================================================

      implicit none

      integer nstates,i,status
      real*8 t,value;
      integer constructSplineSuite
      real*8 spvalPos,spvalVel,spvalAcc
		character*9 suiteName

      data suiteName/'subave.sp'/

      external constructSplineSuite,destroySplineSuite
      external spvalPos,spvalVel,spvalAcc


c     INITIALIZE SUITE
		nstates = 23
      status = constructSplineSuite(suiteName,23)


c     EVALUATION
      t=0.0;
      do i = -1,24

         value = spvalPos(i,t);
         print*,'spPos',i+1,' = ',value

         value = spvalVel(i,t);
         print*,'spVel',i+1,' = ',value

         value = spvalAcc(i,t);
         print*,'spAcc',i+1,' = ',value

			print*

      enddo

c     DESTRUCTION
      call destroySplineSuite();

      end