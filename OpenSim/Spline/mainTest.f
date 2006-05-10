c==============================================================================
      program mainTest
c==============================================================================

      implicit none

      integer nstates,i,status
      real*8 t,value
      integer constructSplineSuite
      real*8 sppos,spvel,spacc
		character*14 suiteName

      data suiteName/'TEST/subave.sp'/

      external constructSplineSuite,destroySplineSuite
      external sppos,spvel,spacc


c     INITIALIZE SUITE
		nstates = 23
      status = constructSplineSuite(suiteName,23)


c     EVALUATION
      t=0.0
      do i = -1,24

         value = sppos(i,t)
         print*,'spPos',i+1,' = ',value

         value = spvel(i,t)
         print*,'spVel',i+1,' = ',value

         value = spacc(i,t)
         print*,'spAcc',i+1,' = ',value

			print*

      enddo

c     DESTRUCTION
      call destroySplineSuite()

      end
