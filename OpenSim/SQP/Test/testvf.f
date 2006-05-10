c testvf.f



c______________________________________________________________________________
c
c A simple routine to test vf02ad when called from C.
c
      program testvf

      implicit none

c     OPTIMIZATION VARIABLES
      integer NX
		parameter (NX=1)
      integer i,nc,nceq,status
      real*8 x(NX),p,c(1)
      real*8 dpdx(NX),dxda(NX)
      real*8 dcdx(NX+1,1),mu(1)
      real*8 alpha

c     PARAMOPT PARAMETERS
      integer lcn,iprint,inf,NWI,NWD
      parameter(NWI=100000,NWD=10000)
      integer wi(NWI)
      real*8 wd(NWD),eps,convergence

c     INITIALIZE
      nc = 0
      lcn = nx + 1
      iprint = 1
      eps = 1.0e-4
      status = 1

c     INITIAL X
      x(1) = 7.0d0

      do i=0,20

c       COMPUTE PERFORMANCE
        p = x(1)*x(1) + 1.0d0

c       COMPUTE DERIVATIVES
        dpdx(1) = 2.0*x(1)

c       PARAMETER OPTIMIZATION
        call paramopt(nx,nc,nceq,x,p,dpdx,c,dcdx,lcn,
     &   alpha,eps,convergence,status,wd,NWD,wi,NWI,dxda,mu)

c       PRINT
        print*,'iter=',i,'  p=',p,'dpdx=',dpdx(1),'dxda=',dxda(1)

c       CONVERGED?
        if(status.eq.1) then
          print*,"CONVERGED"
          call exit(0)
        elseif(inf.lt.0) then
          print*,"Error ",inf
          call exit(0)
        endif

c       LINE SEARCH
        if(x(1).ne.0.0) then
          alpha = 1.0d0
        else
          alpha = 0.0d0
        endif
        x(1) = x(1) - alpha

      enddo


      end
