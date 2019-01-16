!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
! File  sqopt_wrapper.f90 --- C/C++ wrapper for SQOPT
!
!+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

module sqopt_wrapper
  use  iso_c_binding
  implicit none

  public

  external :: &
       sqinit, sqspec, sqinitf, sqspecf, sqendf,      &
       sqmem,  snoptq, snkerq,                        &
       sqgeti, sqgetr, sqseti, sqsetr, sqset, &
       snFileRead, snFileOpenRead, &
       snFileOpenAppend, snFileClose

  public  :: &
       f_sqinit, f_sqinitf, f_sqspec, f_sqspecf, &
       f_sqsetprint, f_sqsetprintf, &
       f_sqmem,  f_sqopt,  f_snkerq, &
       f_sqset,  f_sqseti, f_sqsetr, &
       f_sqgeti, f_sqgetr, f_sqend

  !-----------------------------------------------------------------------------

  interface
     ! Interface for user-defined subroutines.
     subroutine sqLog &
         (Prob, ProbTag, Elastc, gotR, jstFea, feasbl, &
          m, mBS, nnH, nS, jSq, jBr, jSr,              &
          linesP, linesS, itn, itQP, kPrc, lvlInf,     &
          pivot, step, nInf, sInf, wtInf,              &
          ObjPrt, condHz, djqPrt, rgNorm, kBS, xBS,    &
          iw, leniw)
       character, intent(in) :: &
            ProbTag*20
       logical, intent(in) :: &
            Elastc, gotR, jstFea, feasbl
       integer, intent(in) :: &
            Prob, m, mBS, nnH, nS, jSq, jBr, jSr, itn, itQP, kPrc, &
            linesP, linesS, lvlInf, nInf, kBS(mBS), leniw
       double precision, intent(in) :: &
            condHz, djqPrt, ObjPrt, pivot, rgNorm, &
            step, sInf, wtInf, xBS(mBS)
       integer,       intent(inout) :: iw(leniw)
     end subroutine sqLog

     subroutine iusrHx &
          (nnH, x, Hx, nState, cu, lencu, iu, leniu, ru, lenru)
       integer,          intent(in)    :: nnH, nState, lencu, leniu, lenru
       double precision, intent(in)    :: x(nnH)
       character,        intent(inout) :: cu(lencu)*8
       integer,          intent(inout) :: iu(leniu)
       double precision, intent(inout) :: ru(lenru)
       double precision, intent(out)   :: Hx(nnH)
     end subroutine iusrHx
  end interface

  !-----------------------------------------------------------------------------

  ! Character arrays don't work well with Fortran/C/C++ so have a dummy one here.
  integer, parameter :: lencw = 500
  character*8        :: cw(lencw)

contains

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqinit &
      (printfile, plen, iPrint, summaryfile, slen, &
       iSumm, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqinit")

    integer(c_int),    intent(in), value :: &
         plen, slen, leniw, lenrw, iPrint, iSumm
    character(c_char), intent(in)        :: &
         printfile(plen), summaryfile(slen)
    integer(c_int),    intent(inout)     :: &
         iw(leniw)
    real(c_double),    intent(inout)     :: &
         rw(lenrw)

    !===========================================================================
    ! Call sqInit.  Pass provided file unit numbers.
    !===========================================================================
    character(plen) :: pfile
    character(slen) :: sfile
    integer         :: j, stat

    pfile  = ''
    if (iPrint /= 6) then
       if (plen > 0) then
          if (printfile(1) /= c_null_char) then
             do j = 1, plen
                if (printfile(j) == c_null_char) exit
                pfile(j:j) = printfile(j)
             end do
          end if
       end if
       call snFileOpenAppend(iPrint, trim(pfile), stat)
       if (stat /= 0) return
    end if

    sfile  = ''
    if (iSumm /= 6) then
       if (slen > 0) then
          if (summaryfile(1) /= c_null_char) then
             do j = 1, slen
                if (summaryfile(j) == c_null_char) exit
                sfile(j:j) = summaryfile(j)
             end do
          end if
       end if
       call snFileOpenAppend(iSumm,  trim(sfile), stat)
       if (stat /= 0) return
    end if

    call sqInit &
         (iPrint, iSumm, cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqinit

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqinitf &
       (name, len, summOn, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqinitf")

    integer(c_int),    intent(in), value :: len, summOn, leniw, lenrw
    character(c_char), intent(in)        :: name(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)

    !===========================================================================
    ! Call sqInit.  If a name is provided, use it as the print file name.
    ! Else assume no print file required (for now).
    !
    ! 07 Jul 2014: First version.
    !===========================================================================
    character(len) :: file
    character*6    :: screen
    integer        :: j, iPrt, iSum

    file  = ''
    if (len > 0) then
       if (name(1) /= c_null_char) then
          do j = 1, len
             if (name(j) == c_null_char) exit
             file(j:j) = name(j)
          end do
       end if
    end if

    if (summOn == 0) then
       screen = ''
    else
       screen = 'screen'
    end if

    call sqInitF &
         (trim(file), screen, iPrt, iSum, cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqinitf

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqsetprint &
       (name, len, iPrint, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqsetprint")

    integer(c_int),    intent(in), value :: len, iPrint, leniw, lenrw
    character(c_char), intent(in)        :: name(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)

    !===========================================================================
    ! Set print file name and unit.
    !===========================================================================
    integer        :: Errors, j, stat
    character(len) :: prtfile

    if (iPrint /= 6) then
       prtfile = ''
       do j = 1, len
          if (name(j) == c_null_char) exit
          prtfile(j:j) = name(j)
       end do
       call snFileOpenAppend(iPrint,trim(prtfile), stat)
       if (stat /= 0) return
    end if

    call sqSeti &
         ('Print file', iPrint, 0, 0, Errors, &
          cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqsetprint

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqsetprintf &
       (name, len, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqsetprintf")

    integer(c_int),    intent(in), value :: len, leniw, lenrw
    character(c_char), intent(in)        :: name(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)

    !===========================================================================
    ! Set print file name and unit.
    !===========================================================================
    integer        :: Errors, j, iPrt, stat
    character(len) :: prtfile

    prtfile = ''
    do j = 1, len
       if (name(j) == c_null_char) exit
       prtfile(j:j) = name(j)
    end do

    if (prtfile /= '') then
       call snFileAppend(iPrt,trim(prtfile),stat)
       if (stat /= 0) return

       call sqSeti('Print file', iPrt, 0, 0, Errors, &
                    cw, lencw, iw, leniw, rw, lenrw)
    end if

  end subroutine f_sqsetprintf

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqspec &
       (name, len, iSpecs, inform, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqspec")

    integer(c_int),    intent(in), value :: len, iSpecs, leniw, lenrw
    character(c_char), intent(in)        :: name(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)
    integer(c_int),    intent(out)       :: inform

    !===========================================================================
    ! Read options from the given specifications file.
    !===========================================================================
    integer        :: j, stat
    character(len) :: spcfile

    inform  = 0

    ! Get specs file name.
    spcfile = ''
    do j = 1, len
       if (name(j) == c_null_char) exit
       spcfile(j:j) = name(j)
    end do

    ! If we have a file, try to read it.
    if (spcfile /= '') then
       call snFileOpenRead(iSpecs,trim(spcfile),stat)
       if (stat /= 0) return
       call sqSpec(iSpecs, inform, cw, lencw, iw, leniw, rw, lenrw)
       close(iSpecs)
    end if

  end subroutine f_sqspec

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqspecf &
       (name, len, inform, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqspecf")

    integer(c_int),    intent(in), value :: len, leniw, lenrw
    character(c_char), intent(in)        :: name(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)
    integer(c_int),    intent(out)       :: inform

    !===========================================================================
    ! Read options from the given specifications file.
    !===========================================================================
    integer        :: j
    character(len) :: spcfile

    inform  = 0

    ! Get specs file name.
    spcfile = ''
    do j = 1, len
       if (name(j) == c_null_char) exit
       spcfile(j:j) = name(j)
    end do

    ! If we have a file, try to read it.
    if (spcfile /= '') then
       call sqSpecF(trim(spcfile), inform, cw, lencw, iw, leniw, rw, lenrw)
    end if

  end subroutine f_sqspecf

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqmem &
      (info, m, n, neA, ncObj, nnH, &
       miniw, minrw, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqmem")

    integer(c_int), intent(in), value :: &
         m, n, neA, ncObj, nnH, leniw, lenrw
    integer(c_int), intent(inout)     :: iw(leniw)
    real(c_double), intent(inout)     :: rw(lenrw)

    integer(c_int), intent(out)       :: INFO, miniw, minrw

    !===========================================================================
    ! Estimate workspace for SQOPT.
    !===========================================================================
    integer :: mincw

    call sqMem &
        (INFO, m, n, neA, ncObj, nnH, &
         mincw, miniw, minrw, &
         cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqmem

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqopt &
      (Start, c_qpHx, m, n, neA, ncObj, nnH, &
       iObj, ObjAdd, Prob,                   &
       valA, indA, locA, bl, bu, cObj,       &
       eType, hs, x, pi, rc,                 &
       INFO, nS, nInf, sInf, Obj,            &
       miniw, minrw,                         &
       iu, leniu, ru, lenru,                 &
       iw, leniw, rw, lenrw) bind(C,name='f_sqopt')
    integer(c_int), intent(in), value :: &
         Start, m, n, iObj, neA, ncObj, nnH, &
         leniu, lenru, leniw, lenrw
    real(c_double), intent(in), value :: ObjAdd

    integer(c_int),    intent(in) :: eType(n+m), indA(neA), locA(n+1)
    real(c_double),    intent(in) :: cObj(ncObj), bl(n+m), bu(n+m), valA(neA)
    character(c_char), intent(in) :: Prob(*)

    integer(c_int), intent(inout) :: nInf, nS, hs(n)
    real(c_double), intent(inout) :: sInf, x(n), pi(m), rc(n+m)

    integer(c_int), intent(inout) :: iw(leniw), iu(leniu)
    real(c_double), intent(inout) :: rw(lenrw), ru(lenru)

    integer(c_int), intent(out)   :: INFO, miniw, minrw
    real(c_double), intent(out)   :: Obj
    type(c_funptr), value         :: c_qpHx

    !===========================================================================
    ! Solve the problem with SQOPT.
    !===========================================================================
    integer      :: j, nNames, mincw
    character(8) :: pname, Names(1), Fnames(1)
    character(4) :: nStart

    procedure(iusrHx), pointer :: qpHx

    nNames = 1

        if      (Start == 1) then
       nStart = 'Warm'
    else if (Start == 2) then
       nStart = 'Hot'
    else
       nStart = 'Cold'
    end if

    call c_f_procpointer(c_qpHx, qpHx)

    pname  = ''
    do j = 1, 8
       if (Prob(j) == c_null_char) exit
       pname(j:j) = Prob(j)
    end do

    call sqOpt &
        (nStart, qpHx, m, n, neA, nNames,       &
         ncObj, nnH, iObj, ObjAdd, pname,       &
         valA, indA, locA, bl, bu, cObj, Names, &
         eType, hs, x, pi, rc,                  &
         INFO, mincw, miniw, minrw,             &
         nS, nInf, sInf, Obj,                   &
         cw, lencw, iu, leniu, ru, lenru,       &
         cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqopt

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_snkerq &
      (Start, c_qpHx, c_sqLog,         &
       m, n, neA, ncObj, nnH,          &
       iObj, ObjAdd, Prob,             &
       valA, indA, locA, bl, bu, cObj, &
       eType, hs, x, pi, rc,           &
       INFO, nS, nInf, sInf, Obj,      &
       miniw, minrw,                   &
       iu, leniu, ru, lenru,           &
       iw, leniw, rw, lenrw) bind(C,name='f_snkerq')
    integer(c_int), intent(in), value :: &
         Start, m, n, iObj, neA, ncObj, nnH, &
         leniu, lenru, leniw, lenrw
    real(c_double), intent(in), value :: ObjAdd

    integer(c_int),    intent(in) :: eType(n+m), indA(neA), locA(n+1)
    real(c_double),    intent(in) :: cObj(ncObj), bl(n+m), bu(n+m), valA(neA)
    character(c_char), intent(in) :: Prob(*)

    integer(c_int), intent(inout) :: nInf, nS, hs(n)
    real(c_double), intent(inout) :: sInf, x(n), pi(m), rc(n+m)

    integer(c_int), intent(inout) :: iw(leniw), iu(leniu)
    real(c_double), intent(inout) :: rw(lenrw), ru(lenru)

    integer(c_int), intent(out)   :: INFO, miniw, minrw
    real(c_double), intent(out)   :: Obj
    type(c_funptr), value         :: c_qpHx, c_sqLog

    !===========================================================================
    ! Solve the problem with SQOPT.
    !===========================================================================
    integer      :: j, nNames, mincw
    character(8) :: pname, Names(1), Fnames(1)
    character(4) :: nStart

    procedure(iusrHx), pointer :: qpHx
    procedure(sqLog),  pointer :: myLogQ

    nNames = 1

    if      (Start == 1) then
       nStart = 'Warm'
    else if (Start == 2) then
       nStart = 'Hot'
    else
       nStart = 'Cold'
    end if

    myLogQ => null()

    call c_f_procpointer(c_qpHx, qpHx)
    call c_f_procpointer(c_sqLog, myLogQ)

    if (.not. associated(myLogQ)) myLogQ => sqLog

    pname  = ''
    do j = 1, 8
       if (Prob(j) == c_null_char) exit
       pname(j:j) = Prob(j)
    end do

    call snKerQ &
        (nStart, qpHx, myLogQ,                  &
         m, n, neA, nNames,                     &
         ncObj, nnH, iObj, ObjAdd, pname,       &
         valA, indA, locA, bl, bu, cObj, Names, &
         eType, hs, x, pi, rc,                  &
         INFO, mincw, miniw, minrw,             &
         nS, nInf, sInf, Obj,                   &
         cw, lencw, iu, leniu, ru, lenru,       &
         cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_snkerq

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqend(iw, leniw, rw, lenrw) bind(C,name="f_sqend")
    integer(c_int),    intent(in), value :: leniw, lenrw
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)

    !===========================================================================
    ! Finish up.
    !===========================================================================
    call sqEndF( cw, lencw, iw, leniw, rw, lenrw )

  end subroutine f_sqend

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqset &
       (option, len, Errors, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqset")
    integer(c_int),    intent(in), value :: len, leniw, lenrw
    character(c_char), intent(in)        :: option(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)
    integer(c_int),    intent(out)       :: Errors

    !===========================================================================
    ! Set option via string.
    !===========================================================================
    character(len) :: buffer
    integer        :: j

    errors = 0
    buffer = ''
    do j = 1, len
       if (option(j) == c_null_char) exit
       buffer(j:j) = option(j)
    end do

    call sqSet(buffer, 0, 0, Errors, cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqset

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqseti &
       (option, len, ivalue, Errors, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqseti")
    integer(c_int),    intent(in), value :: len, ivalue, leniw, lenrw
    character(c_char), intent(in)        :: option(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)
    integer(c_int),    intent(out)       :: Errors

    !===========================================================================
    ! Set option with integer value.
    !===========================================================================
    character(len) :: buffer
    integer        :: j

    errors = 0
    buffer = ''
    do j = 1, len
       if (option(j) == c_null_char) exit
       buffer(j:j) = option(j)
    end do

    call sqSeti &
         (buffer, ivalue, 0, 0, Errors, cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqseti

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqsetr &
       (option, len, rvalue, Errors, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqsetr")
    integer(c_int),    intent(in), value :: len, leniw, lenrw
    real(c_double),    intent(in), value :: rvalue
    character(c_char), intent(in)        :: option(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)
    integer(c_int),    intent(out)       :: Errors

    !===========================================================================
    ! Set option with real value.
    !===========================================================================
    character(len) :: buffer
    integer        :: j

    errors = 0
    buffer = ''

    do j = 1, len
       if (option(j) == c_null_char) exit
       buffer(j:j) = option(j)
    end do

    call sqSetr &
         (buffer, rvalue, 0, 0, Errors, cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqsetr

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqgeti &
       (option, len, ivalue, Errors, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqgeti")
    integer(c_int),    intent(in), value :: len, leniw, lenrw
    character(c_char), intent(in)        :: option(len)
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)
    integer(c_int),    intent(out)       :: ivalue, Errors

    !===========================================================================
    ! Get integer option value.
    !===========================================================================
    character(len) :: buffer
    integer        :: j

    errors = 0
    buffer = ''
    do j = 1, len
       if (option(j) == c_null_char) exit
       buffer(j:j) = option(j)
    end do

    call sqGetI(buffer, ivalue, Errors, cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqgeti

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  subroutine f_sqgetr &
       (option, len, rvalue, Errors, iw, leniw, rw, lenrw) &
       bind(C,name="f_sqgetr")
    integer(c_int),    intent(in), value :: len, leniw, lenrw
    character(c_char), intent(in)        :: option(len)
    real(c_double),    intent(out)       :: rvalue
    integer(c_int),    intent(inout)     :: iw(leniw)
    real(c_double),    intent(inout)     :: rw(lenrw)
    integer(c_int),    intent(out)       :: Errors

    !===========================================================================
    ! Get real option value.
    !===========================================================================
    character(len) :: buffer
    integer        :: j

    errors = 0
    buffer = ''
    do j = 1, len
       if (option(j) == c_null_char) exit
       buffer(j:j) = option(j)
    end do

    call sqGetR(buffer, rvalue, Errors, cw, lencw, iw, leniw, rw, lenrw)

  end subroutine f_sqgetr

  !+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

end module sqopt_wrapper
