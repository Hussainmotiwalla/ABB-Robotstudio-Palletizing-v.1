MODULE PmMain
  !***********************************************************
  !
  ! File: PmMain.mod
  !
  !
  ! Description
  !   This is a main module template for basic palletizing 
  !   with PickMaster.
  !   The template makes a one to one station with one flow.
  !
  ! Copyright (c) ABB Automation Technology Products 2006.
  ! All rights reserved
  !
  !***********************************************************

  ! Workarea descriptors used to access the workareas.
  VAR pm_wadescr waInFeeder;
  VAR pm_wadescr waOutFeeder;

  ! Flag indicating that next operation is done at
  ! the same work area as the previous operation.
  VAR bool MultiOperation:=FALSE;

  ! Constant for the maximum allowed tool angle.
  LOCAL CONST num MaxToolAngle:=360;
  LOCAL CONST num MinToolAngle:=-360;

  ! General path height offset for intermediate movements between work areas.
  ! The offset is added to the maximum height of passed work areas.
  CONST num PmSafetyOffsetZ:=100;

  VAR bool FirstMainLoop:=TRUE;

  !***********************************************************
  !
  ! Procedure main
  !
  !   This is the PickMaster MAIN routine.
  !   The project is started from here.
  !   First the robot moved to the home position
  !   Then the operate sequence will make the robot to the work.
  !
  !***********************************************************
  PROC Main()
    IF FirstMainLoop THEN
      MoveHomePos;
      FirstMainLoop:=FALSE;
    ENDIF
    CornerPathWarning FALSE;
    PmWaitProjStart;
    OperateSequence;
  ERROR
    IF ERRNO=PM_ERR_JOB_EMPTY THEN
      RETRY;
    ENDIF
  ENDPROC

  !***********************************************************
  !
  ! Procedure OperateSequence
  !
  !   Here the two Work Areas descriptors are fetched out of the flow.
  !   This routine executes the operation sequence.
  !
  !***********************************************************
  PROC OperateSequence()
    PmGetFlow waInFeeder,waOutFeeder;
    Operate waInFeeder;
    Operate waOutFeeder;
  ERROR
    TEST ERRNO
      CASE PM_ERR_PALLET_REDUCED:
        ! Number of remaining layers on pallet was updated after stack search.
        ! Operate the same work area again to access the new current layer.
        MultiOperation:=TRUE;
        RETRY;
      CASE PM_ERR_PALLET_EMPTY:
        ! The pallet stack was found empty during stack search.
        MultiOperation:=TRUE;
        RETRY;
      CASE PM_ERR_JOB_EMPTY:
        ! The pallet stack was found empty during stack search on master.
        MultiOperation:=FALSE;
        RAISE;
    ENDTEST
  ENDPROC

  !***********************************************************
  !
  ! Procedure Operate
  !
  !   This routine executes one operation. It pops one
  !   operation and execute all its targets and actions.
  !   Before moving towards every first target in the
  !   operation, the robot will go to a intermediate position.
  !
  ! Arguments:
  !   IN:
  !     pm_wadescr WorkArea
  !
  !***********************************************************
  PROC Operate(VAR pm_wadescr WorkArea)
    VAR pm_operationdata Op;
    VAR pm_targetdata Tgt;
    VAR pm_actiondata Act;
    VAR bool FirstTgtInOp:=TRUE;

    PmGetOperation WorkArea,Op;
    WHILE PmGetTarget(WorkArea\OpHandle:=Op.OpHandle,Tgt) DO
      WHILE PmGetTgtAction(WorkArea,Tgt.TargetHandle,Act) DO
        PmCalcArmConf Act.RobTgt,Tgt.TargetTool,Tgt.TargetWobj\cf6\MaxAngle:=MaxToolAngle\MinAngle:=MinToolAngle\SingAreaType:=Act.SingAreaType;
        IF FirstTgtInOp AND (NOT MultiOperation) THEN
          MoveInterMid WorkArea,Tgt,Act,PmSafetyOffsetZ\MaxAngle:=MaxToolAngle\MinAngle:=MinToolAngle;
          FirstTgtInOp:=FALSE;
        ENDIF
        Act.Speed.v_leax:=5000;
        Act.Speed.v_reax:=1000;
        PmDoAction WorkArea,Tgt,Act;
        SetLastPos WorkArea,Tgt,Act;
      ENDWHILE
    ENDWHILE
    MultiOperation:=FALSE;
  ERROR
    TEST ERRNO
    CASE PM_ERR_PALLET_REDUCED:
      RAISE;
    CASE PM_ERR_PALLET_EMPTY:
      RAISE;
    CASE PM_ERR_JOB_EMPTY:
      RAISE;
    ENDTEST
  ENDPROC
ENDMODULE