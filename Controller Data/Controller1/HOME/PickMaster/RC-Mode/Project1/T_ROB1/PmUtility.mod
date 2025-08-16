MODULE PmUtility
  !***********************************************************
  !
  ! File: PmUtility.mod
  !
  ! Description
  !   This is a utility module for palletizing with PickMaster.
  !   The offers intermediate position calculation and 
  !   HomePos movement.
  !
  ! Copyright (c) ABB Automation Technology Products 2006.
  ! All rights reserved
  !
  !***********************************************************

  ! A home position to go to.
  LOCAL PERS jointtarget HomePos:=[[0,0,0,0,90,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

  LOCAL PERS tooldata TempTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
  LOCAL PERS wobjdata TempWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

  !***********************************************************
  !
  ! Procedure MoveHomePos
  !
  !   This routine moves the robot to a predefined home 
  !   position and stores data for next intermediate position 
  !   calculation.
  !
  !***********************************************************
  PROC MoveHomePos()
    CONST num RetractDist:=50;
    VAR pm_targetdata Tgt;
    VAR pm_actiondata Act;
    VAR pm_wadescr HomeWorkArea;

    ! Get the weight from current tool and frame from tool0
    PmLastTool:=CTool();
    PmLastTool.tframe:=tool0.tframe;
    PmLastWobj:=wobj0;
    PmLastRobTgt:=CRobT(\Tool:=PmLastTool\Wobj:=PmLastWobj);

    Act.Speed:=v500;
    Act.Speed.v_leax:=5000;
    Act.Speed.v_reax:=1000;
    Act.Accel.AccLim:=FALSE;
    Act.Accel.AccMax:=100;
    Act.Accel.DecelLim:=FALSE;
    Act.Accel.DecelMax:=100;
    Act.Accel.Acc:=100;
    Act.Accel.Ramp:=100;
    Act.RobTgt:=CalcRobT(HomePos,tool0);
    Tgt.TargetTool:=PmLastTool;
    Tgt.TargetWobj:=pm_home_WObj;

    PmLastRobTgt.trans.z:=PmLastRobTgt.trans.z+RetractDist;
    MoveL PmLastRobTgt,Act.Speed,fine,PmLastTool\Wobj:=PmLastWobj;

    PmWaitProjStart;
    PmGetWaByWobj pm_home_WObj,HomeWorkArea;
    MoveInterMid HomeWorkArea,Tgt,Act,PmSafetyOffsetZ\MoveToEndPoint;

    PmLastRobTgt:=Act.RobTgt;
    PmLastWobj:=Tgt.TargetWobj;
    PmLastTool:=Tgt.TargetTool;
    PmSetLastWa HomeWorkArea;
  ENDPROC

  !***********************************************************
  !
  ! Procedure SetLastPos
  !
  !   This routine stores data for next intermediate position
  !   calculation.
  !
  ! Arguments:
  !   IN:
  !     pm_wadescr WorkArea
  !     pm_targetdata Tgt
  !     pm_actiondata Act
  !
  !***********************************************************
  PROC SetLastPos(VAR pm_wadescr WorkArea,VAR pm_targetdata Tgt,VAR pm_actiondata Act)
    VAR robtarget temp;

    temp:=PmLastRobTgt;
    PmLastRobTgt:=Act.RobTgt;
    PmLastWobj:=Tgt.TargetWobj;
    PmLastTool:=Tgt.TargetTool;
    PmSetLastWa WorkArea;
    IF Act.ArmConfMon=FALSE THEN
      PmLastRobTgt.robconf:=temp.robconf;
    ENDIF
  ENDPROC

  !***********************************************************
  !
  ! Procedure MoveInterMid
  !
  !   This routine calculates an intermediate position out of
  !   last stored position, the target argument and the next 
  !   action. The move will be executed as a joint movement.
  !
  ! Arguments:
  !   IN:
  !     pm_wadescr WorkArea
  !     pm_targetdata Tgt
  !     pm_actiondata Act
  !     num SafetyOffsetZ
  !   optional IN:
  !     num MaxToolAngle
  !     num MinToolAngle
  !
  !***********************************************************
  PROC MoveInterMid(VAR pm_wadescr WorkArea,VAR pm_targetdata Tgt,VAR pm_actiondata Act,num SafetyOffsetZ,\num MaxAngle,\num MinAngle,\switch MoveToEndPoint)
    CONST num IntermidPart1:=0.1;
    CONST num IntermidPart2:=0.5;
    CONST num IntermidPart3:=0.9;
    VAR robtarget InterMid1;
    VAR robtarget InterMid2;
    VAR robtarget InterMid3;
    VAR jointtarget FromJointTgt;
    VAR jointtarget ToJointTgt;
    VAR robtarget FromRobTgt;
    VAR robtarget ToRobTgt;
    VAR num MinZ;
    VAR pm_wadescr LastWorkArea;

    PmGetLastWa LastWorkArea;

    ! Calculate MinZ. The z value of the tool and product is not considered in the calculation of min z in PmCalcIntermid.
    IF Tgt.NumOfAppProds=0 THEN
      ! MinZ without product in tool
      MinZ:=PmGetPathHeight(LastWorkArea,WorkArea\UseSafePosition)+Tgt.GripLenEmptyZ+SafetyOffsetZ;
    ELSE
      ! MinZ with product in tool
      MinZ:=PmGetPathHeight(LastWorkArea,WorkArea\UseSafePosition)+Tgt.GripLenLoadedZ+SafetyOffsetZ;
    ENDIF

    ! Check z value also for the start and end target.
    ! Use tool and workobject from target tool.
    TempTool:=Tgt.TargetTool;
    TempWobj:=Tgt.TargetWobj;
    ! Set start and end target values.
    FromJointTgt:=CalcJointT(PmLastRobTgt,PmLastTool\Wobj:=PmLastWobj);
    ToJointTgt:=CalcJointT(Act.RobTgt,TempTool\Wobj:=TempWobj);
    FromRobTgt:=CalcRobT(FromJointTgt,tool0);
    ToRobTgt:=CalcRobT(ToJointTgt,tool0);
    ! Compare z value for start and end targets, set a new higher value for z if needed.    
    IF (FromRobTgt.trans.z<ToRobTgt.trans.z) AND (MinZ<ToRobTgt.trans.z) THEN
      MinZ:=ToRobTgt.trans.z;
    ELSEIF (ToRobTgt.trans.z<FromRobTgt.trans.z) AND (MinZ<FromRobTgt.trans.z) THEN
      MinZ:=FromRobTgt.trans.z;
    ENDIF

    ConfJ\On;

    ! Use the frame from tool0 and the load from target tool
    TempTool:=Tgt.TargetTool;
    TempTool.tframe:=tool0.tframe;

    ! Set Acceleration
    PathAccLim Act.Accel.AccLim\AccMax:=Act.Accel.AccMax,Act.Accel.DecelLim\DecelMax:=Act.Accel.DecelMax;
    AccSet Act.Accel.Acc,Act.Accel.Ramp;

    ! Using PmDoMove3 instead of MoveJ.
    ! PmDoMove3 will automatically avoid using too many consecutive concurrent (\Conc) movements.
    ! Travel distance: 10%    
    InterMid1:=PmCalcIntermid(PmLastRobTgt,PmLastTool,PmLastWobj,Act.RobTgt,Tgt.TargetTool,Tgt.TargetWobj,IntermidPart1\MaxAngle?MaxAngle\MinAngle?MinAngle\AngleLimAx6\MinZ:=MinZ\FromWa:=LastWorkArea\ToWa:=WorkArea);
    PmDoMove3 PM_MOVE_JOINT\Conc,InterMid1,Act.Speed,z200,TempTool,wobj0;

    ! Calculate intermediate targets two and three
    InterMid2:=PmCalcIntermid(PmLastRobTgt,PmLastTool,PmLastWobj,Act.RobTgt,Tgt.TargetTool,Tgt.TargetWobj,IntermidPart2\MaxAngle?MaxAngle\MinAngle?MinAngle\AngleLimAx6\MinZ:=MinZ\FromWa:=LastWorkArea\ToWa:=WorkArea);
    InterMid3:=PmCalcIntermid(PmLastRobTgt,PmLastTool,PmLastWobj,Act.RobTgt,Tgt.TargetTool,Tgt.TargetWobj,IntermidPart3\MaxAngle?MaxAngle\MinAngle?MinAngle\AngleLimAx6\MinZ:=MinZ\FromWa:=LastWorkArea\ToWa:=WorkArea);

    ! Travel distance: 50%
    PmDoMove3 PM_MOVE_JOINT\Conc,InterMid2,Act.Speed,z200,TempTool,wobj0;

    ! Travel distance: 90%
    IF Present(MoveToEndPoint) THEN
      PmDoMove3 PM_MOVE_JOINT\Conc,Act.RobTgt,Act.Speed,fine,TempTool,wobj0;
    ELSE
      PmDoMove3 PM_MOVE_JOINT\Conc,InterMid3,Act.Speed,z200,TempTool,wobj0;
    ENDIF
  ENDPROC
ENDMODULE