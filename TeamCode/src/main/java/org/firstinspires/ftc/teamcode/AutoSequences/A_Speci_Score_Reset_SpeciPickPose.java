package org.firstinspires.ftc.teamcode.AutoSequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class A_Speci_Score_Reset_SpeciPickPose {
    public A_Speci_Score_Reset_SpeciPickPose(Outtake outtake, Intake intake){
        Actions.runBlocking( new SequentialAction(
//              outtake.sliderOutAction(Outtake.SliderStateOut.HIGH),
              new SleepAction(0.4),
              outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PLACE),
              outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PLACE),
              new SleepAction(0.5),
              outtake.gripperAction(Outtake.GripperState.OPEN),
              new SleepAction(0.5),
              new SequentialAction(
//                      outtake.sliderOutAction(Outtake.SliderStateOut.INIT),
//                      new SleepAction(0.5),
                      outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
                      outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK),
//                      outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
                      outtake.gripperAction(Outtake.GripperState.OPEN),// close
//                      intake.elbowAction(Intake.ElbowState.INIT),
//                      new SleepAction(1),
//                      outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
//                      outtake.gripperAction(Outtake.GripperState.OPEN),
                      new SleepAction(1),
                      outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
      )
      ));
    }

}
