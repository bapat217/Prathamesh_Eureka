package org.firstinspires.ftc.teamcode.Sequences;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class ScoreAndResetSequence {
    public static Action ScoreAndResetSequenceAction(Outtake outtake, Intake intake){
      return new SequentialAction(
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
      );
    }public static Action N_ScoreAndResetSequenceAction(Outtake outtake, Intake intake){
      return new SequentialAction(
//              outtake.sliderOutAction(Outtake.SliderStateOut.HIGH),
              outtake.twistAction(Outtake.TwistState.TWIST_180),
              new SleepAction(0.4),
              outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PLACE),
              outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PLACE),
              new SleepAction(0.5),
              outtake.gripperAction(Outtake.GripperState.OPEN),
              new SleepAction(0.5),
              new SequentialAction(

                      outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
                      outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK),

                      outtake.gripperAction(Outtake.GripperState.OPEN),// close

                      new SleepAction(1),
                      outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
      )
      );
    }

    public static Action readyToSpecimenPick(Outtake outtake, Intake intake){
        return new SequentialAction(
                new SequentialAction(
//                      outtake.sliderOutAction(Outtake.SliderStateOut.INIT),
//                      new SleepAction(0.5),
                        outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK),
//                      outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
                        outtake.gripperAction(Outtake.GripperState.OPEN),// close
//                      intake.elbowAction(Intake.ElbowState.INIT),
//                      new SleepAction(1),
                        outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
//                      outtake.gripperAction(Outtake.GripperState.OPEN),
                        new SleepAction(0.5),
                        outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
                )
        );
    }public static Action N_readyToSpecimenPick(Outtake outtake, Intake intake){
        return new SequentialAction(
                new SequentialAction(

                        outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK),

                        outtake.gripperAction(Outtake.GripperState.OPEN),// close

                        outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),

                        new SleepAction(0.5),
                        outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
                )
        );
    }
    public static Action ScoreAndResetSequenceAction(Outtake outtake){
      return new SequentialAction(
//              outtake.sliderOutAction(Outtake.SliderStateOut.SCORE),
              outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PLACE),
              outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PLACE),
              new SleepAction(0.5),
              outtake.gripperAction(Outtake.GripperState.OPEN),
              new SleepAction(0.5)
//              new SequentialAction(
////                      outtake.sliderOutAction(Outtake.SliderStateOut.INIT),
////                      new SleepAction(0.5),
//                      outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK),
////                      outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
//                      outtake.gripperAction(Outtake.GripperState.OPEN),// close
////                      intake.elbowAction(Intake.ElbowState.INIT),
////                      new SleepAction(1),
//                      outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
////                      outtake.gripperAction(Outtake.GripperState.OPEN),
//                      new SleepAction(0.5),
//                      outtake.sliderOutAction(Outtake.SliderStateOut.INIT)

      );
    }

    public static Action BeforeScoreBucketSequence(Outtake outtake){
        return new SequentialAction(
                outtake.wristOutAction(Outtake.WristStateOut.BEFORE_BUCKET),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
                outtake.sliderOutAction(Outtake.SliderStateOut.BUCKET_SCORE)
//                new SleepAction( 2),
//                new InstantAction(()->robot.verticalSliderRight.setPower(0))

                );
    }
    public static Action ScoreBucketAndReset(Intake intake, Outtake outtake){
        return new SequentialAction(
                outtake.wristOutAction(Outtake.WristStateOut.BUCKET_SCORE),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BUCKET_SCORE),
                outtake.gripperAction(Outtake.GripperState.OPEN),
                new SleepAction(0.5),
                outtake.wristOutAction(Outtake.WristStateOut.BEFORE_BUCKET),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
                outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
        );
    }
    public static Action ScoreBucketAndReset(Outtake outtake){
        return new SequentialAction(
                outtake.wristOutAction(Outtake.WristStateOut.BUCKET_SCORE),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BUCKET_SCORE),
                outtake.gripperAction(Outtake.GripperState.OPEN),
                new SleepAction(0.5),
                outtake.wristOutAction(Outtake.WristStateOut.BEFORE_BUCKET),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET)
        );
    }
    public static Action ScoreBucketAndReset(Outtake outtake,Intake intake){
        return new SequentialAction(
//                outtake.elbowOutAction(Outtake.ElbowStateOut.INBETWEEN),
//                new SleepAction(0.1),
                outtake.wristOutAction(Outtake.WristStateOut.BUCKET_SCORE),
//                outtake.elbowOutAction(Outtake.ElbowStateOut.BUCKET_SCORE),
                outtake.gripperAction(Outtake.GripperState.OPEN),
                new SleepAction(0.5),
                outtake.wristOutAction(Outtake.WristStateOut.BEFORE_BUCKET),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
                new SleepAction( 0.6),
                outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
        );
    }
}
