//package org.firstinspires.ftc.teamcode.Sequences;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//
//import org.firstinspires.ftc.teamcode.Subsystems.GripperIntake;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
//
//public class SampleIntakeSequence {
//
//    public static Action BeforePickSequence(Intake intake) {
//        return new SequentialAction(
////                intake.rollerAction(Intake.RollerState.OFF),
////                intake.elbowAction(Intake.ElbowState.BEFORE_PICK), // before pick
////                intake.wristAction(Intake.WristState.BEFORE_PICK)
//        );
//    }
//
//    public static Action IntakeSequenceAction(Intake intake) {
//        return new SequentialAction(
////                intake.elbowAction(Intake.ElbowState.PICK),
////                intake.wristAction(Intake.WristState.PICK),
////                intake.rollerAction(Intake.RollerState.ON)
//                );
//    }
//
//    public static Action AfterIntakeSequenceAction(Intake intake){
//        return new SequentialAction(
////                intake.rollerAction(Intake.RollerState.OFF),
////                intake.flapperAction(Intake.FlapperState.INIT),
////                new SleepAction(0.2),
////                intake.elbowAction(Intake.ElbowState.AFTER_PICK),
////                intake.wristAction(Intake.WristState.AFTER_PICK),
////                new SleepAction(1),
////                intake.sliderAction(Intake.SliderState.INIT)
//
//        );
//    }
//
//    public static Action PlaceSequenceAction(Intake intake){
//        return new SequentialAction(
////                intake.elbowAction(Intake.ElbowState.PLACE),
////                intake.wristAction(Intake.WristState.PLACE),
////                intake.yawAction(Intake.YawState.INIT),
////                new SleepAction(1),
////                intake.flapperAction(Intake.FlapperState.HOLD_OPEN),
////                intake.rollerAction(Intake.RollerState.ON)
//        );
//    }
//
//    public static Action GripperIntakePickAction(GripperIntake intake, Outtake outtake){
//        return new SequentialAction(
//                intake.wristAction(GripperIntake.WristState.PICK),
//                intake.gripperAction(GripperIntake.GripperState.OPEN),
//                intake.shoulderAction(GripperIntake.ShoulderState.PICK),
//                intake.elbowAction(GripperIntake.ElbowState.PRE_PICK),
//                new SleepAction(0.2),
//                intake.elbowAction(GripperIntake.ElbowState.PICK),
//                intake.yawAction(GripperIntake.YawStateNew.PICK)
//        );
//    }
//    public static Action GripperIntakePostPickAction(GripperIntake intake, Outtake outtake){
//        return new SequentialAction(
//                intake.wristAction(GripperIntake.WristState.POST_PICK),
//                intake.shoulderAction(GripperIntake.ShoulderState.POST_PICK),
//                intake.elbowAction(GripperIntake.ElbowState.PRE_PICK),
//                new SleepAction(0.2),
//                intake.elbowAction(GripperIntake.ElbowState.POST_PICK),
//                intake.yawAction(GripperIntake.YawStateNew.POST_PICK)
//        );
//    }
//
//}
