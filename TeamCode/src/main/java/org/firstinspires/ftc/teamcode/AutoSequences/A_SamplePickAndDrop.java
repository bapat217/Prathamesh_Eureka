//package org.firstinspires.ftc.teamcode.AutoSequences;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
//
//public class A_SamplePickAndDrop {
//
//    public static Action SEQ_1(Intake intake, Outtake outtake){
//        return new SequentialAction(
//
//                intake.iXextension(Intake.iXextensionStates.MID),
//                new SleepAction(0.3),
//                intake.iGripper(Intake.iGripperStates.OPEN),
//                intake.iElbow(Intake.iElobowStates.BEFORE_PICK),
//                intake.iShoulde(Intake.iShoulderStates.BEFORE_PICK),
//                intake.iWiper(Intake.iWiperStates.BEFORE_PICK),
//
//                new SleepAction(1),
//                // picked
//                intake.iGripper(Intake.iGripperStates.CLOSE),
//                new SleepAction(0.2),
//                intake.iElbow(Intake.iElobowStates.TRANSFER),
//                intake.iShoulde(Intake.iShoulderStates.TRANSFER),// PICK
//                intake.iWrist(Intake.iWristStates.POSE_90), // try
//                new SleepAction(0.1),
//                intake.iWiper(Intake.iWiperStates.AFTER_PICK),
//                intake.iXextension(Intake.iXextensionStates.INIT),
//
//                intake.iGripper(Intake.iGripperStates.CLOSE),
//                new SleepAction(0.2),
//                intake.iShoulde(Intake.iShoulderStates.TRANSFER),
//                intake.iWrist(Intake.iWristStates.POSE_90),
//                intake.iElbow(Intake.iElobowStates.AFTER_PICK),
//                intake.iElbow(Intake.iElobowStates.TRANSFER),
//                intake.iWiper(Intake.iWiperStates.TRANSFER)
////                intake.iXextension(Intake.iXextensionStates.INIT)
//
//        );
//    }
//}
