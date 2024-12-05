package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_Sample_Transfer_G {
    public A_Sample_Transfer_G (Intake intake, Outtake outtake, Intake.iGripperStates state){
        Actions.runBlocking(new SequentialAction(

                // Pre pick pose

                intake.iGripper(state),
                new SleepAction(0.2),
                intake.iShoulde(Intake.iShoulderStates.TRANSFER),
                intake.iWrist(Intake.iWristStates.POSE_90),
                intake.iElbow(Intake.iElobowStates.AFTER_PICK),
                intake.iElbow(Intake.iElobowStates.TRANSFER),
                intake.iWiper(Intake.iWiperStates.TRANSFER),
                intake.iXextension(Intake.iXextensionStates.INIT)

        ));
    }

//// Transfer Pose
//    public static Action SampleTransferGClosed(Intake intake, Outtake outtake){
//        return new SequentialAction(
//
//                // Pre pick pose
//
//                intake.iGripper(Intake.iGripperStates.CLOSE),
//                new SleepAction(0.2),
//                intake.iShoulde(Intake.iShoulderStates.TRANSFER),
//                intake.iWrist(Intake.iWristStates.POSE_90),
//                intake.iElbow(Intake.iElobowStates.AFTER_PICK),
//                intake.iElbow(Intake.iElobowStates.TRANSFER),
//                intake.iWiper(Intake.iWiperStates.TRANSFER),
//                intake.iXextension(Intake.iXextensionStates.INIT)
//
//        );
//    }
//// Transfer Pose gripper open (intake system)
//    public static Action SampleTransferGOpen(Intake intake, Outtake outtake){
//        return new SequentialAction(
//
//                intake.iGripper(Intake.iGripperStates.OPEN),
//                intake.iWrist(Intake.iWristStates.POSE_90),
//                intake.iElbow(Intake.iElobowStates.TRANSFER),
//                intake.iShoulde(Intake.iShoulderStates.TRANSFER),
//                intake.iWiper(Intake.iWiperStates.TRANSFER)
////                intake.iXextension(Intake.iXextensionStates.INIT)
//
//        );
//    }

}
