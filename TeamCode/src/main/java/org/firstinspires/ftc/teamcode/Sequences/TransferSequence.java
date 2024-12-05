package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

//import org.firstinspires.ftc.teamcode.Subsystems.GripperIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class TransferSequence {

//    public Action TransferSequenceAction(Intake intake, Outtake outtake){
//        return new SequentialAction(
//          intake.
//        );
//    }

    //TODO INTAKE ----------------------------------------------------------------

    public static Action TransferIntakeAction(Intake intake){
        return new SequentialAction(
//                intake.elbowAction(Intake.ElbowState.TRANSFER),
//                intake.sliderAction(Intake.SliderState.INIT),
//                new SleepAction(1),
//                intake.wristAction(Intake.WristState.TRANSFER)
        );
    }
//
//    public static Action GripperTransferIntakeAction(GripperIntake intake) {
//        return new SequentialAction(
//                intake.elbowAction(GripperIntake.ElbowState.TRANSFER),
//                intake.shoulderAction(GripperIntake.ShoulderState.TRANSFER),
//                intake.wristAction(GripperIntake.WristState.TRANSFER)
//        );
//    }
//    public static Action GripperAfterTransferIntakeAction(GripperIntake intake) {
//
//        return new SequentialAction(
//                intake.gripperAction(GripperIntake.GripperState.TRANSFER),
//                new SleepAction(0.5),
//                intake.shoulderAction(GripperIntake.ShoulderState.AFTER_TRANSFER)
//        );
//    }
    //TODO OUTTAKE ----------------------------------------------------------------

    public static Action TransferOuttakeAction(Outtake outtake, Intake intake){
        return new SequentialAction(
                outtake.gripperAction(Outtake.GripperState.OPEN),
                outtake.wristOutAction(Outtake.WristStateOut.TRANSFER),
//
                outtake.elbowOutAction(Outtake.ElbowStateOut.TRANSFER)
        );
    }
    public static Action TransferOuttakeActionGclose(Outtake outtake, Intake intake){
        return new SequentialAction(
                outtake.gripperAction(Outtake.GripperState.CLOSE),
                outtake.wristOutAction(Outtake.WristStateOut.TRANSFER),
//                intake.flapperAction(Intake.FlapperState.INIT),
                new SleepAction(0.5),
                outtake.elbowOutAction(Outtake.ElbowStateOut.TRANSFER)
        );
    }
    public static Action GripperTransferOuttakeAction(Outtake outtake) {
        return new SequentialAction(
            outtake.elbowOutAction(Outtake.ElbowStateOut.GRIPPER_TRANSFER),
            outtake.wristOutAction(Outtake.WristStateOut.GRIPPER_TRANSFER)
        );
    }

    public static Action AfterTransferSequenceAction(Outtake outtake){
        return new SequentialAction(
                outtake.gripperAction(Outtake.GripperState.CLOSE),
                new SleepAction(0.5),
                outtake.sliderOutAction(Outtake.SliderStateOut.AFTER_TRANSFER)
        );
    }
}
