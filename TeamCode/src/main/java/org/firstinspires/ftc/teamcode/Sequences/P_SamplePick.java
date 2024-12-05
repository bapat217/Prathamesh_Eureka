package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


    // TODO Just init things
    public class P_SamplePick {

    public static Action intakeInit(Intake intake, Outtake outtake){
        return new SequentialAction(
                intake.iGripper(Intake.iGripperStates.INIT),
                intake.iWrist(Intake.iWristStates.POSE_0),
                intake.iElbow(Intake.iElobowStates.INIT),
                intake.iShoulde(Intake.iShoulderStates.INIT),
                intake.iWiper(Intake.iWiperStates.INIT),
                intake.iXextension(Intake.iXextensionStates.INIT)

        );
    }

    // Pre Pick
    public static Action PrePickSequence(Intake intake, Outtake outtake){
        return new SequentialAction(
                intake.iXextension(Intake.iXextensionStates.MID),
                new SleepAction(0.3),
                intake.iGripper(Intake.iGripperStates.OPEN),
                intake.iElbow(Intake.iElobowStates.BEFORE_PICK),
                intake.iShoulde(Intake.iShoulderStates.BEFORE_PICK),
                intake.iWiper(Intake.iWiperStates.BEFORE_PICK)

        );
    } public static Action PrePickSequenceUperLift(Intake intake, Outtake outtake){
        return new SequentialAction(
                intake.iXextension(Intake.iXextensionStates.MID),
                new SleepAction(0.3),
                intake.iGripper(Intake.iGripperStates.OPEN),
                intake.iElbow(Intake.iElobowStates.INIT),
                intake.iShoulde(Intake.iShoulderStates.BEFORE_PICK),
                intake.iWiper(Intake.iWiperStates.BEFORE_PICK)

        );
    }
    // Pick sequence
    public static Action PickSequence(Intake intake, Outtake outtake){
        return new SequentialAction(
                // picked
                intake.iGripper(Intake.iGripperStates.CLOSE),
                new SleepAction(0.2),
                intake.iElbow(Intake.iElobowStates.TRANSFER),
                intake.iShoulde(Intake.iShoulderStates.TRANSFER),// PICK
                intake.iWrist(Intake.iWristStates.POSE_90), // try
                new SleepAction(0.1),
                intake.iWiper(Intake.iWiperStates.AFTER_PICK),
                intake.iXextension(Intake.iXextensionStates.INIT)

        );
    }



    // Transfer Pose
    public static Action SampleTransferGClosed(Intake intake, Outtake outtake){
        return new SequentialAction(

                // Pre pick pose
                intake.iGripper(Intake.iGripperStates.CLOSE),
                new SleepAction(0.2),
                intake.iShoulde(Intake.iShoulderStates.TRANSFER),
                intake.iWrist(Intake.iWristStates.POSE_90),
                intake.iElbow(Intake.iElobowStates.AFTER_PICK),
                intake.iElbow(Intake.iElobowStates.TRANSFER),
                intake.iWiper(Intake.iWiperStates.TRANSFER),
                intake.iXextension(Intake.iXextensionStates.INIT)

        );
    }
    // Transfer Pose gripper open (intake system)
    public static Action SampleIntakeGOpen(Intake intake, Outtake outtake){
        return new SequentialAction(

                intake.iGripper(Intake.iGripperStates.OPEN),
                intake.iWrist(Intake.iWristStates.POSE_90),
                intake.iElbow(Intake.iElobowStates.TRANSFER),
                intake.iShoulde(Intake.iShoulderStates.TRANSFER),
                intake.iWiper(Intake.iWiperStates.TRANSFER)

        );
    }

    public static Action XExtState(Intake intake, Intake.iXextensionStates state){
        return new SequentialAction(
                intake.iXextension(state)
        );
    }
    // Outtake pos for transfer
    // Gripper open
    public static Action SampleOutTakeGOpen(Intake intake, Outtake outtake){
    return new SequentialAction(

       outtake.gripperAction(Outtake.GripperState.OPEN),
            outtake.elbowOutAction(Outtake.ElbowStateOut.TRANSFER),
            outtake.wristOutAction(Outtake.WristStateOut.TRANSFER)

    );
    }
    // Gripper Closed
    public static Action SampleOutTakeGClosed(Intake intake, Outtake outtake){
        return new SequentialAction(
                outtake.gripperAction(Outtake.GripperState.CLOSE),
                outtake.elbowOutAction(Outtake.ElbowStateOut.TRANSFER),
                outtake.wristOutAction(Outtake.WristStateOut.TRANSFER)

        );
    }

    // Before transfer pose
    public static Action OutTakeaBeforeTrans(Intake intake, Outtake outtake){
        return new SequentialAction(

                outtake.gripperAction(Outtake.GripperState.OPEN),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_TRANSFER),
                outtake.wristOutAction(Outtake.WristStateOut.BEFORE_TRANSFER)

        );
    }

    // Wrist pose for intake gripper
    public static Action  wristPose(Intake intake,Intake.iWristStates state){
        return new SequentialAction(
                intake.iWrist(state)

        );
    }

    public static Action sampleDropOBS(Intake intake, Outtake outtake){
        return new SequentialAction(
                intake.iXextension(Intake.iXextensionStates.MID),
                intake.iShoulde(Intake.iShoulderStates.OBS_DROP),
                intake.iWrist(Intake.iWristStates.POSE_90),
                intake.iElbow(Intake.iElobowStates.OBS_DROP),
                intake.iWiper(Intake.iWiperStates.OBS_DROP),
                new SleepAction(0.05),
                intake.iGripper(Intake.iGripperStates.OPEN)

        );
    }
}
