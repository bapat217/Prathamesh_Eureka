package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_Sample_Pick {


    // Pick sequence
    public A_Sample_Pick (Intake intake, Outtake outtake){

        Actions.runBlocking(new SequentialAction(
                intake.iGripper(Intake.iGripperStates.CLOSE),
                new SleepAction(0.2),
                intake.iElbow(Intake.iElobowStates.TRANSFER),
                intake.iShoulde(Intake.iShoulderStates.TRANSFER),// PICK
                intake.iWrist(Intake.iWristStates.POSE_90), // try
                new SleepAction(0.1),
                intake.iWiper(Intake.iWiperStates.AFTER_PICK),
                intake.iXextension(Intake.iXextensionStates.INIT)
        ));

    }
    public A_Sample_Pick (Intake intake){

        Actions.runBlocking(new SequentialAction(
                intake.iElbow(Intake.iElobowStates.TRANSFER),
                intake.iShoulde(Intake.iShoulderStates.TRANSFER),// PICK
                intake.iWrist(Intake.iWristStates.POSE_90), // try
                new SleepAction(0.1),
                intake.iWiper(Intake.iWiperStates.AFTER_PICK),
                intake.iXextension(Intake.iXextensionStates.INIT)
        ));

    }
    public A_Sample_Pick (Intake intake, Outtake outtake,Intake.iXextensionStates state ){

        Actions.runBlocking(new SequentialAction(
                intake.iGripper(Intake.iGripperStates.CLOSE),
                new SleepAction(0.2),
                intake.iElbow(Intake.iElobowStates.TRANSFER),
                intake.iShoulde(Intake.iShoulderStates.TRANSFER),// PICK
                intake.iWrist(Intake.iWristStates.POSE_90), // try
                new SleepAction(0.1),
                intake.iWiper(Intake.iWiperStates.AFTER_PICK),
                intake.iXextension(state)
        ));

    }
}
