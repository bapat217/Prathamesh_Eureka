package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_SamplePrePick {



// Pre Pick
    public A_SamplePrePick (Intake intake, Outtake outtake){
        Actions.runBlocking(new SequentialAction(

                intake.iWrist(Intake.iWristStates.POSE_90),
                intake.iXextension(Intake.iXextensionStates.LAST),
//                new SleepAction(0.3),
                intake.iGripper(Intake.iGripperStates.OPEN),
                intake.iElbow(Intake.iElobowStates.BEFORE_PICK),
                intake.iShoulde(Intake.iShoulderStates.BEFORE_PICK),
                intake.iWiper(Intake.iWiperStates.BEFORE_PICK)

        ));
    }
    public A_SamplePrePick (Intake intake, Outtake outtake,int in){
        Actions.runBlocking(new SequentialAction(

                intake.iWrist(Intake.iWristStates.POSE_90),
                intake.iXextension(Intake.iXextensionStates.INIT),
//                new SleepAction(0.3),
                intake.iGripper(Intake.iGripperStates.OPEN),
                intake.iElbow(Intake.iElobowStates.BEFORE_PICK),
                intake.iShoulde(Intake.iShoulderStates.BEFORE_PICK),
                intake.iWiper(Intake.iWiperStates.BEFORE_PICK)

        ));
    }

    public A_SamplePrePick (Intake intake, Outtake outtake, Intake.iXextensionStates state){
        Actions.runBlocking(new SequentialAction(

                intake.iWrist(Intake.iWristStates.POSE_90),
                intake.iXextension(state),
                new SleepAction(0.3),
                intake.iGripper(Intake.iGripperStates.OPEN),
                intake.iElbow(Intake.iElobowStates.BEFORE_PICK),
                intake.iShoulde(Intake.iShoulderStates.BEFORE_PICK),
                intake.iWiper(Intake.iWiperStates.BEFORE_PICK)

        ));
    }
    // Pick sequence



}
