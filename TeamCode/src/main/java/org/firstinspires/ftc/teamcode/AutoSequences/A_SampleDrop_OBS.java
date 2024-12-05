package org.firstinspires.ftc.teamcode.AutoSequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_SampleDrop_OBS {

    public A_SampleDrop_OBS(Intake intake, Outtake outtake){
        Actions.runBlocking(new SequentialAction(

                intake.iXextension(Intake.iXextensionStates.MID),
                intake.iShoulde(Intake.iShoulderStates.OBS_DROP),
                intake.iWrist(Intake.iWristStates.POSE_90),
                intake.iElbow(Intake.iElobowStates.OBS_DROP),
                intake.iWiper(Intake.iWiperStates.OBS_DROP),
                new SleepAction(0.05),
                intake.iGripper(Intake.iGripperStates.OPEN)

        ));
    }}
