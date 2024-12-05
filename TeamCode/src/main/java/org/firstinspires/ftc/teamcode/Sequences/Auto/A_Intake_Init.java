package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_Intake_Init {

    public A_Intake_Init(Intake intake, Outtake outtake){

        Actions.runBlocking(new SequentialAction(


                intake.iGripper(Intake.iGripperStates.INIT),
                intake.iWrist(Intake.iWristStates.POSE_0),
                intake.iElbow(Intake.iElobowStates.INIT),
                intake.iShoulde(Intake.iShoulderStates.INIT),
                intake.iWiper(Intake.iWiperStates.INIT),
                intake.iXextension(Intake.iXextensionStates.INIT)
        ));
    }


}
