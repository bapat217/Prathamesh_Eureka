package org.firstinspires.ftc.teamcode.AutoSequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_Wrist_Ctrl {
    // Wrist pose for uintake gripper
    public A_Wrist_Ctrl(Intake intake,Intake.iWristStates state){
        Actions.runBlocking( new SequentialAction(
                intake.iWrist(state)

        ) );
    }
}
