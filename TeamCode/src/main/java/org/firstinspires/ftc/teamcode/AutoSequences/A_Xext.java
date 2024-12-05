package org.firstinspires.ftc.teamcode.AutoSequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_Xext {
    public A_Xext(Intake intake, Intake.iXextensionStates state){
        Actions.runBlocking( new SequentialAction(
                intake.iXextension(state)
        ));
    }


}
