package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_OuTake_Before_Trans {
// Before transfer pose
    public  A_OuTake_Before_Trans(Intake intake, Outtake outtake){
        Actions.runBlocking(new SequentialAction(

                outtake.gripperAction(Outtake.GripperState.OPEN),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_TRANSFER),
                outtake.wristOutAction(Outtake.WristStateOut.BEFORE_TRANSFER)

        ));
    }
}
