package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_OuTake_Init {

    public A_OuTake_Init(Intake intake, Outtake outtake){

        Actions.runBlocking(new SequentialAction(
                outtake.clutchAction(Outtake.ClutchState.ELEV_ENGAGED),
                new SleepAction(1),
                outtake.gripperAction(Outtake.GripperState.CLOSE),
                outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
                outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK),
                new SleepAction(1)
//                outtake.gripperAction(Outtake.GripperState.OPEN)
        ));
    }


}
