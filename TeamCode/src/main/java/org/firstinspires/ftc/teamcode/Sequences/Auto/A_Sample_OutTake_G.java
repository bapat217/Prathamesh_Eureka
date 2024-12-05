package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class A_Sample_OutTake_G {
// Outtake pos for transfer
    // Gripper open
public A_Sample_OutTake_G(Intake intake, Outtake outtake, Outtake.GripperState state ){
    Actions.runBlocking(new SequentialAction(
            outtake.gripperAction(state),
            outtake.elbowOutAction(Outtake.ElbowStateOut.TRANSFER),
            outtake.wristOutAction(Outtake.WristStateOut.TRANSFER)
    ));

    }
    public A_Sample_OutTake_G(Intake intake, Outtake outtake, Outtake.GripperState state ,int i){
        Actions.runBlocking(new SequentialAction(
                outtake.gripperAction(state),
                outtake.elbowOutAction(Outtake.ElbowStateOut.TRANSFER),
                outtake.wristOutAction(Outtake.WristStateOut.TRANSFER),
                outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
        ));

    }


}
