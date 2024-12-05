package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class A_Score_Reset_Sample {
    public A_Score_Reset_Sample (Outtake outtake,Intake intake){
        Actions.runBlocking(new SequentialAction(
//                outtake.elbowOutAction(Outtake.ElbowStateOut.INBETWEEN),
//                new SleepAction(0.1),
            outtake.wristOutAction(Outtake.WristStateOut.BUCKET_SCORE),
//                outtake.elbowOutAction(Outtake.ElbowStateOut.BUCKET_SCORE),
            outtake.gripperAction(Outtake.GripperState.OPEN),
            new SleepAction(0.5),
            outtake.wristOutAction(Outtake.WristStateOut.BEFORE_BUCKET),
            outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
            new SleepAction( 0.6),
            outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
    ));
}

public A_Score_Reset_Sample (Outtake outtake,double timer){
        Actions.runBlocking(new SequentialAction(
            outtake.wristOutAction(Outtake.WristStateOut.BUCKET_SCORE),
            new SleepAction(0.25),
            outtake.gripperAction(Outtake.GripperState.OPEN),
            new SleepAction(timer),//0.5
            outtake.wristOutAction(Outtake.WristStateOut.BEFORE_BUCKET),
            outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET)
//            new SleepAction( 0.6),
//            outtake.sliderOutAction(Outtake.SliderStateOut.INIT)
    ));
}

}
