package org.firstinspires.ftc.teamcode.Sequences.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class A_High_Bucket_Pose {


    public  A_High_Bucket_Pose(Outtake outtake){
        Actions.runBlocking( new SequentialAction(
                outtake.wristOutAction(Outtake.WristStateOut.BEFORE_BUCKET),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
                outtake.sliderOutAction(Outtake.SliderStateOut.BUCKET_SCORE)


        ));
    }

}
