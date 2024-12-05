//package org.firstinspires.ftc.teamcode.Test;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//
//public class SampleSeq2 {
//
//    public static Action SampleSeq2Action(Intake intake){
//       return new SequentialAction(
//                intake.flapperAction(Intake.FlapperState.DISCARD),
//                new SleepAction(4),
//                intake.flapperAction(Intake.FlapperState.TAKE),
//                new SleepAction(2)
//        );
//
//    }
//
//}
