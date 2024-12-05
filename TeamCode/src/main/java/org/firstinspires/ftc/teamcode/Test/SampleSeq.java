//package org.firstinspires.ftc.teamcode.Test;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//
//public class SampleSeq {
//
//
////    public SampleSeq(Intake intake,List<Action> sampleActions){
////        sampleActions.add(new SequentialAction(
////                intake.flapperAction(Intake.FlapperState.INIT),
////                new SleepAction(4),
////                intake.flapperAction(Intake.FlapperState.CLOSE),
////                new SleepAction(2))
////
////        );
////    }
//    public static Action SampleSeqAction(Intake intake){
//        return new SequentialAction(
//                intake.flapperAction(Intake.FlapperState.DISCARD),
//                new SleepAction(1),
//                intake.flapperAction(Intake.FlapperState.TAKE),
//                new SleepAction(1)
//        );
//    }
//    public static Action SampleSeq2(Intake intake){
//        return new SequentialAction(
//                intake.flapperAction(Intake.FlapperState.DISCARD),
//                new SleepAction(4),
//                intake.flapperAction(Intake.FlapperState.TAKE),
//                new SleepAction(2)
//        );
//
//    }
//
//}
