package org.firstinspires.ftc.teamcode.Sequences.Teleop;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class SpecimenOuttakeSequence {
    public static Action SpecimenOuttakeAfterPick(Outtake outtake){
     return new SequentialAction(
             outtake.gripperAction(Outtake.GripperState.CLOSE),
             new SleepAction(0.3),
//            new InstantAction(()->new ElevatorCommand(outtake, Outtake.SliderStateOut.HIGH)),
             outtake.sliderOutAction(Outtake.SliderStateOut.S_HIGH),
             new SleepAction(0.4),
//             outtake.wristOutAction(Outtake.WristStateOut.AFTER_PICK),
//             new SleepAction(0.2),
             outtake.elbowOutAction(Outtake.ElbowStateOut.AFTER_PICK),
             outtake.wristOutAction(Outtake.WristStateOut.AFTER_PICK),
             new SleepAction(0.1),
             outtake.sliderOutAction(Outtake.SliderStateOut.HIGH)

             );
    } public static Action N_SpecimenOuttakeAfterPick(Outtake outtake){
     return new SequentialAction(outtake.gripperAction(Outtake.GripperState.CLOSE),
             outtake.twistAction(Outtake.TwistState.TWIST_ZERO),
             new SleepAction(0.3),
             outtake.sliderOutAction(Outtake.SliderStateOut.S_HIGH),
             new SleepAction(0.4),
             outtake.wristOutAction(Outtake.WristStateOut.AFTER_PICK),
             new SleepAction(0.3),
             outtake.elbowOutAction(Outtake.ElbowStateOut.AFTER_PICK),
             outtake.wristOutAction(Outtake.WristStateOut.BEFORE_SCORE), // befor score specimen
             new SleepAction(0.1),
             outtake.sliderOutAction(Outtake.SliderStateOut.HIGH)

             );
    }

    public static Action SAMSpecimenOuttakeAfterPick(Outtake outtake){
        return new SequentialAction(
                outtake.gripperAction(Outtake.GripperState.CLOSE),
                new SleepAction(0.3),
//            new InstantAction(()->new ElevatorCommand(outtake, Outtake.SliderStateOut.HIGH)),
                outtake.sliderOutAction(Outtake.SliderStateOut.S_HIGH),
                new SleepAction(0.4),
//             outtake.wristOutAction(Outtake.WristStateOut.AFTER_PICK),
//             new SleepAction(0.2),
                outtake.elbowOutAction(Outtake.ElbowStateOut.AFTER_PICK),
                outtake.wristOutAction(Outtake.WristStateOut.AFTER_PICK),
                new SleepAction(0.1),
                outtake.sliderOutAction(Outtake.SliderStateOut.HIGH),
                outtake.elbowOutAction(Outtake.ElbowStateOut.RUNG),
                outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PLACE)

        );
    }
    public static Action SpecimenOuttakeBeforeScore(Outtake outtake){
     return new SequentialAction(
             outtake.wristOutAction(Outtake.WristStateOut.BEFORE_SCORE),
             outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_SCORE)
             );
    }

}
