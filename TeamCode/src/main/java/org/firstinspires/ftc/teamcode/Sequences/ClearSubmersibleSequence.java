package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class ClearSubmersibleSequence {

    public static Action clearSequenceInitAction(Intake intake){
        return new SequentialAction(
//                intake.rollerAction(Intake.RollerState.REVERSE),
//                intake.elbowAction(Intake.ElbowState.CLEAR_INIT),
//                intake.wristAction(Intake.WristState.CLEAR_INIT)

        );
    }

    public static Action clearSequenceAction(Intake intake){
        return new SequentialAction(
//                intake.wristAction(Intake.WristState.CLEAR),
//                intake.elbowAction(Intake.ElbowState.CLEAR),
//                intake.sliderAction(Intake.SliderState.HIGH),
//                new SleepAction(1),
//                intake.wristAction(Intake.WristState.PICK),
//                intake.elbowAction(Intake.ElbowState.PICK),
//                intake.sliderAction(Intake.SliderState.LOW),
                new SleepAction(1)

        );
    }
}
