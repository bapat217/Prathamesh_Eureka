package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
// TODO TESTED
public class SpecimenIntakeSequence {
    public static Action SpecimenIntakeSequenceAction(Outtake outtake) {

        return new SequentialAction(outtake.gripperAction(Outtake.GripperState.OPEN),
                outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK),
                outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
                new SleepAction(0.5),
                outtake.sliderOutAction(Outtake.SliderStateOut.INIT));
    }
}
