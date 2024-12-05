package org.firstinspires.ftc.teamcode.AutoSequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class A_SpeciGriped_ready_to_hook {
    public A_SpeciGriped_ready_to_hook (Outtake outtake){
        Actions.runBlocking(new SequentialAction(
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

             ));
    }


}
