package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

public class ElevatorCommand {
    public ElevatorCommand(Outtake outtake, Outtake.SliderStateOut state) {
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(()->outtake.updateState(state))
                )
        );
    }
}
