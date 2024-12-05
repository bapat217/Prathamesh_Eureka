package org.firstinspires.ftc.teamcode.InstantCommand;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class SliderCommand {

    public SliderCommand(Outtake outtake,Outtake.SliderStateOut state){
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> {
                           outtake.updateState(state);
                        })
                )
        );
    }
}
