package org.firstinspires.ftc.teamcode.InstantCommand;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

public class IntakeGripCommand {

    public IntakeGripCommand(Intake intake,Intake.iGripperStates state){
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> {
                           intake.updateState(state);
                        })
                )
        );
    }
}
