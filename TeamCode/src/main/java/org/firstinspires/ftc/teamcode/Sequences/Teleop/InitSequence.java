package org.firstinspires.ftc.teamcode.Sequences.Teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;



// TODO Just init things
public class InitSequence {
    public static Action InitSequenceAction(Intake intake, Outtake outtake){
        return new SequentialAction(

// Its a a intake
                intake.iGripper(Intake.iGripperStates.INIT),
                intake.iWrist(Intake.iWristStates.INIT),
                intake.iElbow(Intake.iElobowStates.OBS_DROP),
                new SleepAction(0.3),
                intake.iElbow(Intake.iElobowStates.INIT),
                intake.iShoulde(Intake.iShoulderStates.INIT),
                intake.iWiper(Intake.iWiperStates.INIT),



// Its a a Out take
                outtake.gripperAction(Outtake.GripperState.CLOSE),
                outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
                outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK),
                new SleepAction(1),
                outtake.gripperAction(Outtake.GripperState.OPEN),
                outtake.clutchAction(Outtake.ClutchState.ELEV_ENGAGED)
        );
    }
}
