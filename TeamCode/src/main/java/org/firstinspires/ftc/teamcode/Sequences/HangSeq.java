package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


// TODO Just init things
public class HangSeq {
    public static Action HangerINIT(Intake intake, Outtake outtake){
        return new SequentialAction(

//               outtake.clutchAction(Outtake.ClutchState.HANG_ENGAGED),
                intake.iGripper(Intake.iGripperStates.INIT),
                intake.iWrist(Intake.iWristStates.INIT),
                intake.iElbow(Intake.iElobowStates.INIT),
                intake.iShoulde(Intake.iShoulderStates.INIT),
                intake.iWiper(Intake.iWiperStates.INIT),

                outtake.gripperAction(Outtake.GripperState.CLOSE),
                outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK),
                outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
                outtake.gripperAction(Outtake.GripperState.OPEN)

//                new SleepAction(1),
//                outtake.lowerHangAction(state)

        );
    }
    public static Action HangerL(Outtake outtake, Outtake.HangerState state){
        return new SequentialAction(

//                outtake.clutchAction(Outtake.ClutchState.HANG_ENGAGED),
                outtake.lowerHangAction(state)

        );
    }public static Action HangerLCLUTCH(Outtake outtake, Outtake.HangerState state, Outtake.ClutchState clstate){
        return new SequentialAction(

                outtake.clutchAction(clstate),
                outtake.lowerHangAction(state)

        );
    }
    public static Action HangerELEV(Outtake outtake, Outtake.SliderStateOut state){
        return new SequentialAction(
//                outtake.clutchAction(Outtake.ClutchState.HANG_ENGAGED),
                outtake.sliderOutAction(state)

        );
    }
}
