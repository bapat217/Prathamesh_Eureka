package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Globals.iGripper_Close;
import static org.firstinspires.ftc.teamcode.Hardware.Globals.iGripper_Init;
import static org.firstinspires.ftc.teamcode.Hardware.Globals.iGripper_Open;
import static org.firstinspires.ftc.teamcode.Hardware.Globals.iWiper_Init;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Intake {

    public static RobotHardware robot;
    public Intake(RobotHardware robot){
        this.robot = robot;
    }


    public enum iGripperStates{
        INIT,
        OPEN,
        CLOSE
    }

    public enum iWristStates{
        INIT,
        POSE_0,
        POSE_45,
        POSE_90,
        POSE_135,
        POSE_180,


    }

    public enum iElobowStates{
        INIT,
        BEFORE_PICK,
        PICK,
        AFTER_PICK,
        TRANSFER,
        POST_TRANSFER,
        OBS_DROP
    }

    public enum iShoulderStates{
        INIT,
        BEFORE_PICK,
        PICK,
        AFTER_PICK,
        TRANSFER,
        OBS_DROP

    }
    public enum iWiperStates{
        INIT,
        BEFORE_PICK,
        PICK,
        AFTER_PICK,
        TRANSFER,
        LEFT,
        RIGHT,
        OBS_DROP
    }
    public enum iXextensionStates{
        INIT,
        MID,
        LAST,
        CLOSE,
        AUTO_MID_YELLOW,
        OBS_DROP
    }


    public  iElobowStates ielbowState= iElobowStates.INIT;
    public iGripperStates igripperState = iGripperStates.INIT;
    public iWristStates iwristState = iWristStates.INIT;
    public iShoulderStates ishoulderState = iShoulderStates.INIT;
    public iWiperStates iwiperState = iWiperStates.INIT;
    public iXextensionStates ixextensionState = iXextensionStates.INIT;

    // Update
    public void updateState(iGripperStates currentState){
        this.igripperState = currentState;
        double current = iGripper_Init;
        switch (currentState){
            case INIT:
                current = iGripper_Init;
                break;
            case OPEN:
                current = iGripper_Open;
                break;
            case CLOSE:
                current = iGripper_Close;
                break;
        }
        robot.iGripper.setPosition(current);
    }
    public void updateState(iWristStates currentState){
        this.iwristState = currentState;
        double current = iWiper_Init;
        switch (currentState){
            case INIT:
                current = iWiper_Init;
                break;
            case POSE_0:
                current = Globals.iWrist_0;
                break;
            case POSE_45:
                current = Globals.iWrist_45;
                break;
            case POSE_90:
                current = Globals.iWrist_90;
                break;
            case POSE_135:
                current = Globals.iWrist_135;
                break;
            case POSE_180:
                current = Globals.iWrist_180;
                break;
        }
        robot.iWrist.setPosition(current);
    }



    public void updateState(iElobowStates currentState){
        this.ielbowState = currentState;
        double current = Globals.iElbow_Init;

        switch(currentState){
            case INIT:
                current = Globals.iElbow_Init;
                break;
            case BEFORE_PICK:
                current = Globals.iElbow_BeforePick;
                break;
            case PICK:
                current = Globals.iElbow_Pick;
                break;
            case AFTER_PICK:
                current = Globals.iElbow_AfterPick;
                break;
            case TRANSFER:
                current = Globals.iElbow_Transfer;
                break;
            case POST_TRANSFER:
                current = Globals.iElbow_Post_Transfer;
                break;
            case OBS_DROP:
                current = Globals.iElbow_Obs_Drop;
                break;
        }
        if(this.ixextensionState == iXextensionStates.MID || (this.ixextensionState == iXextensionStates.LAST)) {
            current = Globals.iElbowChangePos;
        }
        robot.iElbow.setPosition(current);

    }





    public void updateState(iShoulderStates currentState){
        this.ishoulderState = currentState;
        double current = Globals.iShoulder_Init;
        switch(currentState){
            case INIT:
                current = Globals.iShoulder_Init;
                break;
            case BEFORE_PICK:
                current = Globals.iShoulder_BeforePick;
                break;
            case PICK:
                current = Globals.iShoulder_Pick;
                break;
            case AFTER_PICK:
                current = Globals.iShoulder_AfterPick;
                break;
            case TRANSFER:
                current = Globals.iShoulder_Transfer;
                break;
            case OBS_DROP:
                current = Globals.iShoulder_Obs_Drop;
                break;
        }
        robot.iShoulder.setPosition(current);
    }
    public void updateState(iWiperStates currentState){
        this.iwiperState = currentState;
        double current = Globals.iWiper_Init;
        switch(currentState){
            case INIT:
                current = Globals.iWiper_Init;
                break;
            case BEFORE_PICK:
                current = Globals.iWiper_BeforePick;
                break;
            case PICK:
                current = Globals.iWiper_Pick;
                break;
            case AFTER_PICK:
                current = Globals.iWiper_AfterPick;
                break;
            case TRANSFER:
                current = Globals.iWiper_Transfer;
                break;
            case LEFT:
                current = Globals.iWiper_Left;
                break;
            case RIGHT:
                current = Globals.iWiper_Right;
                break;
            case OBS_DROP:
                current = Globals.iWiper_Obs_Drop;
                break;

        }
        robot.iWiper.setPosition(current);
    }

    public void updateState(iXextensionStates currentState){
        this.ixextensionState = currentState;
        int current = Globals.iXext_Init;
        switch (currentState){
            case INIT:
                current = Globals.iXext_Init;
                break;
            case CLOSE:
                current = Globals.iXext_Close;
                break;
            case MID:
                current = Globals.iXext_Mid;
                break;
            case LAST:
                current = Globals.iXext_Last;
                break;
            case AUTO_MID_YELLOW:
                current = Globals.iXext_AutoMidYellow;
                break;
        }
        runSliderX(current);

    }


   public Action iGripper (iGripperStates state){
        return new InstantAction(()->updateState(state));
   }

    public Action iWrist (iWristStates state){
        return new InstantAction(()->updateState(state));
    }

    public Action iElbow (iElobowStates state){
        return new InstantAction(()->updateState(state));
    }

    public Action iShoulde (iShoulderStates state){
        return new InstantAction(()->updateState(state));
    }

    public Action iWiper (iWiperStates state){
        return new InstantAction(()->updateState(state));
    }
    public Action iXextension (iXextensionStates state){
        return new InstantAction(()->updateState(state));
    }

    public static void runSliders(int pos, double pow){
        robot.horizontalExtension.setTargetPosition(pos);
        robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontalExtension.setPower(pow);
    }
    public static void runSliderX(int pos){
        robot.horizontalExtension.setTargetPosition(pos);
        robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontalExtension.setPower(1);
    }
}

