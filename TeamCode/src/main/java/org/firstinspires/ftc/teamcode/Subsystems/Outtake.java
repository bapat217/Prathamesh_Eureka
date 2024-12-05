package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.GripperIntakeGlobals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Outtake {
    public RobotHardware robot;
    public static int targetValue = 0;

    public PIDController sliderController = null;
    public Outtake(RobotHardware robot){
        this.robot = robot;
//        sliderController = new PIDController(Globals.kp, Globals.ki, Globals.kd);
    }

    public enum ClutchState{
        INIT,
        HANG_ENGAGED,
        ELEV_ENGAGED,
        DISENGAGED
    }
    public enum TwistState{
        INIT,
        TWIST_ZERO,
        TWIST_180,

    }


    public enum ElbowStateOut{
        INIT,
        PREAFTER_PICK,
        AFTER_PICK,
        BEFORE_SCORE,
        SPECIMEN_PICK,
        SPECIMEN_PLACE,
        BEFORE_TRANSFER,
        TRANSFER,
        BEFORE_BUCKET,
        GRIPPER_TRANSFER,
        BUCKET_SCORE,
        RUNG,
        INBETWEEN
    }
    public enum GripperState{
        INIT,
        OPEN,
        CLOSE
    }

    public enum WristStateOut{
        INIT,
        AFTER_PICK,
        BEFORE_SCORE,
        SPECIMEN_PICK,
        SPECIMEN_PLACE,
        BEFORE_TRANSFER,
        TRANSFER,
        BEFORE_BUCKET,
        GRIPPER_TRANSFER,
        BUCKET_SCORE
    }

    public enum SliderStateOut{
        INIT,
        HIGH,
        S_HIGH,
        SCORE,
        AFTER_TRANSFER,
        BUCKET_SCORE,
        HANG,
        PRE_HANG
    } public enum HangerState{
        INIT,
        OPEN,
        FIRST_HANG,
        SECOND_HANG
    }
    public HangerState hangerState = HangerState.OPEN;
    public GripperState gripperState = GripperState.OPEN;
    public WristStateOut wristStateOut = WristStateOut.INIT;
    public  ElbowStateOut elbowStateOut = ElbowStateOut.INIT;
    public SliderStateOut sliderStateOut = SliderStateOut.INIT;
    public TwistState twistState = TwistState.INIT;
    public ClutchState clutchState = ClutchState.INIT;

    public void updateState(ClutchState currentState){
        this.clutchState = currentState;
        switch(currentState) {
            case INIT:
                robot.clutchServo.setPosition(Globals.DISENGAGED);
                break;
            case HANG_ENGAGED:
                robot.clutchServo.setPosition(Globals.HANG_ENGAGED);
                break;
            case ELEV_ENGAGED:
                robot.clutchServo.setPosition(Globals.ELEV_ENGAGED);
                break;
            case DISENGAGED:
                robot.clutchServo.setPosition(Globals.DISENGAGED);
                break;
        }

    }  public void updateState(TwistState currentState){
        this.twistState = currentState;
        switch(currentState) {
            case INIT:
                robot.twist.setPosition(Globals.TWIST_INIT);
                break;
            case TWIST_ZERO:
                robot.twist.setPosition(Globals.TWIST_0);
                break;
            case TWIST_180:
                robot.twist.setPosition(Globals.TWIST_180);
                break;

        }

    }

    public void updateState(GripperState currentState){
        this.gripperState = currentState;
        switch(currentState){
            case INIT:
                break;
            case OPEN:
                robot.gripper.setPosition(Globals.oGripperOpen);
                break;
            case CLOSE:
                robot.gripper.setPosition(Globals.oGripperClosed);
                break;

        }
    }  public void updateState(HangerState currentState){
        this.hangerState = currentState;
        int target =0;
        switch(currentState){
            case INIT:
                target =Globals.lowerHang_INIT;
                break;
            case OPEN:
                target =Globals.lowerHang_OPEN;
                break;
            case FIRST_HANG:
                target =Globals.lowerHang_First;
                break;
            case SECOND_HANG:
                target =Globals.lowerHang_Second;
                break;
        }
        lHAnger(target);
    }


    public void updateState(ElbowStateOut currentState){
        this.elbowStateOut = currentState;
        switch (currentState){
            case INIT:
                robot.elbowLeftOut.setPosition(Globals.elbowInitOutL);
                robot.elbowRightOut.setPosition(Globals.elbowInitOutR);

                break;
            case PREAFTER_PICK:
                robot.elbowLeftOut.setPosition(Globals.elbowPreAfterPickSpecimenL);
                robot.elbowRightOut.setPosition(Globals.elbowPreAfterPickSpecimenR);
                break;
            case AFTER_PICK:
                robot.elbowLeftOut.setPosition(Globals.elbowAfterPickSpecimenL);
                robot.elbowRightOut.setPosition(Globals.elbowAfterPickSpecimenR);
                break;
            case BEFORE_SCORE:
                robot.elbowLeftOut.setPosition(Globals.elbowBeforeScoreSpecimenL);
                robot.elbowRightOut.setPosition(Globals.elbowBeforeScoreSpecimenR);
                break;
            case SPECIMEN_PICK:
                robot.elbowLeftOut.setPosition(Globals.elbowPickSpecimenL);
                robot.elbowRightOut.setPosition(Globals.elbowPickSpecimenR);
                break;
            case SPECIMEN_PLACE:
                robot.elbowLeftOut.setPosition(Globals.elbowPlaceSpecimenL);
                robot.elbowRightOut.setPosition(Globals.elbowPlaceSpecimenR);
                break;
            case BEFORE_TRANSFER:
                robot.elbowLeftOut.setPosition(Globals.beforeTransferelbowLeftOut);
                robot.elbowRightOut.setPosition(Globals.beforeTransferelbowRightOut);
                break;
            case TRANSFER:
                robot.elbowLeftOut.setPosition(Globals.transferelbowLeftOut);
                robot.elbowRightOut.setPosition(Globals.transferelbowRightOut);
                break;
            case GRIPPER_TRANSFER:
                robot.elbowLeftOut.setPosition(GripperIntakeGlobals.elbowOutTransferL);
                robot.elbowRightOut.setPosition(GripperIntakeGlobals.elbowOutTransferR);
                break;
            case BEFORE_BUCKET:
                robot.elbowLeftOut.setPosition(Globals.elbowBeforePlaceBucketL);
                robot.elbowRightOut.setPosition(Globals.elbowBeforePlaceBucketR);
                break;
            case BUCKET_SCORE:
                robot.elbowLeftOut.setPosition(Globals.elbowPlaceBucketL);
                robot.elbowRightOut.setPosition(Globals.elbowPlaceBucketR);
//            case RUNG:
//                robot.elbowLeftOut.setPosition(Globals.SAMelbowAfterPickSpecimenL);
//                robot.elbowRightOut.setPosition(Globals.SAMelbowAfterPickSpecimenR);
//            case INBETWEEN:
//                robot.elbowLeftOut.setPosition(Globals.INBET_elbowAfterPickSpecimenL);
//                robot.elbowRightOut.setPosition(Globals.INBET_elbowAfterPickSpecimenR);

        }
    }

    public void updateState(WristStateOut currentState){
        this.wristStateOut = currentState;
        switch (currentState){
            case INIT:
                robot.wristOut.setPosition(Globals.wristInitOutTake);
                break;
            case AFTER_PICK:
                robot.wristOut.setPosition(Globals.wristAfterPickSpecimen);
                break;
            case BEFORE_SCORE:
                robot.wristOut.setPosition(Globals.wristBeforeScoreSpecimen);
                break;
            case SPECIMEN_PICK:
                robot.wristOut.setPosition(Globals.wristPickSpecimen);
                break;
            case SPECIMEN_PLACE:
                robot.wristOut.setPosition(Globals.wristPlaceSpecimen);
                break;
            case BEFORE_TRANSFER:
                robot.wristOut.setPosition(Globals.beforeTransferWristOut);
                break;
            case TRANSFER:
                robot.wristOut.setPosition(Globals.transferWristOut);
                break;
            case GRIPPER_TRANSFER:
                robot.wristOut.setPosition(GripperIntakeGlobals.wristTransferOut);
                break;
            case BEFORE_BUCKET:
                robot.wristOut.setPosition(Globals.wristBeforePlaceBucket);
                break;
            case BUCKET_SCORE:
                robot.wristOut.setPosition(Globals.wristPlaceBucket);
                break;
        }
    }
    public void updateState(SliderStateOut currentState){
        this.sliderStateOut = currentState;
        switch (currentState){
//            case INIT:
//                target = 0;
//                break;
//            case HIGH:
//                target = Globals.HighChamberValue;
//                break;
//            case SCORE:
//                target = Globals.SpecimenScoreHigh;
//                break;
//            case AFTER_TRANSFER:
//                target = Globals.afterTransfer;
//                break;
//            case BUCKET_SCORE:
//                target = BucketValue;
//                break;
            case INIT:
                runSliders(0,1,10);
                break;
            case HIGH:
                targetValue = Globals.HighChamberValue;
                runSliders(Globals.HighChamberValue, 1);
                break;
            case S_HIGH:
                targetValue = Globals.SHighChamberValue;
                runSliders(Globals.SHighChamberValue, 1);
                break;
            case SCORE:
                targetValue = Globals.SpecimenScoreHigh;
                runSliders(Globals.SpecimenScoreHigh, 1);
                break;
            case AFTER_TRANSFER:
                runSliders(Globals.afterTransfer, 1);
                break;
            case BUCKET_SCORE:
                runSliders(Globals.BucketValue, 1,10);
//                runSliderPID(Globals.BucketValueThroughBore);
//                runSliderPID();
//                Globals.BucketValueThroughBore = 50000;
                break;
            case HANG:
                runSliders(Globals.ELEV_HANG, 1);
                break;
            case PRE_HANG:
                runSliders(Globals.ELEV_PRE_HANG, 1);
                break;

        }
    }

    public Action gripperAction(GripperState state){
        return new InstantAction(()->updateState(state));
    }
    public Action elbowOutAction(ElbowStateOut state){
        return new InstantAction(()->updateState(state));
    }
    public Action clutchAction(ClutchState state){
        return new InstantAction(()->updateState(state));
    }
    public Action twistAction(TwistState state){
        return new InstantAction(()->updateState(state));
    }
    public Action sliderOutAction(SliderStateOut state){
        return new InstantAction(()->updateState(state));
    }
    public Action wristOutAction(WristStateOut state){
        return new InstantAction(()->updateState(state));
    }
    public Action lowerHangAction (HangerState state){
        return new InstantAction(()->updateState(state));
    }

    public void runSliders(int pos, double pow){
        robot.verticalSliderLeft.setTargetPositionTolerance(2);
        robot.verticalSliderRight.setTargetPositionTolerance(2);
        robot.verticalSliderLeft.setTargetPosition(pos);
        robot.verticalSliderRight.setTargetPosition(pos);
        robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSliderRight.setPower(pow);
        robot.verticalSliderLeft.setPower(pow);
    }
    public void runSliders(int pos, double pow, int tol){
        robot.verticalSliderLeft.setTargetPositionTolerance(tol);
        robot.verticalSliderRight.setTargetPositionTolerance(tol);
        robot.verticalSliderLeft.setTargetPosition(pos);
        robot.verticalSliderRight.setTargetPosition(pos);
        robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSliderRight.setPower(pow);
        robot.verticalSliderLeft.setPower(pow);
    }

    public double clutchPID(int target){
//        controller.calculate(kp, ki, kd);
        sliderController.setPID(Globals.kp,Globals.ki,Globals.kd);
        double power = sliderController.calculate(robot.verticalEncoder.getPosition(), target);
        return power;
    }
    public double clutchPID(){
//        controller.calculate(kp, ki, kd);
        sliderController.setPID(Globals.kp,Globals.ki,Globals.kd);
        double power = sliderController.calculate(robot.verticalEncoder.getPosition(), Globals.BucketValueThroughBore);
        return power;
    }

    public void runSliderPID(int pos){
        robot.verticalSliderLeft.setPower(-clutchPID(pos));
        robot.verticalSliderRight.setPower(-clutchPID(pos));
    }
    public void runSliderPID(){
        robot.verticalSliderLeft.setPower(-clutchPID());
        robot.verticalSliderRight.setPower(-clutchPID());
    }
    public void lHAnger(int target){
        robot.HangerLow.setTargetPosition(target);
        robot.HangerLow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.HangerLow.setPower(1);
    }
}










