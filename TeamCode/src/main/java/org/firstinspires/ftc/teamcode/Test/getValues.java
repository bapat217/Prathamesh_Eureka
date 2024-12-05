package org.firstinspires.ftc.teamcode.Test;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@Config
@TeleOp
@Disabled
public class getValues extends LinearOpMode {

    /* straight line
  leftelblow = 0.372
  rightelbow = 0.372
  flapper = 0.6
  yaw = 0.8811
    */
    public static double intakeServoL = -1;
    public static double intakeServoR = 0.5;
    public static double wristSer    = 0.56;
    public static double yawSer      = 0.8811;
    public static double elbowSerL   = 0.32722;
    public static double elbowSerR   =  0.3272;
    public static double flapSer     = 0.6;
    public static double gripperSer  = 0;
    public static double wristOutSer = 0;
    public static double elbowOutL   = 1;
    public static double elbowOutR   = 0;
    public static double clutchSer   = 0;

    public static double horizontalValueLow = 0;
    public static double horizontalValueHigh = 0;
    public static double horizontalValueMiddle = 0;
    public static double verticalValueUpperBasket = 0;
    public static double verticalValueLowerBasket = 0;
    public static double verticalLowChember = 0;
    public static double verticalHighChember = 0;
    public static double lowHangValue = 0;
    public static double highHangValue = 0;



    RobotHardware robot = RobotHardware.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);


//        wristSer = robot.gripper.getPosition();

//        elbowSerR = robot.elbowRight.getPosition();
//        flapSer = robot.flapper.getPosition();
//        gripperSer = robot.gripper.getPosition();
//        wristOutSer = robot.wristOut.getPosition();
//        elbowOutL= robot.elbowLeftOut.getPosition();
//        elbowOutR = robot.elbowRightOut.getPosition();
//        clutchSer = robot.clutchServo.getPosition();

        robot.elbowRightOut.setPosition(0.5);
        robot.elbowLeftOut.setPosition(0.5);
        robot.horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeInInit()){
//            robot.gripper.setPosition(gripperSer);
//            robot.wristServo.setPosition(wristSer);
////            robot.wristOut.setPosition(wristOutSer);
//            robot.elbowLeft.setPosition(elbowSerL);
//            robot.elbowRight.setPosition(elbowSerR);
//
////            robot.clutchServo.setPosition(clutchSer);
//            robot.flapper.setPosition(flapSer);
//            robot.yawServo.setPosition(yawSer);
            telemetry.addLine("INTAKE");
//            telemetry.addData("Left Elbow: ",robot.elbowLeft.getPosition());
//            telemetry.addData("Right Elbow: ",robot.elbowRight.getPosition());
//            telemetry.addData("Flapper: ",robot.flapper.getPosition());
//            telemetry.addData("Yaw Servo: ",robot.yawServo.getPosition());
//            telemetry.addData("Wrist Servo: ",robot.wristServo.getPosition());
            telemetry.addLine();
            telemetry.addLine("OUTTAKE");
            telemetry.addData("Outtake wrist: ",robot.wristOut.getPosition());
            telemetry.addData("Left Elbow Out: ",robot.elbowLeftOut.getPosition());
            telemetry.addData("Right Elbow Out: ",robot.elbowRightOut.getPosition());
            telemetry.addData("Clutch Servo: ",robot.clutchServo.getPosition());
            telemetry.addData("Gripper: ",robot.gripper.getPosition());
            telemetry.addData("Vertical Slider Left: ", robot.verticalSliderLeft.getCurrentPosition());
            telemetry.addData("Vertical Slider Right: ", robot.verticalSliderRight.getCurrentPosition());
            telemetry.addData("Horizontal Slider Left:", robot.horizontalExtension.getCurrentPosition());
            telemetry.addData("Hanger Low Value: ", robot.HangerLow.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){

//            if(gamepad1.x){
//                robot.intakeServoRight.setPower(intakeServoL);
//                robot.intakeServoLeft.setPower(intakeServoR);
//            }
//            if(gamepad1.b){
//                robot.intakeServoRight.setPower(0);
//                robot.intakeServoLeft.setPower(0);
//            }
            if(gamepad1.a){
                robot.horizontalExtension.setTargetPosition(robot.horizontalExtension.getCurrentPosition() + 100);
                robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.horizontalExtension.setPower(1);
            }
            if (gamepad1.b) {
                robot.horizontalExtension.setTargetPosition(robot.horizontalExtension.getCurrentPosition() - 100);
                robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.horizontalExtension.setPower(1);
            }
////            if(gamepad1.dpad_up){
////                robot.elbowLeft.setPosition(robot.elbowLeft.getPosition() + 0.001);
////                robot.elbowRight.setPosition(robot.elbowRight.getPosition() - 0.001);
////            }
//            if(gamepad1.dpad_down){
//                robot.elbowLeft.setPosition(robot.elbowLeft.getPosition() - 0.001);
//                robot.elbowRight.setPosition(robot.elbowRight.getPosition() + 0.001);
//            }
//
//            if(gamepad1.right_bumper){
//                robot.elbowLeft.setPosition(robot.elbowLeft.getPosition() + 0.001);
//                robot.elbowRight.setPosition(robot.elbowRight.getPosition() + 0.001);
//            }
//            if(gamepad1.left_bumper){
//                robot.elbowLeft.setPosition(robot.elbowLeft.getPosition() - 0.001);
//                robot.elbowRight.setPosition(robot.elbowRight.getPosition() - 0.001);
//            }
//
//
//            if(gamepad1.dpad_left){
//                robot.yawServo.setPosition(robot.yawServo.getPosition() + 0.001);
//            }
//            if(gamepad1.dpad_right){
//                robot.yawServo.setPosition(robot.yawServo.getPosition() - 0.001);
//            }
//            if(gamepad1.left_trigger>0){
//                robot.wristServo.setPosition(robot.wristServo.getPosition() + 0.001);
//            }
//            if(gamepad1.right_trigger>0){
//                robot.wristServo.setPosition(robot.wristServo.getPosition() - 0.001);
//            }

            if(gamepad2.right_bumper){
                robot.elbowLeftOut.setPosition(robot.elbowLeftOut.getPosition() + 0.001);
                robot.elbowRightOut.setPosition(robot.elbowRightOut.getPosition() - 0.001);

            }
            if(gamepad2.left_bumper){
                robot.elbowLeftOut.setPosition(robot.elbowLeftOut.getPosition() - 0.001);
                robot.elbowRightOut.setPosition(robot.elbowRightOut.getPosition() + 0.001);
            }

            if(gamepad2.dpad_down){
                robot.elbowLeftOut.setPosition(robot.elbowLeftOut.getPosition() - 0.001);
                robot.elbowRightOut.setPosition(robot.elbowRightOut.getPosition() - 0.001);
            }
            if(gamepad2.dpad_up){
                robot.elbowLeftOut.setPosition(robot.elbowLeftOut.getPosition() + 0.001);
                robot.elbowRightOut.setPosition(robot.elbowRightOut.getPosition() + 0.001);
            }
            if(gamepad2.y){
                robot.clutchServo.setPosition(robot.clutchServo.getPosition()+ 0.001);
            }
            if(gamepad2.a){
                robot.clutchServo.setPosition(robot.clutchServo.getPosition() - 0.001);
            }
//            if(gamepad2.x){
//                robot.flapper.setPosition(robot.flapper.getPosition() + 0.001);
//            }
//            if(gamepad2.b){
//                robot.flapper.setPosition(robot.flapper.getPosition() - 0.001);
//            }
            if(gamepad2.dpad_left){
                robot.gripper.setPosition(robot.gripper.getPosition() + 0.001);
            }
            if(gamepad2.dpad_right){
                robot.gripper.setPosition(robot.gripper.getPosition() - 0.001);
            }
            if(gamepad2.left_trigger>0){
                robot.wristOut.setPosition(robot.wristOut.getPosition() + 0.001);
            }
            if(gamepad2.right_trigger>0){
                robot.wristOut.setPosition(robot.wristOut.getPosition() - 0.001);
            }

            telemetry.addLine("INTAKE");
//            telemetry.addData("Left Elbow: ",robot.elbowLeft.getPosition());
//            telemetry.addData("Right Elbow: ",robot.elbowRight.getPosition());
//            telemetry.addData("Flapper: ",robot.flapper.getPosition());
//            telemetry.addData("Yaw Servo: ",robot.yawServo.getPosition());
//            telemetry.addData("Wrist Servo: ",robot.wristServo.getPosition());
            telemetry.addLine();
            telemetry.addLine("OUTTAKE");
            telemetry.addData("Outtake wrist: ",robot.wristOut.getPosition());
            telemetry.addData("Left Elbow Out: ",robot.elbowLeftOut.getPosition());
            telemetry.addData("Right Elbow Out: ",robot.elbowRightOut.getPosition());
            telemetry.addData("Clutch Servo: ",robot.clutchServo.getPosition());
            telemetry.addData("Gripper: ",robot.gripper.getPosition());
            telemetry.addData("Vertical Slider Left: ", robot.verticalSliderLeft.getCurrentPosition());
            telemetry.addData("Vertical Slider Right: ", robot.verticalSliderRight.getCurrentPosition());
            telemetry.addData("Horizontal Slider Left:", robot.horizontalExtension.getCurrentPosition());
            telemetry.addData("Hanger Low Value: ", robot.HangerLow.getCurrentPosition());
            telemetry.update();
        }

    }

/* pick
left 0.3317
right 9.2317
flap 0.6
yaw 0 .88
    */

// initialize



}
