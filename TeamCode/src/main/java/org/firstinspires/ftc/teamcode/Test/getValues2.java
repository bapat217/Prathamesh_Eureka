package org.firstinspires.ftc.teamcode.Test;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@Config
@TeleOp
@Disabled
public class getValues2 extends LinearOpMode {

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


    private double servoLeftPosition = 0.5;
    private double servoRightPosition = 0.5;


    private double intElbowLeftPosition = 0.5;
    private double intElbowRightPosition = 0.5;

    private static final double INCREMENT = 0.01;
    private static final double MAX_POS = 1.0;
    private static final double MIN_POS = 0.0;


    RobotHardware robot = RobotHardware.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);


//        robot.wristServo.setPosition(wristSer);
//
//        robot.elbowLeft.setPosition(elbowSerL);
//        robot.elbowRight.setPosition(elbowSerR);
//
//        robot.flapper.setPosition(flapSer);
//        robot.yawServo.setPosition(yawSer);

        robot.horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //outake
        robot.elbowLeftOut.setPosition(servoLeftPosition);
        robot.elbowRightOut.setPosition(servoRightPosition);

        //intake
//        robot.elbowLeft.setPosition(intElbowLeftPosition);
//        robot.elbowRight.setPosition(intElbowRightPosition);

        while(opModeInInit()){
            printStuff(telemetry);
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
            if(gamepad1.dpad_up){
           regularIntakeMotion(false);
            }
            if(gamepad1.dpad_down){
              regularIntakeMotion(true);
            }

            if(gamepad1.right_bumper){
             viperIntakeMotion(false);
            }
            if(gamepad1.left_bumper){
             viperIntakeMotion(true);
            }


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

            if (gamepad2.left_bumper) {
                viperMotion(true);
            } else if (gamepad2.right_bumper) {
                viperMotion(false);
            }

            if (gamepad2.dpad_up) {
                regularMotion(true);
            } else if (gamepad2.dpad_down) {
                regularMotion(false);
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
            if(gamepad2.back){
                robot.gripper.setPosition(robot.gripper.getPosition() + 0.001);
            }
            if(gamepad2.start){
                robot.gripper.setPosition(robot.gripper.getPosition() - 0.001);
            }
            if(gamepad2.left_trigger>0){
                robot.wristOut.setPosition(robot.wristOut.getPosition() + 0.001);
            }
            if(gamepad2.right_trigger>0){
                robot.wristOut.setPosition(robot.wristOut.getPosition() - 0.001);
            }


            printStuff(telemetry);

            telemetry.update();
        }

    }




    private void viperMotion(boolean increase) {
        if (increase) {
            servoLeftPosition = Math.min(servoLeftPosition + INCREMENT, MAX_POS);
            servoRightPosition = Math.max(servoRightPosition - INCREMENT, MIN_POS);
        } else {
            servoLeftPosition = Math.max(servoLeftPosition - INCREMENT, MIN_POS);
            servoRightPosition = Math.min(servoRightPosition + INCREMENT, MAX_POS);
        }
        robot.elbowLeftOut.setPosition(servoLeftPosition);
        robot.elbowRightOut.setPosition(servoRightPosition);
    }

    private void regularMotion(boolean increase) {
        if (increase) {
            servoLeftPosition = Math.min(servoLeftPosition + INCREMENT, MAX_POS);
            servoRightPosition = Math.min(servoRightPosition + INCREMENT, MAX_POS);
        } else {
            servoLeftPosition = Math.max(servoLeftPosition - INCREMENT, MIN_POS);
            servoRightPosition = Math.max(servoRightPosition - INCREMENT, MIN_POS);
        }
        robot.elbowLeftOut.setPosition(servoLeftPosition);
        robot.elbowRightOut.setPosition(servoRightPosition);
    }

    private void viperIntakeMotion(boolean increase) {
        if (increase) {
            intElbowLeftPosition = Math.min(intElbowLeftPosition + INCREMENT, MAX_POS);
            intElbowRightPosition = Math.max(intElbowRightPosition - INCREMENT, MIN_POS);
        } else {
            intElbowLeftPosition = Math.min(intElbowLeftPosition - INCREMENT, MAX_POS);
            intElbowRightPosition = Math.max(intElbowRightPosition + INCREMENT, MIN_POS);
        }
//        robot.elbowLeft.setPosition(intElbowLeftPosition);
//        robot.elbowRight.setPosition(intElbowRightPosition);
    }

    private void regularIntakeMotion(boolean increase) {
        if (increase) {
            intElbowLeftPosition = Math.min(intElbowLeftPosition + INCREMENT, MAX_POS);
            intElbowRightPosition = Math.max(intElbowRightPosition + INCREMENT, MIN_POS);
        } else {
            intElbowLeftPosition = Math.min(intElbowLeftPosition - INCREMENT, MAX_POS);
            intElbowRightPosition = Math.max(intElbowRightPosition - INCREMENT, MIN_POS);
        }
//        robot.elbowLeft.setPosition(intElbowLeftPosition);
//        robot.elbowRight.setPosition(intElbowRightPosition);
    }

    public  void printStuff(Telemetry telemetry){
        telemetry.addLine("INTAKE");
//        telemetry.addData("Left Elbow: ",robot.elbowLeft.getPosition());
//        telemetry.addData("Right Elbow: ",robot.elbowRight.getPosition());
//        telemetry.addData("Flapper: ",robot.flapper.getPosition());
//        telemetry.addData("Yaw Servo: ",robot.yawServo.getPosition());
//        telemetry.addData("Wrist Servo: ",robot.wristServo.getPosition());
        telemetry.addLine();
        telemetry.addLine("OUTTAKE");
        telemetry.addData("Outtake wrist: ",robot.wristOut.getPosition());
        telemetry.addData("ELBOWOUT Left Position", servoLeftPosition);
        telemetry.addData("ELBOWOUT Right Position", servoRightPosition);
        telemetry.addData("Left Elbow Out: ",robot.elbowLeftOut.getPosition());
        telemetry.addData("Right Elbow Out: ",robot.elbowRightOut.getPosition());
        telemetry.addData("Clutch Servo: ",robot.clutchServo.getPosition());
        telemetry.addData("Gripper: ",robot.gripper.getPosition());
        telemetry.addData("Vertical Slider Left: ", robot.verticalSliderLeft.getCurrentPosition());
        telemetry.addData("Vertical Slider Right: ", robot.verticalSliderRight.getCurrentPosition());
        telemetry.addData("Horizontal Slider Left:", robot.horizontalExtension.getCurrentPosition());
        telemetry.addData("Hanger Low Value: ", robot.HangerLow.getCurrentPosition());
    }

/* pick
left 0.3317
right 9.2317
flap 0.6
yaw 0 .88
    */

// initialize



}















/*
      telemetry.addLine("INTAKE");
            telemetry.addData("Left Elbow: ",robot.elbowLeft.getPosition());
            telemetry.addData("Right Elbow: ",robot.elbowRight.getPosition());
            telemetry.addData("Flapper: ",robot.flapper.getPosition());
            telemetry.addData("Yaw Servo: ",robot.yawServo.getPosition());
            telemetry.addData("Wrist Servo: ",robot.wristServo.getPosition());
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
 */