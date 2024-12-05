//package org.firstinspires.ftc.teamcode.Opmode.Teleop;
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.Hardware.Globals;
//import org.firstinspires.ftc.teamcode.Hardware.GripperIntakeGlobals;
//import org.firstinspires.ftc.teamcode.Hardware.NewIntakeRobotHardware;
//
//
//@TeleOp
//@Config
//public class elev_follower extends LinearOpMode {
//    public static double kP = 0;
//    public static double kV = 0;
//
//    DcMotor  L = null ;
//    DcMotor R = null  ;
////    DcMotorEx k = null;
//    public MotorGroup motorgrp1 ;
//
////    NewIntakeRobotHardware robot = NewIntakeRobotHardware.getInstance();
//
//    public static int poseX = 0;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//
//
//        motorgrp1 = new DcMotorGroup(
//                hardwareMap.get(DcMotorEx.class, "sliderL"),
//                hardwareMap.get(DcMotorEx.class, "sliderR")
//
//        );
//
//
//        motorgrp1.setRunMode(Motor.RunMode.VelocityControl);
//        motorgrp1.setVeloCoefficients(kP, 0, 0);
//        motorgrp1.setFeedforwardCoefficients(0, kV);
//
//
//
////        robot.gripperOut.setPosition(Globals.gripperServoOpen);
//        waitForStart();
//
//
//        while (opModeIsActive()) {
//
//            if(gamepad1.a){
//
//                motorgrp1.setTargetPosition(1000);
//                motorgrp1.setRunMode(Motor.RunMode.PositionControl);
//                motorgrp1.set(1);
//            }
//            if(gamepad1.b){
//
//                motorgrp1.setTargetPosition(1000);
//                motorgrp1.set(1);
////                motorgrp1.setRunMode(Motor.RunMode.PositionControl);
////                motorgrp1.setRunMode(Motor.RunMode.PositionControl);
//            }if(gamepad1.a){
//
//                motorgrp1.setTargetPosition(0);
//                motorgrp1.setRunMode(Motor.RunMode.PositionControl);
//                motorgrp1.set(1);
//            }
//            if(gamepad1.b){
//
//                motorgrp1.setTargetPosition(0);
//                motorgrp1.set(1);
////                motorgrp1.setRunMode(Motor.RunMode.PositionControl);
////                motorgrp1.setRunMode(Motor.RunMode.PositionControl);
//            }
//
//
//
//
//
//            telemetry.addData("left pose", L.getCurrentPosition());
//            telemetry.addData("right pose", R.getCurrentPosition());
//            telemetry.addData("left Current", L.getCurrentPosition());
//            telemetry.addData("right Current", R.getCurrentPosition());
//            telemetry.addData("left Velocity", L.getCurrentPosition());
//            telemetry.addData("right Velocity", R.getCurrentPosition());
//
////            telemetry.addData("Hanger mot POSE----> ", robot.HangerLow.getCurrentPosition());
////            telemetry.addData("Hanger mot CURRENT->", robot.HangerLow.getCurrent(CurrentUnit.AMPS));
//            telemetry.update();
//
//        }
//
//    }
//    public static void extendTO (int target, DcMotorEx m){
//        m.setTargetPosition(target);
//        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        m.setPower(1);
//
//
//    }
//
//
//}
