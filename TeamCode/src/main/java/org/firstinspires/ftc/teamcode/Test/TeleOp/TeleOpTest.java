//package org.firstinspires.ftc.teamcode.Test;
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.Hardware.Globals;
//import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
//
//
//@Config
//@TeleOp
//public class TeleOpTest extends LinearOpMode {
//
//    /* straight line
//  leftelblow = 0.372
//  rightelbow = 0.372
//  flapper = 0.6
//  yaw = 0.8811
//    */
//    public static double intakeServoL = -0.5;
//    public static double intakeServoR = 1;
//    public static double yawSer      = 0.8667;
//    public static double elbowSerL   = 0.525;
//    public static double elbowSerR   =  0.3972;
//    public static double flapSer     = 0.6;
//    //specimen score
//    // wrist 0.3922
//    // elbowLOut 0.285
//    //elbowR out 0.7083
//    public static double wristPlace = 0.3922;
//    public static double elbowLOut = 0.285;
//    public static double elbowROut = 0.7083;
//
//    public static double wristPickOut = 0.3844;
//    public static double elbowLPick = 0.9667;
//    public static double elbowRPick = 0.0328 ;
//    private double servoLeftPosition = 0.5;
//    private double servoRightPosition = 0.5;
//
//
//    private double intElbowLeftPosition = 0.5;
//    private double intElbowRightPosition = 0.5;
//
//    public static double INCREMENT = 0.001;
//    private static final double MAX_POS = 1.0;
//    private static final double MIN_POS = 0.0;
//    public static double power = 1;
//    public static int sliderInc = 50;
//    public static double gripperSer = 0.5;
//    public static double wristSer = 0.5;
//    RobotHardware robot = RobotHardware.getInstance();
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, telemetry);
//
//
////        robot.wristServo.setPosition(wristSer);
//        robot.gripper.setPosition(gripperSer);
//        robot.elbowLeft.setPosition(elbowSerL);
//        robot.elbowRight.setPosition(elbowSerR);
//        robot.wristOut.setPosition(wristSer);
//
//        robot.flapper.setPosition(flapSer);
//        robot.yawServo.setPosition(yawSer);
//
//        robot.horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //outake
//        robot.elbowLeftOut.setPosition(servoLeftPosition);
//        robot.elbowRightOut.setPosition(servoRightPosition);
//
//        //intake
//
//
//        while(opModeInInit()){
//            printStuff(telemetry);
//            robot.verticalEncoder.reset();
//            telemetry.update();
//        }
//        waitForStart();
//        while(opModeIsActive()){
//
////            if(gamepad2.left_trigger> 0){
//////                robot.intakeServoRight.setPower(Globals.intakePowerR);
//////                robot.intakeServoLeft.setPower(Globals.intakePowerL);
////                runMotors(Globals.HighChamberValue, 1);
////                sleep(1000);
////                robot.elbowLeftOut.setPosition(Globals.elbowAfterPickSpecimenL);
////                robot.elbowRightOut.setPosition(Globals.elbowAfterPickSpecimenR);
////
////            }
//            if(gamepad2.right_trigger > 0){
//                robot.wristOut.setPosition(Globals.wristBeforeScoreSpecimen);
//                sleep(500);
//                robot.elbowLeftOut.setPosition(Globals.elbowBeforeScoreSpecimenL);
//                robot.elbowRightOut.setPosition(Globals.elbowBeforeScoreSpecimenR);
////                robot.intakeServoRight.setPower(0);
////                robot.intakeServoLeft.setPower(0);
//            }
//            if(gamepad2.back){
//                robot.intakeServoRight.setPower(-Globals.intakePowerR);
//                robot.intakeServoLeft.setPower(-Globals.intakePowerL);
//            }
//
//            if(gamepad2.y){
////                robot.clutchServo.setPosition(robot.clutchServo.getPosition()+ 0.001);
////                robot.elbowLeftOut.setPosition(Globals.elbowPlaceSpecimenL);
////                robot.elbowRightOut.setPosition(Globals.elbowPlaceSpecimenR);
////                robot.wristOut.setPosition(Globals.wristPlaceSpecimen);
//////                runMotors(0,1);
////                sleep(500);
////                robot.gripper.setPosition(Globals.gripperServoOpen);
//                runMotorsHorizontal(0, 1);
//                robot.elbowLeft.setPosition(Globals.elbowInitL);
//                robot.elbowRight.setPosition(Globals.elbowInitR);
//                robot.yawServo.setPosition(Globals.yawServoInit);
//                robot.wristServo.setPosition(Globals.wristInitIntake);
//
//            }
//            if(gamepad2.a){
////                robot.clutchServo.setPosition(robot.clutchServo.getPosition() - 0.001);
////                robot.elbowLeftOut.setPosition(Globals.elbowPickSpecimenL);
////                robot.elbowRightOut.setPosition(Globals.elbowPickSpecimenR);
////                robot.wristOut.setPosition(Globals.wristPickSpecimen);
//////// CLEAR SUBMERSIBLE
//                robot.elbowLeft.setPosition(Globals.clearSubmersibleInitL);
//                robot.elbowRight.setPosition(Globals.clearSubmersibleInitR);
//                robot.wristServo.setPosition(Globals.wristClearSubmersibleInit);
//                robot.intakeServoLeft.setPower(Globals.intakePowerR);
//                robot.intakeServoRight.setPower(-Globals.intakePowerR);
//
//            }
//            if(gamepad2.left_trigger > 0){
////                robot.wristServo.setPosition(Math.min(robot.wristOut.getPosition() - 0.001, Globals.wristPickSubmersibleIntake));
//                runMotorsHorizontal(713, 1);
//                // AFTER ACHIEVING SUBMERSIBLE
//                robot.elbowLeft.setPosition(0.7106);
//                robot.elbowRight.setPosition(0.1111);
//                robot.wristServo.setPosition(0.8078);
//                sleep(1500);
//                runMotorsHorizontal(350, 1);
//                robot.elbowLeft.setPosition(Globals.elbowPickSubmersibleL);
//                robot.elbowRight.setPosition(Globals.elbowPickSubmersibleR);
//                robot.wristServo.setPosition(Globals.wristPickSubmersibleIntake);
//                sleep(1000);
//                robot.intakeServoLeft.setPower(Globals.intakePowerL);
//                robot.intakeServoRight.setPower(Globals.intakePowerR);
////                runMotors(Globals.SpecimenScoreHigh, 1);
////                sleep(1000);
////                robot.gripper.setPosition(Globals.gripperServoOpen);
//            /// EXTENSION FOR CLEARING
//
//
//            }
//            if(gamepad2.b){
//                robot.elbowLeftOut.setPosition(0.7261);
//                robot.elbowRightOut.setPosition(0.345);
//                robot.wristOut.setPosition(0.9617);
//                sleep(1000);
//                robot.gripper.setPosition(Globals.gripperServoOpen);
//                sleep(1000);
//                robot.wristOut.setPosition(0.7828);
////                runMotors(-1150, 1);
//
//            }
//            if(gamepad2.left_bumper){
//                robot.gripper.setPosition(0.5);
//            }
//            if(gamepad2.right_bumper){
//                robot.gripper.setPosition(0.3528);
//                sleep(500);
////                runMotors(-200, 1);
//
//            }
//
//
//            printStuff(telemetry);
//
//            telemetry.update();
//        }
//
//    }
//
//    public void runMotorsVertical(int pos, double power){
//        robot.verticalSliderLeft.setTargetPosition(pos);
//        robot.verticalSliderRight.setTargetPosition(pos);
//        robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.verticalSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.verticalSliderLeft.setPower(power);
//        robot.verticalSliderRight.setPower(power);
//    }
//    public void runMotorsHorizontal(int pos, double power){
//        robot.horizontalExtension.setTargetPosition(pos);
//        robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.horizontalExtension.setPower(power);
//    }
//
//
//
//
//
//
//
//
//    public  void printStuff(Telemetry telemetry){
//        telemetry.addLine("INTAKE");
//        telemetry.addData("Left Elbow: ",robot.elbowLeft.getPosition());
//        telemetry.addData("Right Elbow: ",robot.elbowRight.getPosition());
//        telemetry.addData("Flapper: ",robot.flapper.getPosition());
//        telemetry.addData("Yaw Servo: ",robot.yawServo.getPosition());
//        telemetry.addData("Wrist Servo: ",robot.wristServo.getPosition());
//        telemetry.addLine();
//        telemetry.addLine("OUTTAKE");
//        telemetry.addData("Outtake wrist: ",robot.wristOut.getPosition());
//        telemetry.addData("ELBOWOUT Left Position", servoLeftPosition);
//        telemetry.addData("ELBOWOUT Right Position", servoRightPosition);
//        telemetry.addData("Left Elbow Out: ",robot.elbowLeftOut.getPosition());
//        telemetry.addData("Right Elbow Out: ",robot.elbowRightOut.getPosition());
//        telemetry.addData("Clutch Servo: ",robot.clutchServo.getPosition());
//        telemetry.addData("Gripper: ",robot.gripper.getPosition());
//        telemetry.addData("Vertical Slider Left: ", robot.verticalSliderLeft.getCurrentPosition());
//        telemetry.addData("Vertical Slider Right: ", robot.verticalSliderRight.getCurrentPosition());
//        telemetry.addData("Horizontal Slider Left:", robot.horizontalExtension.getCurrentPosition());
//        telemetry.addData("Hanger Low Value: ", robot.HangerLow.getCurrentPosition());
//        telemetry.addData("Port number  sliderL: ", robot.verticalSliderLeft.getPortNumber());
//        telemetry.addData("Port number  sliderR: ", robot.verticalSliderRight.getPortNumber());
//        telemetry.addData("Port number  horizontal: ", robot.horizontalExtension.getPortNumber());
//        telemetry.addData("hang low: ", robot.HangerLow.getPortNumber());
//        telemetry.addData("vertical Encoder: ", robot.verticalEncoder.getPosition());
//        telemetry.addLine();
//        telemetry.addData("Current horizontal:", robot.horizontalExtension.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Current vertical:", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Current vertical:", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
//
//    }
//
///* pick
//left 0.3317
//right 9.2317
//flap 0.6
//yaw 0 .88
//    */
//
//// initialize
//// specimen pick
//    // gripper open 0.4772
//    //close 0.29
//    //wrist specimen pick 0.4517
//    //wrist specimen after pick 0.1017
//    //after pick slider left -77
//    //after pick slider right -77
//    //elbow left 0.0694
//    //elbow right 0.9328
//
//    /*
//    score values
//    outakw wrist
//     */
//
//
//    //
//
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
///*
//      telemetry.addLine("INTAKE");
//            telemetry.addData("Left Elbow: ",robot.elbowLeft.getPosition());
//            telemetry.addData("Right Elbow: ",robot.elbowRight.getPosition());
//            telemetry.addData("Flapper: ",robot.flapper.getPosition());
//            telemetry.addData("Yaw Servo: ",robot.yawServo.getPosition());
//            telemetry.addData("Wrist Servo: ",robot.wristServo.getPosition());
//            telemetry.addLine();
//            telemetry.addLine("OUTTAKE");
//            telemetry.addData("Outtake wrist: ",robot.wristOut.getPosition());
//            telemetry.addData("Left Elbow Out: ",robot.elbowLeftOut.getPosition());
//            telemetry.addData("Right Elbow Out: ",robot.elbowRightOut.getPosition());
//            telemetry.addData("Clutch Servo: ",robot.clutchServo.getPosition());
//            telemetry.addData("Gripper: ",robot.gripper.getPosition());
//            telemetry.addData("Vertical Slider Left: ", robot.verticalSliderLeft.getCurrentPosition());
//            telemetry.addData("Vertical Slider Right: ", robot.verticalSliderRight.getCurrentPosition());
//            telemetry.addData("Horizontal Slider Left:", robot.horizontalExtension.getCurrentPosition());
//            telemetry.addData("Hanger Low Value: ", robot.HangerLow.getCurrentPosition());
// */