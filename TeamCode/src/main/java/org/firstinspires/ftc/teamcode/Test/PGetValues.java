package org.firstinspires.ftc.teamcode.Test;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

@Disabled
@Config
@TeleOp
public class PGetValues extends LinearOpMode {

    public PIDFController controller = new PIDFController(0,0,0,0);

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    public static double Target = 0;



    /* straight line
  leftelblow = 0.372
  rightelbow = 0.372
  flapper = 0.6
  yaw = 0.8811
    */
    public static double intakeServoL = 0.5;
    public static double intakeServoR = -1;
    public static double wristSer    = 0.5;//0.7994;
    public static double yawSer      = 0.8667;
    public static double elbowSerL   = 0.525;
    public static double elbowSerR   =  0.3972;
    public static double flapSer     = 0.6;
    //specimen score
    // wrist 0.3922
    // elbowLOut 0.285
    //elbowR out 0.7083
    public static double wristPlace = 0.3922;
    public static double elbowLOut = 0.285;
    public static double elbowROut = 0.7083;

    public static double wristPickOut = 0.3844;
    public static double elbowLPick = 0.9667;
    public static double elbowRPick = 0.0328 ;
    private double servoLeftPosition = 0.5;
    private double servoRightPosition = 0.5;


    private double intElbowLeftPosition = 0.5;
    private double intElbowRightPosition = 0.5;

    public static double INCREMENT = 0.001;
    private static final double MAX_POS = 1.0;
    private static final double MIN_POS = 0.0;
    public static double power = 1;
    public static int sliderInc = 50;
    public static double gripperSer = 0.5;
    RobotHardware robot = RobotHardware.getInstance();
    public static int target=0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);


//        robot.wristServo.setPosition(wristSer);
        robot.gripper.setPosition(Globals.gripperServoOpen);
//        robot.elbowLeft.setPosition(Globals.elbowInitL);
//        robot.elbowRight.setPosition(Globals.elbowInitR);
//        robot.wristServo.setPosition(Globals.wristInitIntake);
//        robot.wristOut.setPosition(wristSer);
//        robot.flapper.setPosition(Globals.flapperInit);
//        robot.yawServo.setPosition(Globals.yawServoInit);

        robot.horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //outake
        robot.elbowLeftOut.setPosition(servoLeftPosition);
        robot.elbowRightOut.setPosition(servoRightPosition);

        //intake


        while(opModeInInit()){
            printStuff(telemetry);
            telemetry.update();
        }
        robot.verticalEncoder.reset();
        waitForStart();
        while(opModeIsActive()){

//            if(gamepad1.x){
//                robot.intakeServoLeft.setPower(intakeServoL);
//                robot.intakeServoRight.setPower(intakeServoR);
//            }
//            if(gamepad1.b){
//                robot.intakeServoRight.setPower(-intakeServoL);
//                robot.intakeServoLeft.setPower(-intakeServoR);
//            }
            if(gamepad1.a){
                robot.horizontalExtension.setTargetPosition(robot.horizontalExtension.getCurrentPosition() + 100);
                robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.horizontalExtension.setPower(1);
            }
            if (gamepad1.y) {
                robot.horizontalExtension.setTargetPosition(robot.horizontalExtension.getCurrentPosition() - 100);
                robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.horizontalExtension.setPower(1);
            }
//            if(gamepad1.dpad_up){
////                regularIntakeMotion(false);
//                robot.elbowRight.setPosition(robot.elbowRight.getPosition() - INCREMENT);
//                robot.elbowLeft.setPosition(robot.elbowLeft.getPosition() + INCREMENT);
//            }
//            if(gamepad1.dpad_down){
//                robot.elbowRight.setPosition(robot.elbowRight.getPosition() + INCREMENT);
//                robot.elbowLeft.setPosition(robot.elbowLeft.getPosition() - INCREMENT);
//            }
//            if(gamepad1.dpad_left){
//                robot.yawServo.setPosition(robot.yawServo.getPosition() + 0.001);
//            }
//            if(gamepad1.dpad_right){
//                robot.yawServo.setPosition(robot.yawServo.getPosition() - 0.001);
//            }
//            if(gamepad1.right_bumper){
////                viperIntakeMotion(false);
//                robot.elbowRight.setPosition(robot.elbowRight.getPosition() - INCREMENT);
//                robot.elbowLeft.setPosition(robot.elbowLeft.getPosition() - INCREMENT);
//            }
//            if(gamepad1.left_bumper){
////                viperIntakeMotion(true);
//                robot.elbowRight.setPosition(robot.elbowRight.getPosition() + INCREMENT);
//                robot.elbowLeft.setPosition(robot.elbowLeft.getPosition() + INCREMENT);
//            }
//
//            if(gamepad1.left_trigger>0){
//                robot.wristServo.setPosition(robot.wristServo.getPosition() + 0.001);
//            }
//            if(gamepad1.right_trigger>0){
//                robot.wristServo.setPosition(robot.wristServo.getPosition() - 0.001);
//            }

            if (gamepad2.left_bumper) {
                robot.elbowRightOut.setPosition(robot.elbowRightOut.getPosition() + INCREMENT);
                robot.elbowLeftOut.setPosition(robot.elbowLeftOut.getPosition() + INCREMENT);
            }
            if (gamepad2.right_bumper) {
                robot.elbowRightOut.setPosition(robot.elbowRightOut.getPosition() - INCREMENT);
                robot.elbowLeftOut.setPosition(robot.elbowLeftOut.getPosition() - INCREMENT);            }

            if (gamepad2.dpad_up) {
                robot.elbowRightOut.setPosition(robot.elbowRightOut.getPosition() - INCREMENT);
                robot.elbowLeftOut.setPosition(robot.elbowLeftOut.getPosition() + INCREMENT);
            }
            if (gamepad2.dpad_down) {
                robot.elbowRightOut.setPosition(robot.elbowRightOut.getPosition() + INCREMENT);
                robot.elbowLeftOut.setPosition(robot.elbowLeftOut.getPosition() - INCREMENT);
            }
//            if(gamepad1.dpad_left){
//                robot.flapper.setPosition(robot.flapper.getPosition() + 0.001);
//
//            }
//            if(gamepad1.dpad_right){
//                robot.flapper.setPosition(robot.flapper.getPosition() - 0.001);
//
//            }
            if(gamepad2.y){
                robot.clutchServo.setPosition(robot.clutchServo.getPosition()+ 0.001);

            }
            if(gamepad2.a){
                robot.clutchServo.setPosition(robot.clutchServo.getPosition() - 0.001);

            }

            // vertical left will follow right motor
//            robot.verticalSliderLeft.setTargetPosition(robot.verticalSliderRight.getCurrentPosition());
//            robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.verticalSliderLeft.setPower(-power);


            if(gamepad2.x){

//                Target = 10000;
                robot.verticalSliderLeft.setTargetPosition(target);
                robot.verticalSliderRight.setTargetPosition(target);
                robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalSliderLeft.setPower(power);
                robot.verticalSliderRight.setPower(power);

            }

            if(gamepad2.b){
//                Target = 0;

                robot.verticalSliderLeft.setTargetPosition(0);
                robot.verticalSliderRight.setTargetPosition(0);

                robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalSliderLeft.setPower(power);
                robot.verticalSliderRight.setPower(power);
//                robot.flapper.setPosition(robot.flapper.getPosition() - 0.001);
            }
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
//            updateElevator();
            printStuff(telemetry);

            telemetry.update();
        }

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
        telemetry.addData("Vertical Slider left Velo: ", robot.verticalSliderLeft.getVelocity());
        telemetry.addData("Vertical Slider Right Velo: ", robot.verticalSliderRight.getVelocity());
        telemetry.addData("Horizontal Slider Left:", robot.horizontalExtension.getCurrentPosition());
        telemetry.addData("Hanger Low Value: ", robot.HangerLow.getCurrentPosition());
        telemetry.addData("Port number  sliderL: ", robot.verticalSliderLeft.getPortNumber());
        telemetry.addData("Port number  sliderR: ", robot.verticalSliderRight.getPortNumber());
        telemetry.addData("Port number  horizontal: ", robot.horizontalExtension.getPortNumber());
        telemetry.addData("hang low: ", robot.HangerLow.getPortNumber());
        telemetry.addLine();
        telemetry.addData("Current horizontal:", robot.horizontalExtension.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current vertical:", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current vertical:", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
        telemetry.addData("Red:", robot.colorSensorIn.red());
        telemetry.addData("BLUE:", robot.colorSensorIn.blue());
        telemetry.addData("GREEN:", robot.colorSensorIn.green());
        telemetry.addData("External lifter enc",robot.verticalEncoder.getPosition());
    }


    // TODO --> PID
    public void updateElevator(){
        controller.setPIDF(kp,ki,kd,kf);
        double error=controller.calculate(robot.verticalEncoder.getPosition(),Target);
        robot.verticalSliderLeft.setPower(-error);
        robot.verticalSliderRight.setPower(-error);
    }



/* pick
left 0.3317
right 9.2317
flap 0.6
yaw 0 .88
    */

// initialize
// specimen pick
    // gripper open 0.4772
    //close 0.29
    //wrist specimen pick 0.4517
    //wrist specimen after pick 0.1017
    //after pick slider left -77
    //after pick slider right -77
    //elbow left 0.0694
    //elbow right 0.9328

    /*
    score values
    outakw wrist
     */


    //

}













