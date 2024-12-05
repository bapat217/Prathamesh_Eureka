package org.firstinspires.ftc.teamcode.Opmode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.GripperIntakeGlobals;
import org.firstinspires.ftc.teamcode.Hardware.NewIntakeRobotHardware;

@TeleOp

public class GetValues extends LinearOpMode {
    NewIntakeRobotHardware robot = NewIntakeRobotHardware.getInstance();
    private double INCREMENT = 0.001;
    private static double power = 1;
    private static int sliderInc = 50;
    private double servoLeftPosition = 0.5;
    private double servoRightPosition = 0.5;
    private static double wristSer    = 0.5;//0.7994;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.newIntakeInit(hardwareMap, telemetry);

        robot.gripper.setPosition(GripperIntakeGlobals.gripperClose);
        robot.elbow.setPosition(GripperIntakeGlobals.elbowInit);
        robot.shoulder.setPosition(GripperIntakeGlobals.shoudlerInit);
        robot.wristServo.setPosition(GripperIntakeGlobals.wristINIT);
        robot.yawServo.setPosition(GripperIntakeGlobals.yawPick);
        robot.horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.elbowLeftOut.setPosition(servoLeftPosition);
        robot.elbowRightOut.setPosition(servoRightPosition);
        robot.wristOut.setPosition(wristSer);
        robot.gripperOut.setPosition(Globals.oGripperOpen);
        waitForStart();


        while(opModeIsActive()) {
            if(gamepad1.dpad_up){
                robot.shoulder.setPosition(robot.shoulder.getPosition() + 0.001);
            }
            if(gamepad1.dpad_down){
                robot.shoulder.setPosition(robot.shoulder.getPosition() - 0.001);

            }

            if(gamepad1.dpad_left){
                robot.yawServo.setPosition(robot.yawServo.getPosition() + 0.001);
            }
            if(gamepad1.dpad_right){
                robot.yawServo.setPosition(robot.yawServo.getPosition() - 0.001);

            }

            if(gamepad1.x){
                robot.wristServo.setPosition(robot.wristServo.getPosition() + 0.001);
            }
            if(gamepad1.b){
                robot.wristServo.setPosition(robot.wristServo.getPosition() - 0.001);

            }

            if(gamepad1.y){
                robot.elbow.setPosition(robot.elbow.getPosition() + 0.001);
            }
            if(gamepad1.a){
                robot.elbow.setPosition(robot.elbow.getPosition() - 0.001);

            }

            if(gamepad1.left_bumper){
                robot.gripper.setPosition(robot.gripper.getPosition() + 0.01);
            }
            if(gamepad1.right_bumper){
                robot.gripper.setPosition(robot.gripper.getPosition() - 0.01);

            }
            if(gamepad1.start){
                robot.horizontalExtension.setTargetPosition(700);
                robot.horizontalExtension.setTargetPositionTolerance(10);
                robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.horizontalExtension.setPower(1);
            }
            if (gamepad1.back) {
                robot.horizontalExtension.setTargetPosition(0);
                robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.horizontalExtension.setPower(1);
            }


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
            if(gamepad2.back){
                robot.gripperOut.setPosition(robot.gripperOut.getPosition() + 0.001);

            }
            if(gamepad2.start){
                robot.gripperOut.setPosition(robot.gripperOut.getPosition() - 0.001);

            }if(gamepad2.left_stick_button){
                robot.twist.setPosition(robot.twist.getPosition() - 0.01);

            }if(gamepad2.right_stick_button){
                robot.twist.setPosition(robot.twist.getPosition() + 0.01);

            }

            if(gamepad2.x){

//                robot.verticalSliderLeft.setTargetPosition(robot.verticalSliderLeft.getCurrentPosition() + sliderInc);
//                robot.verticalSliderRight.setTargetPosition(robot.verticalSliderRight.getCurrentPosition() + sliderInc);
                robot.verticalSliderLeft.setTargetPosition(0);
                robot.verticalSliderRight.setTargetPosition(0);
                robot.verticalSliderLeft.setTargetPositionTolerance(10);
                robot.verticalSliderRight.setTargetPositionTolerance(10);
                robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalSliderLeft.setPower(power);
                robot.verticalSliderRight.setPower(power);

            }
            if(gamepad2.b){

//                robot.verticalSliderLeft.setTargetPosition(robot.verticalSliderLeft.getCurrentPosition() - sliderInc);
//                robot.verticalSliderRight.setTargetPosition(robot.verticalSliderRight.getCurrentPosition() - sliderInc);
                robot.verticalSliderLeft.setTargetPosition(Globals.BucketValue);
                robot.verticalSliderRight.setTargetPosition(Globals.BucketValue);
                robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.verticalSliderLeft.setPower(power);
                robot.verticalSliderRight.setPower(power);
//                robot.flapper.setPosition(robot.flapper.getPosition() - 0.001);
            }
            if(gamepad2.left_trigger>0){
                robot.wristOut.setPosition(robot.wristOut.getPosition() + 0.001);
            }
            if(gamepad2.right_trigger>0){
                robot.wristOut.setPosition(robot.wristOut.getPosition() - 0.001);
            }
            telemetry.addData("gripper O :", robot.gripperOut.getPosition());
            telemetry.addData("gripper I:", robot.gripper.getPosition());
            telemetry.addData("wrist:", robot.wristServo.getPosition());
            telemetry.addData("twist:", robot.twist.getPosition());
            telemetry.addData("elbow:", robot.elbow.getPosition());
            telemetry.addData("shoulder:", robot.shoulder.getPosition());
            telemetry.addData("yaw:",robot.yawServo.getPosition());

            telemetry.addData("Left Elbow Out: ",robot.elbowLeftOut.getPosition());
            telemetry.addData("Right Elbow Out: ",robot.elbowRightOut.getPosition());
            telemetry.addData("Outtake wrist: ",robot.wristOut.getPosition());

            telemetry.addData("Horizontal Pos:", robot.horizontalExtension.getCurrentPosition());
            telemetry.addData("Horizontal Current", robot.horizontalExtension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Vertical Current L", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Vertical Current R", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Vertical Slider Left: ", robot.verticalSliderLeft.getCurrentPosition());
            telemetry.addData("Vertical Slider Right: ", robot.verticalSliderRight.getCurrentPosition());
            telemetry.update();
        }
    }


}
