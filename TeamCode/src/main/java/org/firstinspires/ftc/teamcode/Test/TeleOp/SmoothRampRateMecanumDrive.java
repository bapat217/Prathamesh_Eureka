package org.firstinspires.ftc.teamcode.Test.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.InitSequence;
import org.firstinspires.ftc.teamcode.Sequences.SpecimenIntakeSequence;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Smooth Ramp Rate Mecanum Drive", group = "Sample")
@Config
public class SmoothRampRateMecanumDrive extends LinearOpMode {

    // Ramp rate limit - change in power per update (between 0 and 1)
    public static double rampRate = 0.02;

    // Current power levels for each motor
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double backLeftPower = 0;
    private double backRightPower = 0;
    MecanumDrive drive = null;
    public static double X_STICK = 1.1;


    public RobotHardware robot = RobotHardware.getInstance();
    public static List<Action> runningActions = new ArrayList<>();
    Intake intake = null;
    Outtake outtake = null;

    public static int sliderLeft = 0;
    public static int sliderRight = 0;
    public static double Open = 0;
    public static double Close = 0;
    private float y, x, rx;
    double STRAFE =1, THROTTLE = 1, HEADING = 1;
    YawPitchRollAngles botHeading;
    private IMU imu;


    @Override
    public void runOpMode() {
        // Initialize motors
        robot.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        // Set motor directions if needed

        intake = new Intake(robot);
        outtake = new Outtake(robot);
//
        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeInInit()){
            Actions.runBlocking(InitSequence.InitSequenceAction(intake, outtake));
        }


        waitForStart();

        while (opModeIsActive()) {
            runningActions = updateAction();
            // Get gamepad inputs for mecanum drive
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.right_stick_x * X_STICK; // Strafe, with scaling for accuracy
            double rx = gamepad1.left_stick_x; // Rotation

            // Calculate target powers for mecanum drive
            double targetFrontLeftPower = y + x + rx;
            double targetFrontRightPower = y - x - rx;
            double targetBackLeftPower = y - x + rx;
            double targetBackRightPower = y + x - rx;

            // Apply ramp rate to each motor's target power

            frontLeftPower = MecanumDrive.rampToTarget(frontLeftPower, targetFrontLeftPower, rampRate);
            frontRightPower = MecanumDrive.rampToTarget(frontRightPower, targetFrontRightPower, rampRate);
            backLeftPower = MecanumDrive.rampToTarget(backLeftPower, targetBackLeftPower, rampRate);
            backRightPower = MecanumDrive.rampToTarget(backRightPower, targetBackRightPower, rampRate);

            // Set the motor powers
            drive.leftFront.setPower(frontLeftPower);
            drive.rightFront.setPower(frontRightPower);
            drive.rightBack.setPower(backLeftPower);
            drive.leftBack.setPower(backRightPower);

            //----------------- DRIVE CONTROL --------- -----------------------

//            if(robot.colorSensor.getDistance(DistanceUnit.CM) < 7){
            if(gamepad1.start){
                runningActions.add(new SequentialAction(
//                                new InstantAction(()->outtake.gripperAction(Outtake.GripperState.CLOSE)),
//                                SpecimenOuttakeSequence.SpecimenOuttakeSequenceAction(outtake)
                        )
                );
            }
            if(gamepad1.a){
//                runningActions.add(ScoreAndResetSequence.ScoreAndResetSequenceAction(outtake));
            }
            if(gamepad1.b){
                runningActions.add(SpecimenIntakeSequence.SpecimenIntakeSequenceAction(outtake));
            }
            if(gamepad1.x){
//                runningActions.add(ScoreAndResetSequence.ScoreAndResetSequenceAction2(outtake));

//                runningActions.add(intake.rollerAction(Intake.RollerState.ON));
            }


            if(gamepad1.left_bumper){
                robot.gripper.setPosition(Open);

            }
            if(gamepad1.right_bumper){
                robot.gripper.setPosition(Close);
            }
            if(gamepad1.left_trigger>0){

            }
            if(gamepad1.right_trigger>0){

            }

            if(gamepad1.dpad_left){

            }
            if(gamepad1.dpad_right){

            }
            if(gamepad1.dpad_up){

            }
            if(gamepad1.dpad_down){

            }

            // ------------------  OPERATOR CONTROL  -------------------------

            if(gamepad1.dpad_left){

            }
            if(gamepad1.dpad_right){

            }
            if(gamepad1.dpad_up){

            }
            if(gamepad1.dpad_down){

            }

            telemetry.addData("Distancce: ", robot.colorSensorOut.getDistance(DistanceUnit.CM));
//            telemetry.addData("Heading: ", botHeading.getYaw());
            telemetry.addData("STATE GRIPPER: ", outtake.gripperState);
            telemetry.addData("State ELBOW: ", outtake.elbowStateOut);
            telemetry.addData("State wrist: ", outtake.wristStateOut);
            telemetry.addData("Vertivcal Motor:", robot.verticalSliderRight.getCurrentPosition());
            telemetry.addData("Vertivcal Motor:", robot.verticalSliderLeft.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("left Front current: ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left Back current: ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Front current: ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Back current: ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            runningActions = updateAction();












            // Telemetry for debugging
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }


    }

    public static List<Action> updateAction(){
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : runningActions) {
//            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
//        runningActions.removeAll(RemovableActions);
        return newActions;
    }

}
