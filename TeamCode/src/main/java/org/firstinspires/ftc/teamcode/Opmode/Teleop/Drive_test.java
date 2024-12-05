package org.firstinspires.ftc.teamcode.Opmode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.Teleop.InitSequence;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config

public class Drive_test extends LinearOpMode {


    // TODO __________________________
    public static double kp = 0.025/*0.0018*/;
    public static double   ki = 0/*0.00001*/;
    public static double kd = 0.0001/*0.00005*/;

    public PIDController elevetorCTRL = null;
    public static int tarGet = 0;
    public RobotHardware robot = RobotHardware.getInstance();
    public static List<Action> runningActions = new ArrayList<>();
    Intake intake = null;
    Outtake outtake = null;

    public static int sliderLeft = 0;
    public static int sliderRight = 0;
    public static double Open = 0;
    public static double Close = 0;
    public static double ColourDistanceOutake = 9;
    MecanumDrive drive = null;
    private float y, x, rx;
    double STRAFE =1, THROTTLE = 1, HEADING = 0.8;
    YawPitchRollAngles botHeading;
    private IMU imu;
    public static boolean isBreak = false;
    public static boolean sample_detected = false;
    public static boolean wrong_detected = false;
    public static boolean not_transfer = false;
    public static boolean specimen_done = false;

    public static boolean done_scoring = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(robot);
        outtake = new Outtake(robot);

//
        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.clutchServo.setPosition(0.3789);
        Globals.test = false;
        while (opModeInInit()) {

            robot.verticalEncoder.reset();
            Actions.runBlocking(InitSequence.InitSequenceAction(intake, outtake));

            telemetry.update();

        }

        waitForStart();

        while (opModeIsActive()) {

            runningActions = updateAction();


            // TODO Drive
            // Drive field centric
            driveFieldCentric(-gamepad1.left_stick_x * THROTTLE
                    , -gamepad1.left_stick_y * STRAFE,
                    gamepad1.right_stick_x * HEADING,
                    drive.pose.heading.toDouble()
            );

            if (gamepad1.right_stick_button){
                drive.navxMicro.initialize();
            }


            // Speed controll
            if (gamepad1.left_trigger > 0) {
                THROTTLE = 0.3;
                HEADING = 0.3;
                STRAFE = 0.3;
            } else {
                THROTTLE = 1;
                HEADING = 0.8;
                STRAFE = 1;
            }
            drive.updatePoseEstimate();
            printTelemetry(telemetry);
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

    public void printTelemetry(Telemetry telemetry){

        telemetry.addLine();
        telemetry.addData("LeftFront", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("LeftBack", drive.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RightFront", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RightBAck", drive.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Heading", TwoDeadWheelLocalizer.robotHeading);
    }



    public void driveFieldCentric(double x, double y, double rx, double botHeading) {

        double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading));
        double rotY = (x * Math.sin(botHeading) + y * Math.cos(botHeading));

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);


        double frontLeftPower = (rotY - rotX + rx) / denominator;
        double backLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY + rotX - rx) / denominator;
        double backRightPower = (rotY - rotX - rx) / denominator;

        drive.leftFront.setPower(frontLeftPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightBack.setPower(backRightPower);
        drive.rightFront.setPower(frontRightPower);

    }




}
