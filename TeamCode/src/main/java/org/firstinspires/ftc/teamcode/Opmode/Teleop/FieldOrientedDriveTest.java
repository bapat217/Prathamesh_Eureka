package org.firstinspires.ftc.teamcode.Opmode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@TeleOp

public class FieldOrientedDriveTest extends LinearOpMode{
        RobotHardware robot = null;
    MecanumDrive drive =null;
    public void runOpMode() throws InterruptedException {

        while (opModeInInit()){
            robot.init(hardwareMap);
            drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        }
        waitForStart();

        while (opModeIsActive()){

            driveFieldCentric(gamepad1.left_stick_x
            , gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    drive.pose.heading.toDouble()
                    );


        }
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
