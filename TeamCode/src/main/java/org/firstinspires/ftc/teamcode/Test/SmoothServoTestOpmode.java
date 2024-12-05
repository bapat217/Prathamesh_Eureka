//package org.firstinspires.ftc.teamcode.Test;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.util.SmoothServo;
//
//@Config
//@TeleOp(name = "Dynamic Smooth Servo Movement", group = "Examples")
//public class SmoothServoTestOpmode extends LinearOpMode {
//
//    // Declare the SmoothServo object
//    private SmoothServo smoothServo;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize the servo hardware and SmoothServo object
//        Servo servo = hardwareMap.get(Servo.class, "servo");
//        smoothServo = new SmoothServo(servo, 1.0, 2000); // start at 1.0, duration of 2 seconds
//
//        // Initialize Road Runner drive
//        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        // Wait for the game to start
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Update Road Runner localization
//            drive.updatePoseEstimate();
//
//            // Trigger servo movement with the X button to different positions
//            if (gamepad1.x && !smoothServo.isMoving()) {
//                // Example: Move servo to position 0.0 when `X` is pressed
//                smoothServo.startMovement(0.0);
//            } else if (gamepad1.y && !smoothServo.isMoving()) {
//                // Example: Move servo back to position 1.0 when `Y` is pressed
//                smoothServo.startMovement(1.0);
//            }
//
//            // Update the servo position
//            smoothServo.update();
//
//            // Telemetry data for debugging
//            telemetry.addData("Servo Moving", smoothServo.isMoving());
//            telemetry.update();
//        }
//    }
//}
