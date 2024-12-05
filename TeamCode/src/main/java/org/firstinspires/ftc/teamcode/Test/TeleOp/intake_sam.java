//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Continuous Servo Control", group = "Examples")
//public class intake_sam extends LinearOpMode {
//
//    private CRServo continuousServo;
//
//    @Override
//    public void runOpMode() {
//        // Initialize the continuous rotation servo
//        continuousServo = hardwareMap.get(CRServo.class, "intakeR");
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Forward movement
//            if (gamepad1.dpad_up) {
//                continuousServo.setPower(1.0); // Full speed forward
//            }
//            // Reverse movement
//            else if (gamepad1.dpad_down) {
//                continuousServo.setPower(-1.0); // Full speed reverse
//            }
//            // Stop movement
//            else {
//                continuousServo.setPower(0.0); // Stop
//            }
//
//            // Telemetry for debugging
//            telemetry.addData("Servo Power", continuousServo.getPower());
//            telemetry.update();
//        }
//    }
//}
