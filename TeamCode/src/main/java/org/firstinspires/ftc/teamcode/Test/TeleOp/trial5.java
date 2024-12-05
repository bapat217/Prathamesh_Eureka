//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "trial5", group = "Examples")
//public class trial5 extends LinearOpMode {
//
//    private Motor motor1, motor2;
//    private MotorGroup motorGroup;
//
//    public static int target = 0;
//    @Override
//    public void runOpMode() {
//        // Initialize individual motors
//        motor1 = new Motor(hardwareMap, "sliderL", Motor.GoBILDA.BARE);
//        motor2 = new Motor(hardwareMap, "sliderR", Motor.GoBILDA.BARE);
//
//        // Enable encoders for position control
//        motor1.setRunMode(Motor.RunMode.PositionControl);
//        motor2.setRunMode(Motor.RunMode.PositionControl);
//
//        // Reset encoders to start at position 0
//        motor1.resetEncoder();
//        motor2.resetEncoder();
//
//        // Create a MotorGroup
//        motorGroup = new MotorGroup(motor1, motor2);
//
//        // Set PID coefficients for better control (adjust as needed)
//        motor1.setPositionCoefficient(1.0); // Proportional gain
//        motor2.setPositionCoefficient(-1.0);
//
//        // Inform the driver that the robot is ready
//        telemetry.addLine("Ready! Press A or B.");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Set a target position with button A
//            if (gamepad1.a) {
//                target = 1000;
//                motorGroup.setTargetPosition(target); // Move to 1000 ticks
//                motorGroup.set(1); // Set motor power for movement
//            }
//            // Set a target position with button B
//            else if (gamepad1.b) {
//                target = 0;
//                motorGroup.setTargetPosition(target); // Return to starting position
//                motorGroup.set(1); // Set motor power
//            }
//
//            // Stop the motor group if it reaches the target position
//            int positionError = Math.abs(motor1.getCurrentPosition() - target);
//            if (positionError < 10) { // Threshold to stop near target
//                motorGroup.stopMotor();
//            }
//
//            // Display telemetry data for debugging
//            telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
//            telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
////            telemetry.addData("Target Position", motor1.getTargetPosition());
//            telemetry.addData("Position Error", positionError);
//            telemetry.update();
//        }
//    }
//}
