//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "trial4", group = "Examples")
//public class trial4 extends LinearOpMode {
//
//    private Motor motor1, motor2;
//    private MotorGroup motorGroup;
//
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
//        // Create MotorGroup with two motors
//        motorGroup = new MotorGroup(motor1, motor2);
//
//        // Optional: Set PID coefficients for smoother control
//        motor1.setPositionCoefficient(1.0); // Proportional gain
//        motor2.setPositionCoefficient(1.0);
//
//
//
//        // Optional: Reset encoders
//        motor1.resetEncoder();
//        motor2.resetEncoder();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                // Move motors to position 1000 encoder ticks
//                motor1.setTargetPosition(10000); // Target in encoder ticks
//                motorGroup.set(0.5); // Set power to move to position
//            } else if (gamepad1.b) {
//                // Move motors back to position 0
////                setTargetPosition(motorGroup.setTargetPosition(1000));
//
//                motor1.setTargetDistance(1000); // Target in encoder ticks
//                motor1.setRunMode(Motor.RunMode.PositionControl);
//                motorGroup.set(0.5); // Set  Set power
//            }
//
//            // Display encoder data
//            telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
//            telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
////            telemetry.addData("Target Position", motor1.getTargetPosition());
//            telemetry.update();
//        }
//    }
//}
