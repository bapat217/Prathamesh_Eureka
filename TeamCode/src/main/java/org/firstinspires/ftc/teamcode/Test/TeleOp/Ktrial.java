//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "ktrial", group = "Examples")
//public class Ktrial extends LinearOpMode {
//
//    private Motor motor1, motor2;
//    Motor.Encoder m1,m2;
//    private MotorGroup motorGroup;
//
//    @Override
//    public void runOpMode() {
//        // Initialize individual motors
//        motor1 = new Motor(hardwareMap, "sliderL", Motor.GoBILDA.BARE);
//        m1= new MotorEx(hardwareMap,"sliderL").encoder;
//
//        motor2 = new Motor(hardwareMap, "sliderR", Motor.GoBILDA.BARE);
//        m2= new MotorEx(hardwareMap,"sliderR").encoder;
//
//
//
//
//        // Enable encoders for position control
////        motor1.setRunMode(Motor.RunMode.PositionControl);
////        motor2.setRunMode(Motor.RunMode.PositionControl);
//
//        // Create MotorGroup with two motors
//        motorGroup = new MotorGroup(motor1, motor2);
//
//        // Optional: Set PID coefficients for smoother control
////        motor1.setPositionCoefficient(1.0); // Proportional gain
////        motor2.setPositionCoefficient(1.0);
//
//
//
//        motorGroup.resetEncoder();
////        // Optional: Reset encoders
////        motor1.resetEncoder();
////        motor2.resetEncoder();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // reset the encoder
////            motor1.resetEncoder();
////
////// the current position of the motor
////            int pos = motor1.getCurrentPosition();
////
////// get the current velocity
////            double[] velocity = motor1.getVeloCoefficients(); // only for MotorEx
////            double corrected = motor1.getCorrectedVelocity();
////
////// grab the encoder instance
////            Motor.Encoder encoder = motor1.encoder;
////
////// get number of revolutions
////            double revolutions = encoder.getRevolutions();
////
////// set the distance per pulse to 18 mm / tick
////            encoder.setDistancePerPulse(18.0);
////            motor1.setDistancePerPulse(18.0); // also an option
////
////// get the distance traveled
////            double distance = encoder.getDistance();
////            distance = motor1.getDistance(); // also an option
////
/////** USEFUL FEATURES **/
////
////// you can set the encoder of the motor to a different motor's
////// encoder
//////            motor1.encoder = other_motor.encoder;
////
////// sometimes the encoder needs to be reset completely
////// through hardware
////            motor1.stopAndResetEncoder();
//
//
//            if (gamepad1.a) {
//                // Move motors to position 1000 encoder ticks
//
//                motorGroup.setTargetPosition(-1000); // Target in encoder ticks
//                motorGroup.setRunMode(Motor.RunMode.PositionControl);
//                motorGroup.set(0.5); // Set power to move to position
//            } else if (gamepad1.b) {
//
//
//                motorGroup.setTargetPosition(0); // Target in encoder ticks
//                motorGroup.setRunMode(Motor.RunMode.PositionControl);
//                motorGroup.set(0.5); // Set  Set power
//            }
//
//            // Display encoder data
//            telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
//            telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
//            telemetry.addData("Motor groupPosition",motorGroup.getPositions());
////            telemetry.addData("Target Position", motor1.getTargetPosition());
//            telemetry.update();
//        }
//    }
//}
