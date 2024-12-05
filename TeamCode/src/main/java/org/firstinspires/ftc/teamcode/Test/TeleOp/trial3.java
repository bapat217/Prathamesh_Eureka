//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@Config
//@TeleOp(name = "Trial 3", group = "Examples")
//public class trial3 extends LinearOpMode {
//    public static double kP = 20;
//    public static int target = 0;
//    public static double kV = 0.7;
//    public static double ki = 0.7;
//    public static double kd = 0.7;
//    public static double ks = 0.7;
//    private MotorGroup motorGroup;
//
//    @Override
//    public void runOpMode() {
//        // Initialize individual motors
//        Motor motor1 = new Motor(hardwareMap, "sliderL", Motor.GoBILDA.BARE);
//        Motor motor2 = new Motor(hardwareMap, "sliderR", Motor.GoBILDA.BARE);
//
//        // Enable encoders for position control
//        motor1.setRunMode(Motor.RunMode.PositionControl);
//        motor2.setRunMode(Motor.RunMode.PositionControl);
//
//        // Create a MotorGroup
//        motorGroup = new MotorGroup(motor1, motor2);
//        motorGroup.setRunMode(Motor.RunMode.VelocityControl);
//        motorGroup.setVeloCoefficients(kP, ki, kd);
//        motorGroup.setFeedforwardCoefficients(ks, kV);
//
//
//        motor1.setPositionCoefficient(1.0); // Proportional gain
//        motor2.setPositionCoefficient(1.0);
//        while (opModeInInit()){
//            motor1.resetEncoder();
//            motor2.resetEncoder();
//        }
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                    target = 0;
////                motorGroup.setTargetPosition(1000);
////                motorGroup.set(0.5);
//            } else if (gamepad1.b) {
//
//                target = 1000;
////                motorGroup.setTargetPosition(0);
////                motorGroup.set(0.5);
////                motorGroup.
//            }
//            motorGroup.setTargetPosition(target);
////            motorGroup.set(1);
//
//            // Display telemetry data
//            telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
//            telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
//            telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
//            telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
//
////            telemetry.addData("Target Position", motor1.getTargetPosition());
//            telemetry.update();
//        }
//    }
//}
