//package org.firstinspires.ftc.teamcode.Opmode.Teleop;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Config
//@TeleOp(name = "FollowerPID", group = "Examples")
//public class FollowerPID extends LinearOpMode {
//
//    private Motor motor1, motor2;
//    private MotorGroup motorGroup;
//
//    DcMotorEx m;
//
//
//
//    public static int target = 0;
//
//    public static double kp = 0.001/*0.0018*/, ki = 0.0001/*0.00001*/, kd = 0.0001/*0.00005*/, kf=0;
//    public PIDFController controller = null;
//
//
//    @Override
//    public void runOpMode() {
//        // Initialize individual motors
//        motor1 = new Motor(hardwareMap, "sliderL", Motor.GoBILDA.BARE);
//        m = hardwareMap.get(DcMotorEx.class, "sliderR");
////        m = (DcMotorEx) new Motor(hardwareMap, "sliderR", Motor.GoBILDA.BARE);
//
//        // Enable encoders for position control
//        motor1.setRunMode(Motor.RunMode.PositionControl);
//        motor2.setRunMode(Motor.RunMode.PositionControl);
//
//        controller = new PIDFController(kp, ki, kd,kf);
//
//        // Reset encoders to start at position 0
//        motor1.resetEncoder();
//        motor2.resetEncoder();
//
//        // Create a MotorGroup
//        motorGroup = new MotorGroup(motor1, (Motor) m);
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
//
//            }
//            // Set a target position with button B
//            else if (gamepad1.b) {
//                target = -1000;
//
//            }
//
//            motorGroup.set(clutchPID(target));
//
//            // Display telemetry data for debugging
//            telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
//            telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
////            telemetry.addData("Target Position", motor1.getTargetPosition());
//
//            telemetry.update();
//        }
//    }
//
//    public double clutchPID(int target){
//        controller.setPIDF(kp,ki,kd,kf);
//        double power = controller.calculate(motor1.getCurrentPosition(),target);
//        return power;
//        motorGroup.setTargetPosition();
////        m1.setPower(power);
////        m2.setPower(power);
//    }
//}
