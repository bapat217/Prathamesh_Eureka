//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Config
//@TeleOp(name = "Trial 2", group = "Examples")
//public class trial2 extends LinearOpMode {
//    Motor motor1 ;
//    Motor motor2 ;
//    public static double Ckp = 0/*0.0018*/, Cki = 0/*0.00001*/, Ckd = 0/*0.00005*/, Ckf=0;
//    public PIDFController controller = null;
//    public static double kP = 20;
//    public static int target = 0;
//    public static double kV = 0.7;
//    public static double ki = 0.7;
//    public static double kd = 0.7;
//    public static double ks = 0.7;
//    public static double m1Power = 1;
//    public static double m2Power = -1;
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
//        // Create a MotorGroup
//        motorGroup = new MotorGroup(motor1, motor2);
//        motorGroup.setRunMode(Motor.RunMode.VelocityControl);
//        motorGroup.setVeloCoefficients(kP, ki, kd);
//        motorGroup.setFeedforwardCoefficients(ks, kV);
//
//        controller = new PIDFController(Ckp, Cki, Ckd,Ckf);
//
//
//        motor1.setPositionCoefficient(m1Power); // Proportional gain
//        motor2.setPositionCoefficient(m2Power);
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
//            clutchPID(target);
//
//            // Display telemetry data
//            telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
//            telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
//            telemetry.addData("Motor1 Position", motor1.getCurrentPosition());
//            telemetry.addData("Motor2 Position", motor2.getCurrentPosition());
////            telemetry.addData("m1 current", );
//
////            telemetry.addData("Target Position", motor1.getTargetPosition());
//            telemetry.update();
//        }
//    }
//    public void clutchPID(int target){
//        controller.setPIDF(Ckp,Cki,Ckd,Ckf);
//        double power = controller.calculate(motor1.getCurrentPosition(),target);
//        motorGroup.set(power);
//    }
//}
