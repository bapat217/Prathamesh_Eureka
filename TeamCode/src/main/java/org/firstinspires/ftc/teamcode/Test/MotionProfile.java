//package org.firstinspires.ftc.teamcode.Test;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.util.MovingStatistics;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.navigation.TrapezoidalProfile;
//
//@TeleOp(name = "SliderTrapezoidalFTC", group = "Linear Opmode")
//public class MotionProfile extends LinearOpMode {
//
//    // Define the motor for the slider mechanism
//    private DcMotorEx sliderMotor;
//
//    // Trapezoidal Profile Parameters
//    private static final double MAX_VELOCITY = 2000;  // ticks per second
//    private static final double MAX_ACCELERATION = 1000; // ticks per second^2
//    private static final double TARGET_POSITION = 5000; // Target position in ticks
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize the slider motor
//        sliderMotor = hardwareMap.get(DcMotorEx.class, "sliderMotor");
//
//        // Reset motor encoder and set mode to run using encoder
//        sliderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        sliderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        // Wait for start
//        waitForStart();
//
//        // Create trapezoidal motion profile
//        TrapezoidalProfile trapezoidalProfile = new TrapezoidalProfile(
//                TARGET_POSITION,  // target position (in ticks)
//                MAX_VELOCITY,     // max velocity (ticks/second)
//                MAX_ACCELERATION  // max acceleration (ticks/second^2)
//        );
//
//        // Track time to drive profile
//        ElapsedTime timer = new ElapsedTime();
//
//        while (opModeIsActive()) {
//            double elapsedTime = timer.seconds();
//
//            // Get the target position at the current time
//            double targetPosition = trapezoidalProfile.getPosition(elapsedTime);
//
//            // Set motor target position
//            sliderMotor.setTargetPosition((int) targetPosition);
//            sliderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            sliderMotor.setPower(1.0);  // Power the motor
//
//            // Display telemetry data
//            telemetry.addData("Elapsed Time", elapsedTime);
//            telemetry.addData("Target Position", targetPosition);
//            telemetry.addData("Motor Position", sliderMotor.getCurrentPosition());
//            telemetry.update();
//
//            // Stop once we reach the end of the profile
//            if (elapsedTime > trapezoidalProfile.duration()) {
//                break;
//            }
//        }
//
//        // Stop the motor after completing the motion
//        sliderMotor.setPower(0);
//    }
//}
