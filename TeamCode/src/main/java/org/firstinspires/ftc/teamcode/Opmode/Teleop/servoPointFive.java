package org.firstinspires.ftc.teamcode.Opmode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Control with Buttons", group = "Examples")
public class servoPointFive extends LinearOpMode {
    private Servo myServo;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        myServo = hardwareMap.get(Servo.class, "myServo");

        // Wait for the start of the op mode
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Button A sets position to 0.5
            if (gamepad1.a) {
                myServo.setPosition(0.5);
            }

            // Button B sets position to 0.0
            else if (gamepad1.b) {
                myServo.setPosition(0.0);
            }

            // Button X sets position to 1.0
            else if (gamepad1.x) {
                myServo.setPosition(1.0);
            }

            // Display the servo's current position
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.update();

            // Allow other processes to run
            idle();
        }
    }
}
