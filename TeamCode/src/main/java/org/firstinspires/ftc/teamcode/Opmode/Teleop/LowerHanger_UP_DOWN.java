package org.firstinspires.ftc.teamcode.Opmode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class LowerHanger_UP_DOWN extends LinearOpMode {

    private DcMotorEx motor = null;

    @Override
    public void runOpMode() {
        // Initialize the motor
        motor = hardwareMap.get(DcMotorEx.class, "hangLow");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            handleGamepadControls();

            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void handleGamepadControls() {
        if (gamepad1.dpad_up) {
            moveMotorBy(50); // Increment by 50 ticks
        } else if (gamepad1.dpad_down) {
            moveMotorBy(-50); // Decrement by 50 ticks
        }
    }

    private void moveMotorBy(int delta) {
        int targetPosition = motor.getCurrentPosition() + delta;
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(1.0);
    }
}
