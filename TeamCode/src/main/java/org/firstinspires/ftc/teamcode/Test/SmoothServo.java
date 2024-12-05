package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
public class SmoothServo {
    private Servo servo;
    private double startPosition;
    private double endPosition;
    private double duration;
    private ElapsedTime timer;
    private boolean isMoving;

    public SmoothServo(Servo servo, double startPosition, double duration) {
        this.servo = servo;
        this.startPosition = startPosition;
        this.duration = duration;
        this.timer = new ElapsedTime();
        this.isMoving = false;
        this.servo.setPosition(startPosition);
    }

    // Start the movement with a new target end position
    public void startMovement(double newPosition) {
        this.startPosition = servo.getPosition(); // Set start to current position
        this.endPosition = newPosition;           // Set the new end position
        timer.reset();                            // Reset the timer
        isMoving = true;                          // Set movement flag to true
    }

    // Update method to run within the main loop
    public void update() {
        if (!isMoving) return;

        // Calculate elapsed time and normalize progress
        double elapsedTime = timer.milliseconds();
        double rawProgress = Math.min(elapsedTime / duration, 1.0);

        // Quadratic ease-out interpolation
        double easedProgress = 1 - (1 - rawProgress) * (1 - rawProgress);

        // Calculate interpolated position
        double interpolatedPosition = startPosition + easedProgress * (endPosition - startPosition);

        // Set the servo to the interpolated position
        servo.setPosition(interpolatedPosition);

        // Stop moving once the duration is complete
        if (rawProgress >= 1.0) {
            isMoving = false;
        }
    }

    // Check if the servo is currently moving
    public boolean isMoving() {
        return isMoving;
    }
}
