package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
public class ThroughBoretest extends LinearOpMode{
    Encoder par;

    @Override
    public void runOpMode() throws InterruptedException {
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Park", par.getPositionAndVelocity().position);
            telemetry.update();
            sleep(500);  // wait for a second before looping again.
        }
    }
}
