package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
@Disabled
public class ServoTest extends LinearOpMode {
        public double val = 0.5;
         
    public Servo s1;
    public Servo s2;
    AnalogInput a1;

    @Override
    public void runOpMode() throws InterruptedException {
        s1 = hardwareMap.get(Servo.class,"s1");
        s2 = hardwareMap.get(Servo.class,"s2");
//        a1 = hardwareMap.get(AnalogInput.class, "a1");

        while(opModeInInit()){
            s1.setPosition(0.5);
            s2.setPosition(0.5);
            telemetry.addData("Servo Position 1", s1.getPosition());
            telemetry.addData("Servo Position 2", s2.getPosition());
            telemetry.update();
        }
        waitForStart();

        while(opModeIsActive()){
            s1.setPosition(val);
            s2.setPosition(val);
            telemetry.addData("Servo Position 1", s1.getPosition());
            telemetry.addData("Servo Position 2", s2.getPosition());
//            telemetry.addData("Analog Value", a1.getVoltage());
            telemetry.update();
        }

    }
}