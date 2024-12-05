package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class ControlHubTest extends LinearOpMode {
    public Servo s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12;
    public DcMotorEx m1, m2, m3, m4, m5, m6, m7, m8;
    public RevColorSensorV3 cs1, cs2, cs3, cs4, cs5, cs6;


    @Override
    public void runOpMode() throws InterruptedException {
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");
        s6 = hardwareMap.get(Servo.class, "s6");
        s7 = hardwareMap.get(Servo.class, "s7");
        s8 = hardwareMap.get(Servo.class, "s8");
        s9 = hardwareMap.get(Servo.class, "s9");
        s10 = hardwareMap.get(Servo.class, "s10");
        s11 = hardwareMap.get(Servo.class, "s11");
        s12 = hardwareMap.get(Servo.class, "s12");



        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m4 = hardwareMap.get(DcMotorEx.class, "m4");
        m5 = hardwareMap.get(DcMotorEx.class, "m5");
        m6 = hardwareMap.get(DcMotorEx.class, "m6");
        m7 = hardwareMap.get(DcMotorEx.class, "m7");
        m8 = hardwareMap.get(DcMotorEx.class, "m8");


        cs1 = hardwareMap.get(RevColorSensorV3.class, "cs1");
        cs2 = hardwareMap.get(RevColorSensorV3.class, "cs2");
        cs3 = hardwareMap.get(RevColorSensorV3.class, "cs3");
        cs4 = hardwareMap.get(RevColorSensorV3.class, "cs4");
        cs5 = hardwareMap.get(RevColorSensorV3.class, "cs5");
        cs6 = hardwareMap.get(RevColorSensorV3.class, "cs6");


        waitForStart();
        while(opModeIsActive()){
            s1.setPosition(0.5);
            s2.setPosition(1);
            s3.setPosition(0.5);
            s4.setPosition(1);
            s5.setPosition(0.5);
            s6.setPosition(1);
            s7.setPosition(0.5);
            s8.setPosition(1);
            s9.setPosition(0.5);
            s10.setPosition(1);
            s11.setPosition(0.5);
            s12.setPosition(1);
            m1.setPower(1);
            m2.setPower(1);
            m3.setPower(1);
            m4.setPower(1);
            m5.setPower(1);
            m6.setPower(1);
            m7.setPower(1);
            m8.setPower(1);

            telemetry.addData("red", cs1.red());
            telemetry.addData("green", cs1.green());
            telemetry.addData("blue", cs1.blue());
            telemetry.addLine();
            telemetry.addData("red", cs2.red());
            telemetry.addData("green", cs2.green());
            telemetry.addData("blue", cs2.blue());
            telemetry.addLine();
            telemetry.addData("red", cs3.red());
            telemetry.addData("green", cs3.green());
            telemetry.addData("blue", cs3.blue());
            telemetry.addLine();
            telemetry.addData("red", cs4.red());
            telemetry.addData("green", cs4.green());
            telemetry.addData("blue", cs4.blue());
            telemetry.addLine();
            telemetry.addData("red", cs5.red());
            telemetry.addData("green", cs5.green());
            telemetry.addData("blue", cs5.blue());
            telemetry.addLine();
            telemetry.addData("red", cs6.red());
            telemetry.addData("green", cs6.green());
            telemetry.addData("blue", cs6.blue());
            telemetry.addLine();
            telemetry.addData("m1 Position", m1.getCurrentPosition());
            telemetry.addData("m2 Position", m2.getCurrentPosition());
            telemetry.addData("m3 Position", m3.getCurrentPosition());
            telemetry.addData("m4 Position", m4.getCurrentPosition());
            telemetry.addData("m5 Position", m5.getCurrentPosition());
            telemetry.addData("m6 Position", m6.getCurrentPosition());
            telemetry.addData("m7 Position", m7.getCurrentPosition());
            telemetry.addData("m8 Position", m8.getCurrentPosition());

            telemetry.update();

        }
    }
}
