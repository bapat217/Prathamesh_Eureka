package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
@Disabled
public class MotorTest extends LinearOpMode {
    public DcMotorEx m1, m2, m3, m4;
    public Servo s1;
    public static double inc = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
     m1 = hardwareMap.get(DcMotorEx.class, "m1");
     m2 = hardwareMap.get(DcMotorEx.class, "m2");
     m3 = hardwareMap.get(DcMotorEx.class, "m3");
     m4 = hardwareMap.get(DcMotorEx.class, "m4");


     waitForStart();
     while(opModeIsActive()){

         if(gamepad1.b){
             m1.setPower(0);
             m2.setPower(0);
             m3.setPower(0);
             m4.setPower(0);
         }
         if(gamepad1.a){
             m1.setPower(1);
             m2.setPower(1);
             m3.setPower(1);
             m4.setPower(1);
         }

         telemetry.addData("Current M1", m1.getCurrent(CurrentUnit.AMPS));
         telemetry.addData("Current M2", m2.getCurrent(CurrentUnit.AMPS));
         telemetry.addData("Current M2", m3.getCurrent(CurrentUnit.AMPS));
         telemetry.addData("Current M2", m4.getCurrent(CurrentUnit.AMPS));
         telemetry.addData("Velocity M1", m1.getVelocity());
         telemetry.addData("Velocity M2", m2.getVelocity());
         telemetry.addData("Velocity M2", m3.getVelocity());
         telemetry.addData("Velocity M2", m4.getVelocity());

         telemetry.update();

     }
    }
}
