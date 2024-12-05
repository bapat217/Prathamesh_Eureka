package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
@Config
public class clutchTest extends LinearOpMode {
    MecanumDrive drive;
    public DcMotorEx m1;
    public DcMotorEx m2;
    FtcDashboard dashboard;


    Motor.Encoder e;

    public static double kp = 0/*0.0018*/, ki = 0/*0.00001*/, kd = 0/*0.00005*/, kf=0;
    public static int tolerance = 50;
    public PIDFController controller = null;

    public static int target = 200;
    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotorEx.class, "sliderL");
        m2 = hardwareMap.get(DcMotorEx.class, "sliderR");

        e= new MotorEx(hardwareMap,"leftBack").encoder;


        controller = new PIDFController(kp, ki, kd,kf);

        while (opModeInInit()){

            dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            e.reset();
//            drive.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Encoder ", e.getPosition());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){


//            m1.setPower(clutchPID(target));
//            m2.setPower(clutchPID(target))q;

            if(!(Math.abs((m2.getCurrentPosition() - target)) <= tolerance )){
                clutchPID(target);
            }

            dashboard.getTelemetry().addData("KP",kp);
            dashboard.getTelemetry().addData("KI",ki);
            dashboard.getTelemetry().addData("KD",kd);
            dashboard.getTelemetry().addData("KF",kf);
            dashboard.getTelemetry().addData("Motor Power 1",m1.getPower());
            dashboard.getTelemetry().addData("Motor Power 2",m2.getPower());
            dashboard.getTelemetry().update();

            telemetry.addData("Current left m1", m1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current right m2", m2.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine(" VELOCITY");
            telemetry.addData("Velocity left m1", m1.getVelocity());
            telemetry.addData("Velocity right m2", m2.getVelocity());
//            telemetry.addData("POWER left", m1.getPower());
//            telemetry.addData("POWER right", m2.getPower());
            telemetry.addLine(" PID STUFF ");
            telemetry.addData("Encoder Value", drive.leftFront.getCurrentPosition());
            telemetry.addData("Error", controller.getPositionError());
            telemetry.addData("At setpoint", controller.atSetPoint());
            telemetry.addData("target", target);
            telemetry.addData("Tolerance",tolerance);
            telemetry.update();

        }
    }

    public void clutchPID(int target){
        controller.setPIDF(kp,ki,kd,kf);

        double power = controller.calculate(drive.leftFront.getCurrentPosition(),target);
        m1.setPower(power);
        m2.setPower(power);
    }







}
