package org.firstinspires.ftc.teamcode.Hardware;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    //TODO  ---------------------- INTAKE ------------------------

    public Servo iGripper = null;
    public Servo iWrist = null;
    public Servo iElbow = null;

    public Servo iShoulder = null;
    public Servo iWiper = null;
    public RevColorSensorV3 colorSensorOut = null;
    public RevColorSensorV3 colorSensorIn = null;
    public DcMotorEx horizontalExtension = null;

    //TODO ----------------IMUs --------------

    public  ElapsedTime timer = new ElapsedTime();

    //TODO --------------------- OUTTAKE --------------------------

    public Servo gripper = null;
    public Servo wristOut = null;
    public Servo twist = null;
    public ServoImplEx elbowLeftOut = null;
    public ServoImplEx elbowRightOut = null;

    public Servo clutchServo = null;
    public DcMotorEx verticalSliderLeft = null;
    public DcMotorEx verticalSliderRight = null;
    public Motor.Encoder verticalEncoder = null;
    public DcMotorEx HangerLow = null;


    //TODO ----------------- ROBOT HARDWARE SETUP --------------------
    private static RobotHardware instance;

    public static RobotHardware getInstance(){
        if( instance == null){
            instance = new RobotHardware();
        }
        return instance;
    }

    //TODO ------------------- INITIALIZE ROBOT HARDWARE -----------------------
    public void init(HardwareMap hardwareMap, Telemetry telemetry){
//        MotorGroup motorgrp1 = new MotorGroup(verticalSliderLeft,verticalSliderRight);

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //TODO --------------- INTAKE HARDWAREMAP ------------------
        iGripper= hardwareMap.get(Servo.class, "grip");
        iWrist= hardwareMap.get(Servo.class, "wrist");
        iElbow = hardwareMap.get(Servo.class, "elbow");
        iShoulder = hardwareMap.get(Servo.class, "shoulder");
        iWiper = hardwareMap.get(Servo.class, "yaw");
        colorSensorOut = hardwareMap.get(RevColorSensorV3.class,  "csO");
        colorSensorIn = hardwareMap.get(RevColorSensorV3.class,  "csI");
        horizontalExtension = hardwareMap.get(DcMotorEx.class, "horizontal");
        horizontalExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO -------------- OUTTAKE HARDWAREMAP -----------------------
        gripper = hardwareMap.get(Servo.class, "gripper");
        wristOut = hardwareMap.get(Servo.class, "wristOut");
        twist = hardwareMap.get(Servo.class, "twist");
        elbowLeftOut = hardwareMap.get(ServoImplEx.class, "leftOut");
        elbowRightOut = hardwareMap.get(ServoImplEx.class, "rightOut");
        clutchServo = hardwareMap.get(Servo.class, "clutch");
        verticalSliderLeft = hardwareMap.get(DcMotorEx.class, "sliderL");
        verticalSliderRight = hardwareMap.get(DcMotorEx.class, "sliderR");


        HangerLow = hardwareMap.get(DcMotorEx.class, "hangLow");
        verticalEncoder= new MotorEx(hardwareMap,"sliderL").encoder;

        //TODO -------------- IMUs ----------------------------




        verticalSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        verticalSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HangerLow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangerLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
//        flapper = hardwareMap.get(Servo.class, "s1");
    }

    public static double getDistance(RobotHardware robot){
         return robot.colorSensorOut.getDistance(DistanceUnit.MM);
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
