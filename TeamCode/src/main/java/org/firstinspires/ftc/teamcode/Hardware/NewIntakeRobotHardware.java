package org.firstinspires.ftc.teamcode.Hardware;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class NewIntakeRobotHardware {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    //TODO  ---------------------- INTAKE ------------------------
    public CRServo intakeServoLeft = null;
    public CRServo intakeServoRight = null;

    public Servo shoulder = null;
    public Servo elbow = null;
    public Servo twist = null;

    public Servo wristServo = null;
    public Servo yawServo = null;
    public Servo gripper = null;
    public DigitalChannel beamBreaker = null;
    public RevColorSensorV3 colorSensorOut = null;
    public RevColorSensorV3 colorSensorIn = null;
    public DcMotorEx horizontalExtension = null;

    //TODO ----------------IMUs --------------
    public IntegratingGyroscope gyro;
    public NavxMicroNavigationSensor navxMicro;
    public  ElapsedTime timer = new ElapsedTime();

    //TODO --------------------- OUTTAKE --------------------------

    public Servo gripperOut = null;
    public Servo wristOut = null;
    public ServoImplEx elbowLeftOut = null;
    public ServoImplEx elbowRightOut = null;

    public Servo clutchServo = null;
    public DcMotorEx verticalSliderLeft = null;
    public DcMotorEx verticalSliderRight = null;
    public Motor.Encoder verticalEncoder = null;
    public DcMotorEx HangerLow = null;

    //TODO ----------------- ROBOT HARDWARE SETUP --------------------
    private static NewIntakeRobotHardware instance;

    public static NewIntakeRobotHardware getInstance(){
        if( instance == null){
            instance = new NewIntakeRobotHardware();
        }
        return instance;
    }

    //TODO ------------------- INITIALIZE ROBOT HARDWARE -----------------------
    public void newIntakeInit(HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //TODO --------------- INTAKE HARDWAREMAP ------------------
      //  intakeServoLeft = hardwareMap.get(CRServo.class, "intakeL");
        //intakeServoRight = hardwareMap.get(CRServo.class, "intakeR");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        elbow = hardwareMap.get(Servo.class, "elbow");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        twist = hardwareMap.get(Servo.class, "twist");
        yawServo = hardwareMap.get(Servo.class, "yaw");
        gripper = hardwareMap.get(Servo.class, "grip");
//        beamBreaker = hardwareMap.get(DigitalChannel.class,  "bb");
//        colorSensorOut = hardwareMap.get(RevColorSensorV3.class,  "csO");
//        colorSensorIn = hardwareMap.get(RevColorSensorV3.class,  "csI");
        horizontalExtension = hardwareMap.get(DcMotorEx.class, "horizontal");
        horizontalExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        //TODO -------------- OUTTAKE HARDWAREMAP -----------------------
        gripperOut = hardwareMap.get(Servo.class, "gripper");
        wristOut = hardwareMap.get(Servo.class, "wristOut");
        elbowLeftOut = hardwareMap.get(ServoImplEx.class, "leftOut");
        elbowRightOut = hardwareMap.get(ServoImplEx.class, "rightOut");
        clutchServo = hardwareMap.get(Servo.class, "clutch");
        verticalSliderLeft = hardwareMap.get(DcMotorEx.class, "sliderL");
        verticalSliderRight = hardwareMap.get(DcMotorEx.class, "sliderR");
        HangerLow = hardwareMap.get(DcMotorEx.class, "hangLow");
        verticalEncoder= new MotorEx(hardwareMap,"sliderL").encoder;
    }

    public void NewIntakeinit(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        gripper = hardwareMap.get(Servo.class, "s1");
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
