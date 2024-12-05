package org.firstinspires.ftc.teamcode.Test;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;




@Disabled

@TeleOp
@Config
public class ServoPID extends LinearOpMode{
    Servo leftS;
    Servo rightS;

    Servo wrist;
    Servo grip;
    DigitalChannel beambreaker;

    public static double shoulderPick=0.1;
    public static double shoulderPlace=0.75;

    public static double wristPos=0.5;
    public static double wristPick=0.68;
    public static double wristDrop=0.089;

    public static double gripOpen=0.938;
    public static double gripClose=0.39;

    public static double prevPos=0;
    public static double targetPos=0;
    public static double speed=0.01;

    public static double kp = 0, ki = 0, kd = 0;

    PIDController controller = null;


    @Override
    public void runOpMode() throws InterruptedException {
        leftS=hardwareMap.get(Servo.class,"ls");
        rightS=hardwareMap.get(Servo.class,"rs");
        wrist=hardwareMap.get(Servo.class,"wt");
        grip=hardwareMap.get(Servo.class,"gp");
        beambreaker = hardwareMap.get(DigitalChannel.class, "bb");
        controller = new PIDController(kp, ki, kd);
        while (opModeInInit()){
            setShoulder(0.5);
            wrist.setPosition(0.5);
            grip.setPosition(0.5);
        }
        waitForStart();
        while (opModeIsActive()) {

            //TODO SHOULDER-POS
            if(gamepad1.dpad_up){
//                shoulderPos+=0.001;
                setShoulder(shoulderPick);
            } else if (gamepad1.dpad_down) {
//                shoulderPos-=0.001;
                setShoulder(shoulderPlace);
            }

            //TODO WRIST-POS
            if(gamepad1.a){
//                wrist.setPosition(wristPick);
                servoPID(wristPick, wrist.getPosition());
            } else if (gamepad1.b) {
//                wrist.setPosition(wristDrop);
                servoPID(wristDrop, wrist.getPosition());

            }

            //TODO GRIPPER-POS
            if(gamepad1.right_bumper){
                grip.setPosition(gripOpen);
            } else if (gamepad1.left_bumper) {
                grip.setPosition(gripClose);
            }

            if(gamepad1.back){
                setShoulder(shoulderPlace);
                wrist.setPosition(wristDrop);
            } else if (gamepad1.start) {
                setShoulder(shoulderPick);
                wrist.setPosition(wristPick);
            }


            if(beambreaker.getState()) {
                telemetry.addData("Beambreaker", "ON");
            } else {
                telemetry.addData("Beambreaker", "OFF");
            }



            //TODO TELEMETRY
//            telemetry.addData("Shoulder", shoulderPos);
//            telemetry.addData("Wrist", wristPos);
//            telemetry.addData("Gripper", gripPos);
//            telemetry.addData("Beambreaker", beambreaker.getState());

            telemetry.update();
        }


    }

    public void setShoulder(double pos){
        leftS.setPosition(pos);
        rightS.setPosition(pos);
    }
    public void setShoulder(double pos,double speed){
        targetPos= speed*pos + prevPos*(1-speed);
        leftS.setPosition(targetPos);
        rightS.setPosition(targetPos);

    }

    public double servoPID(double targetPos,double currentPos){
        controller.setPID(kp, ki,kd);
        double correction =  controller.calculate(targetPos,currentPos);
        if(correction > 1){
            correction = 1;
        }
        if(correction < 0){
            correction = 0;
        }
        return correction;
    }



    public void hardwareServo( Servo servo,String name){
        servo=hardwareMap.get(Servo.class,name);
    }



}
