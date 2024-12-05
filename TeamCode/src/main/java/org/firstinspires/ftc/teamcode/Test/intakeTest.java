package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.Serializable;
@TeleOp
@Disabled
public class intakeTest extends LinearOpMode {

    public CRServo CRL = null;
    public CRServo CRR = null;

    public DigitalChannel beamBreaker = null;
    public boolean state = false;
    public RevColorSensorV3 cs = null;
    public Servo flapper = null;
    public static double inc = 0;
    public static boolean stop = false;
    public static boolean red_reverse = false;
    public static boolean red_detected = false;
    public static boolean blue_detected = false;
    public static boolean yellow_detected = false;
    public static boolean toggleStart = false;
    public static boolean finish_reverse = false;

    // 0.62 out
    //0.85
    //for red red = > 100 blue < 100
    //for blue red < 100 blue > 100
    //for yello green > 200
    @Override
    public void runOpMode() throws InterruptedException {
        CRL = hardwareMap.get(CRServo.class, "crl");
        CRR = hardwareMap.get(CRServo.class, "crr");
        beamBreaker = hardwareMap.get(DigitalChannel.class, "bb");
        cs = hardwareMap.get(RevColorSensorV3.class, "cs");
        flapper = hardwareMap.get(Servo.class, "flapper");
        Gamepad currentGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();

        flapper.setPosition(0.62);
        stop = false;
        toggleStart = false;
        finish_reverse = false;
        waitForStart();
        while(opModeIsActive()){
            prevGamepad.copy(currentGamepad);


            if(!stop){
                telemetry.addLine("Motor is runnning..");
            CRL.setPower(1);
            CRR.setPower(-1);
            }
            else{
                telemetry.addLine("Motor is stopped..");
                CRR.setPower(0);
                CRL.setPower(0);
            }

            if(prevGamepad.start && !currentGamepad.start){
                toggleStart = !toggleStart;
            }

            if(gamepad1.x){
                CRL.setPower(-1);
                CRR.setPower(1);
            }
//            flapper.setPosition(inc);

            //red
            if((cs.red() > 150 && cs.green() < 200) ){
                telemetry.addLine("red detected");
                red_detected = true;
            }
            //blue
            else if((cs.red() < 100 && cs.blue() > 150)){
                telemetry.addLine("blue detected");
                blue_detected = true;
                flapper.setPosition(0.62) ;

            }
            else if((cs.green() > 200 && cs.red()> 200)){
                telemetry.addLine("yellow detected");
                yellow_detected = true;
//                flapper.setPosition(0.9);

            }
            else{
                telemetry.addLine("Nothing detected");
                stop = false;
                toggleStart = false;
                finish_reverse = false;
                red_reverse = false;
                red_detected = false;
                blue_detected = false;
                yellow_detected = false;
            }

            if(!beamBreaker.getState() && red_detected && !stop){
                if(!red_reverse) {
                    flapper.setPosition(0.9);
                    stop = true;
                }
                else{
                    flapper.setPosition(0.62);
                    stop = false;
                    telemetry.addLine("RED DETECTED BLOCK");
                    telemetry.update();

                }
            }

            if(toggleStart && stop && !finish_reverse){
                CRL.setPower(-0.2);
                CRR.setPower(0.2);
//                sleep(100);
                flapper.setPosition(0.62);
                if(beamBreaker.getState() && red_detected){
                    stop = false;
                    red_reverse = true;
                    finish_reverse= true;
                    telemetry.addLine("Reverse complete");
                    telemetry.update();

                }
                telemetry.addLine("INSIDE REVERSE BLOCK");
                telemetry.update();
            }





            telemetry.addData("Color Values RED ", cs.red());
            telemetry.addData("Color Values BLUE", cs.blue());
            telemetry.addData("Color Values GREEN", cs.green());
            telemetry.addData("flapper values", flapper.getPosition());
            telemetry.addData("flapper values inc ", inc);
            telemetry.addData(" red detected", red_detected);
            telemetry.addData("red reverse", red_reverse);
            telemetry.addData("Beam breaker ", beamBreaker.getState());
            telemetry.addData("stop ", stop);
            telemetry.addData("toggle ", toggleStart);
            telemetry.addData("finish ", finish_reverse);
            telemetry.update();
        }
    }
}
