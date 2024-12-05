package org.firstinspires.ftc.teamcode.Opmode.Teleop;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.GripperIntakeGlobals;
import org.firstinspires.ftc.teamcode.Hardware.NewIntakeRobotHardware;

@TeleOp
@Config
public class ELEV_CURR_TEST extends LinearOpMode {
    NewIntakeRobotHardware robot = NewIntakeRobotHardware.getInstance();

    public static int poseX = 0;
    public static int poseY = 0;
    public static int poseA = 0;
    public static int poseB = 0;
    public static int poseXH = 20000;
    public static int poseYH = 0;
    public static int poseAH = -1100;
    public static int poseAHH = -1000;
    public static int poseBH = 0;
    public static int poseBHH = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.newIntakeInit(hardwareMap, telemetry);

//        robot.gripper.setPosition(GripperIntakeGlobals.gripperClose);
//        robot.elbow.setPosition(GripperIntakeGlobals.elbowInit);
//        robot.shoulder.setPosition(GripperIntakeGlobals.shoudlerInit);
//        robot.wristServo.setPosition(GripperIntakeGlobals.wristINIT);
//        robot.yawServo.setPosition(GripperIntakeGlobals.yawPick);
        robot.horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.verticalSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.gripperOut.setPosition(Globals.gripperServoOpen);
        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {

                robot.HangerLow.setTargetPosition(robot.HangerLow.getCurrentPosition() + 20);
                robot.HangerLow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.HangerLow.setPower(1);
            }
            if (gamepad1.dpad_down) {
                robot.HangerLow.setTargetPosition(robot.HangerLow.getCurrentPosition() - 20);
                robot.HangerLow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.HangerLow.setPower(1);
            }

            if (gamepad1.x) {

                robot.HangerLow.setTargetPosition(poseX);
                robot.HangerLow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.HangerLow.setPower(1);
            }

            if (gamepad1.y) {
                extendTO(poseY,robot.HangerLow);
            }
            if (gamepad1.a) {
                extendTO(poseA,robot.HangerLow);

            }
            if (gamepad1.b) {
                extendTO(poseB,robot.HangerLow);
            }


            if(gamepad2.dpad_up){
                extendTO(robot.verticalSliderLeft.getCurrentPosition()-20, robot.verticalSliderLeft);
                extendTO(robot.verticalSliderRight.getCurrentPosition()-20, robot.verticalSliderRight);
            }

            if(gamepad2.dpad_down){
                extendTO(robot.verticalSliderLeft.getCurrentPosition()+20, robot.verticalSliderLeft);
                extendTO(robot.verticalSliderRight.getCurrentPosition()+20, robot.verticalSliderRight);
            }
//            if(gamepad2.a){
//                extendTO(poseAH, robot.verticalSliderLeft); // -1100
//                extendTO(poseAH, robot.verticalSliderRight);
//                sleep(1000);
//                extendTO(poseAHH, robot.verticalSliderLeft); // -1000
//                extendTO(poseAHH, robot.verticalSliderRight);
//            }
//            if(gamepad2.b){
//                extendTO(poseBH, robot.verticalSliderLeft); //-100
//                extendTO(poseBH, robot.verticalSliderRight);
//                sleep(1000);
//                extendTO(poseBHH, robot.verticalSliderLeft); // 0
//                extendTO(poseBHH, robot.verticalSliderRight);
//            }
            if(gamepad2.a){
                extendTO(poseAH, robot.verticalSliderLeft); // -1100
                extendTO(poseAH, robot.verticalSliderRight);
                sleep(1000);
                extendTO(poseAH, robot.verticalSliderRight,0);

            }
            if(gamepad2.b){
                extendTO(poseBH, robot.verticalSliderLeft); //-100
                extendTO(poseBH, robot.verticalSliderRight);
                sleep(1000);
                extendTO(poseBH, robot.verticalSliderRight,0);

            }
            if(gamepad2.x){
                extendTO(poseXH, robot.verticalSliderLeft);
                extendTO(poseXH, robot.verticalSliderRight);
            }
            if(gamepad2.y){
                extendTO(poseYH, robot.verticalSliderLeft);
                extendTO(poseYH, robot.verticalSliderRight);
            }
//            robot.verticalSliderRight.setPower(robot.verticalSliderLeft.getPower());


            // TODO ********************************************************************************
            if(gamepad2.left_bumper){
                extendTO(poseYH, robot.verticalSliderLeft);

            }





            //


            telemetry.addData("PIDF", robot.verticalSliderLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Left ver pose------> ", robot.verticalSliderLeft.getCurrentPosition());
            telemetry.addData("Left ver CURRENT---> ", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right ver pose-----> ", robot.verticalSliderRight.getCurrentPosition());
            telemetry.addData("Right ver CURRENT--> ", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Hanger mot pose----> ", robot.HangerLow.getCurrentPosition());
            telemetry.addData("Hanger mot CURRENT->", robot.HangerLow.getCurrent(CurrentUnit.AMPS));




            telemetry.update();

        }

    }
    public static void extendTO (int target, DcMotorEx m){
        m.setTargetPosition(target);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(1);

//        m.setPositionPIDFCoefficients();

    }
    public static void extendTO (int target, DcMotorEx m,double pow){
        m.setTargetPosition(target);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(pow);



//        m.setPositionPIDFCoefficients();

    }


}
