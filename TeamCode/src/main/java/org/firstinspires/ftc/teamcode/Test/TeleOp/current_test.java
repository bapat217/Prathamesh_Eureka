package org.firstinspires.ftc.teamcode.Test.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;


@TeleOp
@Config
public class current_test extends LinearOpMode {
    public static RobotHardware robot = RobotHardware.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()){
            robot.init(hardwareMap, telemetry);

            robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.init(hardwareMap);
        }


        while (opModeIsActive()){
            if(gamepad1.dpad_up){
                extendTo(robot.verticalSliderLeft.getCurrentPosition()+30,robot.verticalSliderLeft);
                extendTo(robot.verticalSliderRight.getCurrentPosition()+30,robot.verticalSliderRight);
            }
            if(gamepad1.dpad_up){
                extendTo(robot.verticalSliderLeft.getCurrentPosition()-30,robot.verticalSliderLeft);
                extendTo(robot.verticalSliderRight.getCurrentPosition()-30,robot.verticalSliderRight);
            }

            if(gamepad1.x){
                extendTo(robot.horizontalExtension.getCurrentPosition()+30, robot.horizontalExtension);
            }
            if(gamepad1.y){
                extendTo(robot.horizontalExtension.getCurrentPosition()-30, robot.horizontalExtension);
            }

            telemetry.addData("left m verticle", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right m verticle", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left m verticle pose", robot.verticalSliderLeft.getCurrentPosition());
            telemetry.addData("right m verticle pose", robot.verticalSliderRight.getCurrentPosition());

            telemetry.addData("x ext", robot.horizontalExtension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("x EXT pose", robot.horizontalExtension.getCurrentPosition());
            telemetry.update();


        }
    }

    public static void extendTo(int target, DcMotorEx m){
       m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       m.setTargetPosition(target);
       m.setPower(1);
    }
}
