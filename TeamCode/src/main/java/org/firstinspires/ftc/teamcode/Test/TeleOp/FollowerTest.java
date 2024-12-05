package org.firstinspires.ftc.teamcode.Test.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Opmode.Teleop.FollowerMotorClass;

@TeleOp
@Config
public class FollowerTest extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    public static int pos= 0, pow=-1, tolerance =1, offset = 100;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        FollowerMotorClass fm = new FollowerMotorClass(robot.verticalSliderLeft, robot.verticalSliderRight);
        robot.verticalSliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.y){
                fm.followMotor(pos, pow, tolerance, offset);
            }
            if(gamepad1.a){
                fm.followMotor(0, pow, tolerance, offset);
            }
            fm.updateElevator();

            telemetry.addData("pos", pos);
            telemetry.addData("pow left", robot.verticalSliderLeft.getPower());
            telemetry.addData("pow right", robot.verticalSliderRight.getPower());
            telemetry.addData("current left", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("current right", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }

    }
}
