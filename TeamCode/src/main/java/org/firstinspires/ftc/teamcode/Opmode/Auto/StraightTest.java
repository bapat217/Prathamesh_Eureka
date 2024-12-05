package org.firstinspires.ftc.teamcode.Opmode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class StraightTest extends LinearOpMode {
    //Drive
    private MecanumDrive drive = null;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap,startPose);

        Action trajectoryAction=drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(0, 72), Math.toRadians(90))



                .build();


        if (opModeInInit())
        {
            telemetry.addLine("ROBOT INIT MODE");

        }



        waitForStart();

        Actions.runBlocking(new SequentialAction(
                trajectoryAction
        ));

    }
}
