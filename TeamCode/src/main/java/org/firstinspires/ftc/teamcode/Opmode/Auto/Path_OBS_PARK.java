package org.firstinspires.ftc.teamcode.Opmode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.P_SamplePick;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Autonomous
public class Path_OBS_PARK extends LinearOpMode {
    Intake intake;
    Outtake outtake;
    //Drive
    private MecanumDrive drive = null;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(17.5, -64, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap,startPose);

        Action trajectoryAction=drive.actionBuilder(drive.pose)

                .strafeToLinearHeading(new Vector2d(57, -64), Math.toRadians(90))

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
