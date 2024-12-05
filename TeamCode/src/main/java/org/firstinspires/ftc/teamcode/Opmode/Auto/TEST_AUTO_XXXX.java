
package org.firstinspires.ftc.teamcode.Opmode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Sequences.Auto.A_High_Bucket_Pose;
import org.firstinspires.ftc.teamcode.Sequences.Auto.A_Intake_Init;
import org.firstinspires.ftc.teamcode.Sequences.Auto.A_OuTake_Before_Trans;
import org.firstinspires.ftc.teamcode.Sequences.Auto.A_OuTake_Init;
import org.firstinspires.ftc.teamcode.Sequences.Auto.A_SamplePrePick;
import org.firstinspires.ftc.teamcode.Sequences.Auto.A_Sample_OutTake_G;
import org.firstinspires.ftc.teamcode.Sequences.Auto.A_Sample_Pick;
import org.firstinspires.ftc.teamcode.Sequences.Auto.A_Sample_Transfer_G;
import org.firstinspires.ftc.teamcode.Sequences.Auto.A_Score_Reset_Sample;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


@Config
@Autonomous(name = "TEST_AUTO_XXXX", group = "Autonomous")
public class TEST_AUTO_XXXX extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrive drive;
    Intake intake ;
    Outtake outtake;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        outtake = new Outtake(robot);
        intake = new Intake(robot);

        Pose2d initialPose = new Pose2d(-35, -60, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, initialPose);


        //TODO ==================================================== Writing Trajectories ======================================================
        Action myInit = drive.actionBuilder(drive.pose)
                .stopAndAdd(()-> new A_Intake_Init(intake,outtake))
                .stopAndAdd(()->new A_OuTake_Init(intake,outtake))
                .build();


        Action myTraj = drive.actionBuilder(drive.pose)


                .strafeToLinearHeading(new Vector2d(-43, -59), Math.toRadians(0))

                .stopAndAdd(()->new A_High_Bucket_Pose(outtake))
                .waitSeconds(1)
                .stopAndAdd(()->new A_Score_Reset_Sample(outtake,intake))
//                .stopAndAdd(()-> new A_Score_Reset_Sample(outtake,intake))
                .waitSeconds(0.1)
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))


                // TODO SAMPLE FIRST
                .afterTime(0.001,()->new A_SamplePrePick(intake,outtake))
                .strafeToLinearHeading(new Vector2d(23, -59), Math.toRadians(0))

                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))
                .waitSeconds(0.5)
                .stopAndAdd(()->new A_Sample_Pick(intake,outtake))
                .waitSeconds(0.3)
                .stopAndAdd(()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.OPEN))
                .stopAndAdd(()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.CLOSE))

                .waitSeconds(0.7)

                .stopAndAdd(()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.CLOSE))
                .waitSeconds(0.5)
                .stopAndAdd(()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.OPEN))
                .strafeToLinearHeading(new Vector2d(-43, -59), Math.toRadians(0))
                .stopAndAdd(()->new A_High_Bucket_Pose(outtake))
                .waitSeconds(1)
                .stopAndAdd(()->new A_Score_Reset_Sample(outtake,intake))
                .waitSeconds(0.1)
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))

                .strafeToLinearHeading(new Vector2d(-39, -10), Math.toRadians(20))
                .strafeToLinearHeading(new Vector2d(-20, -10), Math.toRadians(20))


                .build();






        if (opModeInInit())
        {
            Actions.runBlocking(new SequentialAction(
//                    P_SamplePick.intakeInit(intake,outtake)
                    myInit
            ));

//            telemetry.addLine("Init");
//            Actions.runBlocking(new SequentialAction(
//                    trajectoryActionInit
//            ));
        }

        waitForStart();
        while (opModeIsActive()) {
    Actions.runBlocking(new SequentialAction(

                myTraj
    ));

        }
    }
}



