
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
import org.firstinspires.ftc.teamcode.InstantCommand.IntakeGripCommand;
import org.firstinspires.ftc.teamcode.InstantCommand.SliderCommand;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


@Config
@Autonomous
public class AutoK extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrive drive;
    Intake intake ;
    Outtake outtake;

    public static double gripOpenTime=0.5;
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

                .afterTime(0.01,()->new A_High_Bucket_Pose(outtake))
                .strafeToLinearHeading(new Vector2d(-43, -59), Math.toRadians(0))
                //TODO PRELOAD DROP
                .waitSeconds(0.5) //remove this later
                .stopAndAdd(()->new A_Score_Reset_Sample(outtake,gripOpenTime))
                .waitSeconds(0.3)//0.1

                //ROBOT INIT
                .afterTime(0.01,()->new SliderCommand(outtake, Outtake.SliderStateOut.INIT) )
                .afterTime(0.01,()->new A_SamplePrePick(intake,outtake))     //INTAKE INIT  READY FOR PICK
                .afterTime(0.01,()->new A_OuTake_Before_Trans(intake,outtake)) //DELIVERY INIT
                .afterTime(0.4,()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.OPEN))


                // TODO OBSERVATION ZONE PICK
                .strafeToLinearHeading(new Vector2d(23, -59), Math.toRadians(0))
                .stopAndAdd(()->new IntakeGripCommand(intake, Intake.iGripperStates.CLOSE))
                .waitSeconds(0.2)


                //todo transfer start
                .afterTime(0.1,()->new A_Sample_Pick(intake))//intake just before transfer pre transfer
                .afterTime(0.1,()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.CLOSE)) //intake transfer pose
                .afterTime(0.2,()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.CLOSE)) //delivery tranfer pose
                .waitSeconds(0.4)
                .afterTime(0.3,()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.CLOSE)) //
                .afterTime(0.5,()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.OPEN))


                //TODO DROP ALLIANCE YELLOW
                .afterTime(0.8,()->new A_High_Bucket_Pose(outtake))

                .strafeToLinearHeading(new Vector2d(-43, -59), Math.toRadians(0)) // was 43 made 40
                .waitSeconds(0.3)
                .stopAndAdd(()->new A_Score_Reset_Sample(outtake,gripOpenTime))
                .waitSeconds(0.2)//0.1
                //ROBOT INIT


                .afterTime(0.01,()->new SliderCommand(outtake, Outtake.SliderStateOut.INIT) )
                .afterTime(0.01,()->new A_SamplePrePick(intake,outtake,1))
//                .afterTime(0.01,()->new A_SamplePrePick(intake,outtake))     //INTAKE INIT  READY FOR PICK
                .afterTime(0.01,()->new A_OuTake_Before_Trans(intake,outtake)) //DELIVERY INIT
                .afterTime(0.01,()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.OPEN,1))


                .stopAndAdd(()-> new A_Intake_Init(intake,outtake))
                .stopAndAdd(()->new A_OuTake_Init(intake,outtake))
//                .strafeToLinearHeading(new Vector2d(-39, -10), Math.toRadians(20))
//                .strafeToLinearHeading(new Vector2d(-20, -10), Math.toRadians(20))

                // TODO Right Yellow


                // TODO Mid Yellow

                // TODO Last Yellow

                // TODO Park




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





