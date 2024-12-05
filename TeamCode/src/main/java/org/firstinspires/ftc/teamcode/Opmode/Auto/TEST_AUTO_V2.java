
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
@Autonomous(name = "TEST_AUTO_V2", group = "Autonomous")
public class TEST_AUTO_V2 extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrive drive;
    Intake intake ;
    Outtake outtake;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        outtake = new Outtake(robot);
        intake = new Intake(robot);

        Pose2d initialPose = new Pose2d(-35, -60, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);


        //TODO ==================================================== Writing Trajectories ======================================================
        Action myInit = drive.actionBuilder(drive.pose)
                .stopAndAdd(()-> new A_Intake_Init(intake,outtake))
                .stopAndAdd(()->new A_OuTake_Init(intake,outtake))
                .build();


        Action myTraj = drive.actionBuilder(drive.pose)


                .strafeToLinearHeading(new Vector2d(-54, -43), Math.toRadians(60))

                .stopAndAdd(()->new A_High_Bucket_Pose(outtake))
                .waitSeconds(1)
                .stopAndAdd(()->new A_Score_Reset_Sample(outtake,intake))
//                .stopAndAdd(()-> new A_Score_Reset_Sample(outtake,intake))
                .waitSeconds(0.1)
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))


                // TODO SAMPLE FIRST
                .stopAndAdd(()->new A_SamplePrePick(intake,outtake))
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))
                .waitSeconds(0.5)
                .stopAndAdd(()->new A_Sample_Pick(intake,outtake))
                .waitSeconds(0.3)
                .stopAndAdd(()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.OPEN))
                .stopAndAdd(()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.CLOSE))

                .waitSeconds(0.4)

                .stopAndAdd(()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.CLOSE))
                .waitSeconds(0.5)
                .stopAndAdd(()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.OPEN))
                .strafeToLinearHeading(new Vector2d(-54.1, -49), Math.toRadians(60))
                .stopAndAdd(()->new A_High_Bucket_Pose(outtake))
                .waitSeconds(1)
                .stopAndAdd(()->new A_Score_Reset_Sample(outtake,intake))
                .waitSeconds(0.1)
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))

                // TODO SECOND SAMPLE - Middle
                .strafeToLinearHeading(new Vector2d(-54, -43.1), Math.toRadians(90))

//                .stopAndAdd(()->new A_SamplePrePick(intake,outtake))
                .stopAndAdd(()->new A_SamplePrePick(intake,outtake, Intake.iXextensionStates.AUTO_MID_YELLOW))
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))
                .waitSeconds(0.5)
                .stopAndAdd(()->new A_Sample_Pick(intake,outtake,Intake.iXextensionStates.AUTO_MID_YELLOW))
                .waitSeconds(0.3)
                .stopAndAdd(()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.OPEN))
                .stopAndAdd(()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.CLOSE))
                .waitSeconds(0.4)
                .stopAndAdd(()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.CLOSE))
                .waitSeconds(0.5)
                .stopAndAdd(()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.OPEN))
                .strafeToLinearHeading(new Vector2d(-54.11, -49), Math.toRadians(60))
                .stopAndAdd(()->new A_High_Bucket_Pose(outtake))
                .waitSeconds(1)
                .stopAndAdd(()-> new A_Score_Reset_Sample(outtake,intake))
                .waitSeconds(0.1)
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))

                // TODO third Sample
                .strafeToLinearHeading(new Vector2d(-54, -43.11), Math.toRadians(100))
                // Sample
                .stopAndAdd(()->new A_SamplePrePick(intake,outtake))
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))
                .waitSeconds(0.5)
                .stopAndAdd(()->new A_Sample_Pick(intake,outtake))
                .waitSeconds(0.3)
                .stopAndAdd(()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.OPEN))
                .stopAndAdd(()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.CLOSE))
                .waitSeconds(0.4)
                .stopAndAdd(()->new A_Sample_OutTake_G(intake,outtake, Outtake.GripperState.CLOSE))
                .waitSeconds(0.5)
                .stopAndAdd(()->new A_Sample_Transfer_G(intake,outtake, Intake.iGripperStates.OPEN))
                .strafeToLinearHeading(new Vector2d(-54, -49.001), Math.toRadians(60))
                .stopAndAdd(()->new A_High_Bucket_Pose(outtake))
                .waitSeconds(1)
                .stopAndAdd(()-> new A_Score_Reset_Sample(outtake,intake))
                .waitSeconds(0.1)
                .stopAndAdd(()->new A_OuTake_Before_Trans(intake,outtake))

                .build();







        Action trajectoryAction0 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(25, -60), Math.toRadians(90))
                .waitSeconds(0.2)
                .stopAndAdd(()->outtake.gripperAction(Outtake.GripperState.CLOSE))
                .waitSeconds(0.3)
                .stopAndAdd(()->outtake.sliderOutAction(Outtake.SliderStateOut.S_HIGH))
                .waitSeconds(0.4)
                .stopAndAdd(outtake.elbowOutAction(Outtake.ElbowStateOut.AFTER_PICK))
                .stopAndAdd(()->outtake.wristOutAction(Outtake.WristStateOut.AFTER_PICK))
                .waitSeconds(0.1)
                .stopAndAdd(()->outtake.sliderOutAction(Outtake.SliderStateOut.HIGH))
                .waitSeconds(0.3)

                // To SPECIMEN HOOK
//                .stopAndAdd(()->outtake.sliderOutAction(Outtake.SliderStateOut.HIGH))
                .waitSeconds(0.4)
                .stopAndAdd(()->outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PLACE))
                .stopAndAdd(()-> outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PLACE))

//               outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PLACE),
                .waitSeconds(0.5)
                .stopAndAdd(()->outtake.gripperAction(Outtake.GripperState.OPEN))


                .build();


//        Action trajectoryActionInit = drive.actionBuilder(drive.pose)
//                        // OutTake Init
//                        .stopAndAdd(()->outtake.wristOutAction(Outtake.WristStateOut.SPECIMEN_PICK))
//                        .stopAndAdd(()->outtake.elbowOutAction(Outtake.ElbowStateOut.SPECIMEN_PICK))
//
//                        .stopAndAdd(()->outtake.gripperAction(Outtake.GripperState.OPEN))// close
//                        .stopAndAdd(()->outtake.sliderOutAction(Outtake.SliderStateOut.INIT))
//
//                        // Intake Init
//                .stopAndAdd(()->intake.iGripper(Intake.iGripperStates.INIT))
//                .stopAndAdd(()->intake.iWrist(Intake.iWristStates.POSE_0))
//                .stopAndAdd(()->intake.iElbow(Intake.iElobowStates.INIT))
//                .stopAndAdd(()->intake.iShoulde(Intake.iShoulderStates.INIT))
//                .stopAndAdd(()->intake.iWiper(Intake.iWiperStates.INIT))
//                .stopAndAdd(()->intake.iXextension(Intake.iXextensionStates.INIT))
//                .waitSeconds(5)
//
//                .build();




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





//                .strafeToLinearHeading(new Vector2d(5, -32), Math.toRadians(90))
//                .stopAndAdd(new SequentialAction(
//
//                        P_SamplePick.PickSequence(intake,outtake),
//                        new SleepAction(1),
//
//                        // PICKING
//                        P_SamplePick.PickSequence(intake, outtake),
////                        new SleepAction(0.4),
//                        P_SamplePick.SampleTransferGClosed(intake, outtake),
//                        new SleepAction(1),
//
//                        // TRANSFERING
//                        P_SamplePick.SampleOutTakeGOpen(intake, outtake),
//                        new SleepAction(0.1),
//                        P_SamplePick.SampleOutTakeGClosed(intake, outtake),
//                        new SleepAction(0.1),
//                        P_SamplePick.SampleTransferGOpen(intake, outtake),
//                        new SleepAction(0.3),
//                        ScoreAndResetSequence.BeforeScoreBucketSequence(outtake),
//
//                        // DROPPING N RETURN
//                        ScoreAndResetSequence.BeforeScoreBucketSequence(outtake),
//                        P_SamplePick.intakeInit(intake, outtake)
//
//
//                ))
//                .waitSeconds(5)

//                .stopAndAdd(()->new P_SamplePick.PickSequence(outtake)

//                .waitSeconds(1)

//TODO - Specimen Droping

//
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
////                .waitSeconds(0.05)
//                .stopAndAdd(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHRUNG, 0))
//                .stopAndAdd(()-> new AutoSpecimenDrop(outtake, elevator))
//                .waitSeconds(0.4)
//                .strafeToLinearHeading(new Vector2d(10, -38), Math.toRadians(90))
//                .stopAndAdd(()-> robot.Shoulder.setPosition(0.45))
//                .waitSeconds(0.05)
//                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
//                .waitSeconds(0.5)
//                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                .stopAndAdd(()-> new InitSeq(outtake, elevator))
//
//
//                .stopAndAdd(()-> new RackCommand(outtake, OuttakeSubsystem.RackState.SAMPLE_PICK))
//                .stopAndAdd(()-> new ArmCommand(outtake, OuttakeSubsystem.ArmState.SAMPLE_PICK))
//                .stopAndAdd(()-> new WristCommand(outtake, OuttakeSubsystem.WristState.WRIST_RIGHT_DIAGONAL))
//                .afterTime(0.8,()-> new RackExtendCommand(outtake, OuttakeSubsystem.RackExtendState.MAX_POS))
//                .afterTime(0.6,()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
//
//
//                // First Sample Picking and Dropping
//                .strafeToLinearHeading(new Vector2d(44.5, -42.5), Math.toRadians(60))
//                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
//                .waitSeconds(0.05)
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
//                .waitSeconds(0.15)
//                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))
//                .strafeToLinearHeading(new Vector2d(44.5, -48), Math.toRadians(-60))
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
//                .waitSeconds(0.15)
//
//
//                // Second Sample Picking and Dropping
//                .strafeToLinearHeading(new Vector2d(50.5, -41.5), Math.toRadians(60))
//                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
//                .waitSeconds(0.15)
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
//                .waitSeconds(0.15)
//                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))
//                .strafeToLinearHeading(new Vector2d(53.5, -48), Math.toRadians(-60))
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
//                .waitSeconds(0.15)
//
//
////                 Third Sample Picking and Dropping
//                .strafeToLinearHeading(new Vector2d(61, -40.5), Math.toRadians(60))
//
//                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
//                .waitSeconds(0.05)
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
//                .waitSeconds(0.15)
//
//                .stopAndAdd(()-> new ShoulderCommand(outtake, OuttakeSubsystem.ShoulderState.AUTO_SAMPLE_PRE_PICK))
//
//                .strafeToLinearHeading(new Vector2d(55, -48), Math.toRadians(-60))
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_OPEN))
//                .waitSeconds(0.15)
//
//                // Specimen PrePick
//                .stopAndAdd(()-> new InitSeq(outtake, elevator))
//                .waitSeconds(0.15)
//                .stopAndAdd(()-> new SpecimenPickSeq(outtake, elevator))
//                .waitSeconds(0.15)
//                .strafeToLinearHeading(new Vector2d(38, -44), Math.toRadians(90))
//                // Specimen Pick
//                .strafeToLinearHeading(new Vector2d(38, -62), Math.toRadians(90))
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
//                .waitSeconds(0.05)
//                .stopAndAdd(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHRUNG, 0))
//                .waitSeconds(0.2)
//
//                // Specimen Hang
//                .afterTime(0.7,()-> new AutoSpecimenDrop(outtake, elevator))
//
//                .strafeToLinearHeading(new Vector2d(7, -40), Math.toRadians(90))
//                .stopAndAdd(()-> robot.Shoulder.setPosition(0.45))
//                .waitSeconds(0.05)
//                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
//                .waitSeconds(0.5)
//                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                .stopAndAdd(()-> new InitSeq(outtake, elevator))
//
//
//                // Third Specimen Pick
//                .stopAndAdd(()-> new SpecimenPickSeq(outtake, elevator))
//                .strafeToLinearHeading(new Vector2d(37, -44), Math.toRadians(90))
//
//                // Specimen Pick
//                .strafeToLinearHeading(new Vector2d(37, -62), Math.toRadians(90))
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
//                .waitSeconds(0.05)
//                .stopAndAdd(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHRUNG, 0))
//                .waitSeconds(0.2)
//                .afterTime(0.7,()-> new AutoSpecimenDrop(outtake, elevator))
//
//
//
//                // Specimen Hang
//                .strafeToLinearHeading(new Vector2d(9, -40), Math.toRadians(90))
//                .stopAndAdd(()-> robot.Shoulder.setPosition(0.45))
//                .waitSeconds(0.05)
//                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
//                .waitSeconds(0.5)
//                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                .stopAndAdd(()-> new InitSeq(outtake, elevator))
//
//                // Fourth Specimen Pick
//                .stopAndAdd(()-> new SpecimenPickSeq(outtake, elevator))
//                .strafeToLinearHeading(new Vector2d(37, -44), Math.toRadians(90))
//
//                // Specimen Pick
////                .strafeToLinearHeading(new Vector2d(38, -59), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(37, -62), Math.toRadians(90))
//                .stopAndAdd(()-> new GripperCommand(outtake, OuttakeSubsystem.GripperState.GRIP_CLOSE))
//                .waitSeconds(0.05)
//                .stopAndAdd(() -> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.HIGHRUNG, 0))
//                .waitSeconds(0.2)
//                .afterTime(0.7,()-> new AutoSpecimenDrop(outtake, elevator))
//
//
//
//                // Specimen Hang
//                .strafeToLinearHeading(new Vector2d(4, -40), Math.toRadians(90))
//                .stopAndAdd(()-> robot.Shoulder.setPosition(0.45))
//                .waitSeconds(0.05)
//                .stopAndAdd(()-> new ElevatorCommand(elevator, ElevatorSubsytem.ElevateState.SPECIMENHANGED, 0))
//                .waitSeconds(0.5)
//                .stopAndAdd(()-> robot.Gripper.setPosition(Globals.gripperOpen))
//                .stopAndAdd(()-> new InitSeq(outtake, elevator))
//
//
//
//                .strafeToLinearHeading(new Vector2d(420, -157), Math.toRadians(90))


