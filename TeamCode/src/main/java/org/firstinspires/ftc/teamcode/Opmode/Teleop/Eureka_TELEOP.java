package org.firstinspires.ftc.teamcode.Opmode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.Hang;
import org.firstinspires.ftc.teamcode.Sequences.InitSequence;
import org.firstinspires.ftc.teamcode.Sequences.P_SamplePick;
import org.firstinspires.ftc.teamcode.Sequences.ScoreAndResetSequence;
import org.firstinspires.ftc.teamcode.Sequences.SpecimenOuttakeSequence;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config

public class Eureka_TELEOP extends LinearOpMode {

    // TODO Teleop Flags
    public static boolean XTransFlag = true;
    public static boolean isIntake= false;
    public static boolean isOuttake= false;
    public static boolean sample = false;
    public static boolean specimen = false;
    public static boolean transfer = false;
    public static boolean slowMode = false;
    public static boolean inSpeciIntakePose = false;
    public static  boolean specimenOutTakeSeq = true;
    public static boolean ternery = false;
    public static  int elevPodPose = 0;

    // Toggle
    public static boolean R_B_toggle_state= true; // Right bumper 1
    public static boolean L_B_toggle_state= true; // Left bumper 1
    public static boolean R_B_last_state= true; // Right bumper 1
    public static boolean L_B_last_state= true; // Left bumper 1


    // TODO __________________________
    public static double kp = 0.025/*0.0018*/;
    public static double   ki = 0/*0.00001*/;
    public static double kd = 0.0001/*0.00005*/;

    public PIDController elevetorCTRL = null;
    public static int tarGet = 0;
    public RobotHardware robot = RobotHardware.getInstance();
    public static List<Action> runningActions = new ArrayList<>();
    Intake intake = null;
    Outtake outtake = null;

    public static int sliderLeft = 0;
    public static int sliderRight = 0;
    public static double Open = 0;
    public static double Close = 0;
    public static double ColourDistanceOutake = 9;
    MecanumDrive drive = null;
    private float y, x, rx;
    double STRAFE =1, THROTTLE = 1, HEADING = 0.8;
    YawPitchRollAngles botHeading;
    private IMU imu;
    public static boolean isBreak = false;
    public static boolean sample_detected = false;
    public static boolean wrong_detected = false;
    public static boolean not_transfer = false;
    public static boolean specimen_done = false;

    public static boolean done_scoring = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(robot);
        outtake = new Outtake(robot);

//        elevetorCTRL = new PIDController(Globals.kp,Globals.ki,Globals.kd);
//
        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.clutchServo.setPosition(0.3789);
        Globals.test = false;
        while (opModeInInit()) {

            robot.verticalEncoder.reset();
            Actions.runBlocking(InitSequence.InitSequenceAction(intake, outtake));

            telemetry.update();

            // TODO HANGER WOUNDING
            if (gamepad1.dpad_up) {
                extendTO(robot.HangerLow.getCurrentPosition() + 100, robot.HangerLow);
            }
            if (gamepad1.dpad_up) {
                extendTO(robot.HangerLow.getCurrentPosition() - 100, robot.HangerLow);
            }
        }


        waitForStart();

        while (opModeIsActive()) {

            runningActions = updateAction();

            // TODO BOOLEAN FLAGS
            if (robot.horizontalExtension.getCurrentPosition() < 3) {
                XTransFlag = true;
            } else {
                XTransFlag = false;
            }


            // TODO Toggle
            boolean L_B_current_state = gamepad1.left_bumper;
            boolean R_B_current_state = gamepad1.right_bumper;
            if (L_B_current_state && !L_B_last_state) {
                L_B_toggle_state = !L_B_toggle_state;
            }
            if (R_B_current_state && !R_B_last_state) {
                R_B_toggle_state = !R_B_toggle_state;
            }
            L_B_last_state = L_B_current_state;
            R_B_last_state = R_B_current_state;

            // TODO MODE
            // --------------------Sample mode
            if (gamepad2.back) {
                sample = true;
                specimen = false;
            }
            // -------------------Specimen mode
            if (gamepad2.start) {
                sample = false;
                specimen = true;
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> ternery = true),
                        new SleepAction(0.3),
                        new InstantAction(() -> ternery = false)
                ));
            }
            // switch from sample to specimen pose
            if (specimen && gamepad2.start && ternery) {
                runningActions.add(new SequentialAction(
                        ScoreAndResetSequence.readyToSpecimenPick(outtake, intake)
                ));
            }

            // TODO Drive
            // Drive field centric
            driveFieldCentric(-gamepad1.left_stick_x * THROTTLE
                    , -gamepad1.left_stick_y * STRAFE,
                    gamepad1.right_stick_x * HEADING,
                    drive.pose.heading.toDouble()
            );

            if (gamepad1.right_stick_button){
                drive.navxMicro.initialize();
            }


            // Speed controll
            if (gamepad1.left_trigger > 0) {
                THROTTLE = 0.3;
                HEADING = 0.3;
                STRAFE = 0.3;
            } else {
                THROTTLE = 1;
                HEADING = 0.8;
                STRAFE = 1;
            }
            drive.updatePoseEstimate();



            // TODO SAMPLE
            // Intake
            if (gamepad1.left_bumper /*&& sample*/) { // if(gamepad1.left_bumper){
                runningActions.add(new SequentialAction(
                        P_SamplePick.PrePickSequence(intake, outtake) // taking pick pose
//                        P_SamplePick.OutTakeaBeforeTrans(intake,outtake) // taking outtake to before transfer pose
                ));
                if (sample) {
                    runningActions.add(new SequentialAction(
                            P_SamplePick.OutTakeaBeforeTrans(intake, outtake) // taking outtake to before transfer pose
                    ));
                }
                if (specimen) {
                    runningActions.add(new SequentialAction(ScoreAndResetSequence.readyToSpecimenPick(outtake, intake)));
                }
                isIntake = true;
                slowMode = true;
            }

            // Retract and starts transfer
//            if(gamepad1.right_bumper && (robot.colorSensorOut.getDistance(DistanceUnit.MM) <= Globals.ColourDistanceOutake) ) {
            if (gamepad1.right_bumper && sample/*!L_B_toggle_state && (robot.colorSensorOut.getDistance(DistanceUnit.MM) <= Globals.ColourDistanceOutake) && sample*/) {

                runningActions.add(new SequentialAction(
                        new SleepAction(0.3),
                        P_SamplePick.PickSequence(intake, outtake),
//                        new SleepAction(0.4),
                        P_SamplePick.SampleTransferGClosed(intake, outtake),
                        new SleepAction(0.2)
                        // Transfer
//                        P_SamplePick.SampleOutTakeGOpen(intake, outtake),
//                        new SleepAction(0.1),
//                        P_SamplePick.SampleOutTakeGClosed(intake, outtake),
//                        new SleepAction(0.1),
//                        P_SamplePick.SampleTransferGOpen(intake, outtake)
                ));
                transfer = true;
                slowMode = false;
            }

            // Transfer
            if (gamepad1.x && transfer && (robot.horizontalExtension.getCurrentPosition() <= 2) /*&& robot.iShoulder.getPosition() ==Globals.iShoulder_Transfer*/) {
                runningActions.add(new SequentialAction(
//                        new SleepAction(0.5),
                        P_SamplePick.SampleOutTakeGOpen(intake, outtake),
                        new SleepAction(0.1),
                        P_SamplePick.SampleOutTakeGClosed(intake, outtake),
                        new SleepAction(0.1),
                        P_SamplePick.SampleTransferGOpen(intake, outtake),
                        new SleepAction(0.3),
                        ScoreAndResetSequence.BeforeScoreBucketSequence(outtake)
                ));
                transfer = false;

            }


            // X extension extreme pose
            if (gamepad1.dpad_up && sample) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.XExtState(intake, Intake.iXextensionStates.LAST)
                ));
            }
            // X extension mid pose
            if (gamepad1.dpad_down && sample) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.XExtState(intake, Intake.iXextensionStates.MID)
                ));
            }


            // Sample drop pose
//            if ((gamepad1.left_trigger > 0.4) && sample/*R_B_toggle_state && sample*/) {
//                runningActions.add(new
//                        SequentialAction(
//                        ScoreAndResetSequence.BeforeScoreBucketSequence(outtake),
//                        P_SamplePick.intakeInit(intake, outtake)
//                ));
//            }

            // Sample drop and return
            if ((gamepad1.right_trigger > 0.4) && sample/*!R_B_toggle_state && sample*/) {
                runningActions.add(new
                        SequentialAction(ScoreAndResetSequence.ScoreBucketAndReset(outtake, intake),
                        new SleepAction(0.3),
                        P_SamplePick.OutTakeaBeforeTrans(intake, outtake)
                ));
            }


//            // TODO SPECIMEN

            // Pick specimen
            if (gamepad1.right_bumper && specimen) {
                runningActions.add(new SequentialAction(
                                SpecimenOuttakeSequence.SpecimenOuttakeAfterPick(outtake)

                        )
                );
            }


            // Specimen drop and returns
            if (gamepad1.b && specimen) {
                runningActions.add(new SequentialAction(
                        ScoreAndResetSequence.ScoreAndResetSequenceAction(outtake, intake),
                        P_SamplePick.intakeInit(intake, outtake),
                        new InstantAction(() -> inSpeciIntakePose = true)
                ));
            }


//            if (gamepad1.right_trigger > 0.4 && specimen) {
//                runningActions.add(new SequentialAction(
//                        new SleepAction(0.3),
//                        P_SamplePick.PickSequence(intake, outtake),
//                        new SleepAction(0.4),
//                        P_SamplePick.SampleTransferGClosed(intake, outtake),
//                        new SleepAction(0.2)
//                ));
//            }
            // Drop sample to obs
            if (gamepad1.y && specimen) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.sampleDropOBS(intake, outtake),
                        new SleepAction(0.5),
                        P_SamplePick.SampleTransferGClosed(intake, outtake)

                ));
            }
            // X extension extreme pose
            if (gamepad1.dpad_up) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.XExtState(intake, Intake.iXextensionStates.LAST)
                ));
            }
            // X extension mid pose
            if (gamepad1.dpad_down) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.XExtState(intake, Intake.iXextensionStates.MID)
                ));
            }

            // wrist pose
            if (gamepad2.dpad_up) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.wristPose(intake, Intake.iWristStates.POSE_0)
                ));
            }
            if (gamepad2.dpad_down) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.wristPose(intake, Intake.iWristStates.POSE_90)
                ));
            }

            // TODO pick pose upper lift
            if (gamepad2.left_bumper) {
                runningActions.add((new SequentialAction(
                        P_SamplePick.PrePickSequenceUperLift(intake,outtake)
                )));


            }


            // TODO HANGING
            if (gamepad2.a) {
                Actions.runBlocking(new SequentialAction(
                        Hang.HangerINIT(intake, outtake),
                        new SleepAction(0.5),
                        Hang.HangerL(outtake, Outtake.HangerState.OPEN),
                        Hang.HangerELEV(outtake, Outtake.SliderStateOut.PRE_HANG)
                ));
            }
            if (gamepad2.y) {
                elevPodPose = drive.leftFront.getCurrentPosition();
                Actions.runBlocking(new SequentialAction(
                        Hang.HangerINIT(intake, outtake),

//                        new SleepAction(0.5),
                        Hang.HangerLCLUTCH(outtake, Outtake.HangerState.FIRST_HANG,Outtake.ClutchState.HANG_ENGAGED)
                ));
            }

            if (gamepad2.right_trigger > 0.4) {

                Actions.runBlocking(new SequentialAction(
                        Hang.HangerELEV(outtake, Outtake.SliderStateOut.HANG)
                ));
            }



            printTelemetry(telemetry);
            telemetry.update();

        }
    }


    public static List<Action> updateAction(){
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : runningActions) {
//            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
//        runningActions.removeAll(RemovableActions);

        return newActions;
    }

    public void printTelemetry(Telemetry telemetry){
        telemetry.addData("NavX ", TwoDeadWheelLocalizer.robotHeading);
        telemetry.addData("elevPod" , elevPodPose);
        telemetry.addData("O Dist ",robot.colorSensorOut.getDistance(DistanceUnit.MM));
        telemetry.addData("MODE specimen ? -> ",specimen);
        telemetry.addData("Grip Colour Sensor Live:" , robot.colorSensorOut.getDistance(DistanceUnit.MM));
        telemetry.addData("Grip Colour Sensor :" , Globals.ColourDistanceOutake);
        telemetry.addData("is Break:", isBreak);
        telemetry.addData("sample detected:", sample_detected);
        telemetry.addData("wrong detected:", wrong_detected);
        telemetry.addData("Not transfer:", not_transfer);
        telemetry.addData("specimen done:", specimen_done);
        telemetry.addData("Scoring done:", done_scoring);
        telemetry.addLine();
//        telemetry.addData("State: ", robot.beamBreaker.getState());
//        telemetry.addData("Distance Out: ", robot.colorSensorOut.getDistance(DistanceUnit.CM));
//        telemetry.addData("Distance In: ", robot.colorSensorIn.getDistance(DistanceUnit.CM));
//        telemetry.addData("RED IN: ", robot.colorSensorIn.red());
//        telemetry.addData("BLUE IN: ", robot.colorSensorIn.blue());
//        telemetry.addData("GREEN IN: ", robot.colorSensorIn.green());
//        telemetry.addData("RED OUT: ", robot.colorSensorIn.red());
//        telemetry.addData("BLUE OUT: ", robot.colorSensorIn.blue());
//        telemetry.addData("GREEN OUT: ", robot.colorSensorIn.green());
        telemetry.addLine();
//        telemetry.addData("ELbow itnake L:", robot.elbowLeft.getPosition());
//        telemetry.addData("ELbow itnake R:", robot.elbowRight.getPosition());
//        telemetry.addData("wrist intake:", robot.wristServo.getPosition());
//        telemetry.addData("yaw intake:", robot.yawServo.getPosition());
        telemetry.addData("wrist outtake:", robot.wristOut.getPosition());
        telemetry.addData("elbow out L:", robot.elbowLeftOut.getPosition());
        telemetry.addData("elbow out R:", robot.elbowRightOut.getPosition());
        telemetry.addData("gripper:", robot.gripper.getPosition());
//        telemetry.addData("flapper:", robot.flapper.getPosition());
        telemetry.addLine();
        telemetry.addData("STATE GRIPPER: ", outtake.gripperState);
        telemetry.addData("State ELBOW: ", outtake.elbowStateOut);
        telemetry.addData("State wrist: ", outtake.wristStateOut);
//        telemetry.addData("Intake State ELBOW: ", intake.elbowState);
        telemetry.addData("Out State wrist: ", outtake.wristStateOut);
        telemetry.addData("Out State elbow: ", outtake.elbowStateOut);
//        telemetry.addData("In State flapper: ", intake.flapperState);
        telemetry.addData("Slider State vertical: ", outtake.sliderStateOut);
//        telemetry.addData("Slider State horizontal: ", intake.sliderState);
        telemetry.addLine();
       telemetry.addLine();
        telemetry.addData("left Front current: ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left Back current: ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front current: ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Back current: ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Motor L current:", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Motor R current:", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Horizontal Motor current :", robot.horizontalExtension.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Motor curr R pose:", robot.verticalSliderRight.getCurrentPosition());
        telemetry.addData("Vertical Motor curr L pose:", robot.verticalSliderLeft.getCurrentPosition());
        telemetry.addData("Horizontal Motor:", robot.horizontalExtension.getCurrentPosition());

        telemetry.addData("Vertical Motor L Power:", robot.verticalSliderLeft.getPower());
        telemetry.addData("Vertical Motor R Power:", robot.verticalSliderRight.getPower());

        telemetry.addData("enco val" , robot.verticalEncoder.getPosition());
        telemetry.addData("tarGet ",tarGet);
        telemetry.addData("PIDF L", robot.verticalSliderLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("PIDF R", robot.verticalSliderRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

    }


    public void elevatorPID(int target1){
        elevetorCTRL.setPID(kp,ki,kd);
        double power = elevetorCTRL.calculate(robot.verticalSliderRight.getCurrentPosition(), target1);
        robot.verticalSliderLeft.setPower(power);
        robot.verticalSliderRight.setPower(power);
    }
    public void driveFieldCentric(double x, double y, double rx, double botHeading) {

        double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading));
        double rotY = (x * Math.sin(botHeading) + y * Math.cos(botHeading));

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);


        double frontLeftPower = (rotY - rotX + rx) / denominator;
        double backLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY + rotX - rx) / denominator;
        double backRightPower = (rotY - rotX - rx) / denominator;

        drive.leftFront.setPower(frontLeftPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightBack.setPower(backRightPower);
        drive.rightFront.setPower(frontRightPower);

    }
    public static void extendTO (int target, DcMotorEx m){
        m.setTargetPosition(target);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(1);

    }




}
