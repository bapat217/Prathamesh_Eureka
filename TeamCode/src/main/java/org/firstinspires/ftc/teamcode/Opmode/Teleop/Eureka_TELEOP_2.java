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
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.Teleop.Hang;
import org.firstinspires.ftc.teamcode.Sequences.Teleop.InitSequence;
import org.firstinspires.ftc.teamcode.Sequences.Teleop.P_SamplePick;
import org.firstinspires.ftc.teamcode.Sequences.Teleop.ScoreAndResetSequence;
import org.firstinspires.ftc.teamcode.Sequences.Teleop.SpecimenOuttakeSequence;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config

public class Eureka_TELEOP_2 extends LinearOpMode {

    // TODO Teleop Flags

    // Elevator extended or not
    public static boolean elevatorExtended = false;
    public static boolean specTestVar = false;

    // Specimen detected in outTake gripper or not
    public static boolean specimenSpotted = false;
    public static boolean specimenSpottedTest = false;
    public static boolean isIntake= false;
    public static boolean sample = false;
    public static boolean specimen = false;
    public static boolean transfer = false;
    public static boolean slowMode = false;
    public static boolean inSpeciIntakePose = false;
    public static boolean ternery = false;

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
    MecanumDrive drive = null;
    double STRAFE =1, THROTTLE = 1, HEADING = 0.8;
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




        // TODO INIT LOOP
        while (opModeInInit()) {
            // TODO RESET encoders
            robot.verticalEncoder.reset();
            robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // TODO Run Init Sequence
            Actions.runBlocking(InitSequence.InitSequenceAction(intake, outtake));
            telemetry.update();

            // TODO HANGER WOUNDING - (Reverse)
            if (gamepad1.dpad_up) {
                extendTO(robot.HangerLow.getCurrentPosition() + 100, robot.HangerLow);
            }
            if (gamepad1.dpad_up) {
                extendTO(robot.HangerLow.getCurrentPosition() - 100, robot.HangerLow);
            }
            // To reset lower hanger motor encoder
            if(gamepad1.x){
                robot.HangerLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // TODO Init - Reset flags
            resetFlags();


            // TODO LATER ################################
            robot.clutchServo.setPosition(0.3789);

        }


        // TODO OpMOde loop ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        waitForStart();

        while (opModeIsActive()) {

            // Update actions once in loop
            runningActions = updateAction();

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
//                        new InstantAction(() -> specimenSpottedTest = true),  // To be tested
                        new InstantAction(() -> ternery = true),
                        new SleepAction(0.3),
                        new InstantAction(() -> ternery = false)

                ));
            }
            // switch from sample to specimen pose
            if (specimen && gamepad2.start && ternery) {
                runningActions.add(new SequentialAction(
                        ScoreAndResetSequence.readyToSpecimenPick(outtake, intake),
                        new InstantAction(()->specTestVar = true)
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


            // Speed control
            if (gamepad1.left_trigger > 0.3  || slowMode) {
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

            /*
            SAMPLE INTAKE
            In Sample mode --> Pre Pick sample
            In Specimen mode --> Pre Pick sample
            But if in sample mode and driver wants to throw in obs zone --> also can be done.
            */
            if (gamepad1.left_bumper  /*&& !elevatorExtended*/ ) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.PrePickSequence(intake, outtake) // taking pick pose
                    ));
                if (sample) {
                    runningActions.add(new SequentialAction(
                            P_SamplePick.OutTakeaBeforeTrans(intake, outtake) // taking outtake to before transfer pose
                    ));
                }
                if (specimen) { // to check
                    runningActions.add(new SequentialAction(
                            ScoreAndResetSequence.readyToSpecimenPick(outtake, intake)
                    ));
                }
                runningActions.add(new SequentialAction(
                        new InstantAction(()->isIntake = true),
                        new InstantAction(()->slowMode = true)
                    ));
            }
            /*
            If Pre pick pose is taken outside submersible
            ---> Controller have button to make intake up Lifted
            */
            if(gamepad2.right_bumper){
                runningActions.add(new SequentialAction(
                        // Uplifting intake
                        P_SamplePick.PrePickSequenceUperLift(intake,outtake)
                ));
            }
            /*
            PICK Sample and ready to do next step ->
            ------> Can transfer to score sample
            ------> Can take pre drop pose for OBS zone
            */
            if (gamepad1.right_bumper && sample/*!L_B_toggle_state && (robot.colorSensorOut.getDistance(DistanceUnit.MM) <= Globals.ColourDistanceOutake) && sample*/) {
                // If in sample mode --> ready to transfer
                if(sample){
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.3),
                            P_SamplePick.PickSequence(intake, outtake),
                            P_SamplePick.SampleTransferGClosed(intake, outtake),
                            new SleepAction(0.2)
                    ));
                }
                // If in specimen mode --> ready to drop to obs zone
                if(specimen){
                    runningActions.add(new SequentialAction(
                            P_SamplePick.sampleDropOBS(intake, outtake),
                            new SleepAction(0.5),
                            P_SamplePick.SampleTransferGClosed(intake, outtake)

                    ));
                }
                runningActions.add(new SequentialAction(
                        new InstantAction(()-> transfer = true),
                        new InstantAction(()->slowMode = false)
                ));
            }
            /*
            Total Transfer of sample to outTake
            Takes Sample drop High Position
            */
            if (gamepad1.x && transfer && (robot.horizontalExtension.getCurrentPosition() <= 2)) {
                runningActions.add(new SequentialAction(
//                        new InstantAction(()-> elevatorExtended = true),
//                        new SleepAction(0.5),
                        P_SamplePick.SampleOutTakeGOpen(intake, outtake),
                        new SleepAction(0.1),
                        P_SamplePick.SampleOutTakeGClosed(intake, outtake),
                        new SleepAction(0.1),
                        P_SamplePick.SampleIntakeGOpen(intake, outtake),
                        new SleepAction(0.3),
                        ScoreAndResetSequence.BeforeScoreBucketSequence(outtake)
                ));
                transfer = false;

            }
            /*
            Sample drop and return to Pre Transfer
            */
            if ((gamepad1.right_trigger > 0.4) && sample/*!R_B_toggle_state && sample*/) {
                runningActions.add(new SequentialAction(
//                        new InstantAction(()->elevatorExtended = false),
                        ScoreAndResetSequence.ScoreBucketAndReset(outtake, intake),
                        new SleepAction(0.3),
                        P_SamplePick.OutTakeaBeforeTrans(intake, outtake)
                ));
            }
            /*
            X extension EXTREME pose
            */
            if (gamepad1.dpad_up && isIntake) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.XExtState(intake, Intake.iXextensionStates.LAST)
                ));
            }
            /*
            X extension MID pose
            */
            if (gamepad1.dpad_down && isIntake) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.XExtState(intake, Intake.iXextensionStates.MID)
                ));
            }
            /*
            wrist position 0 or 90 degree
            */
            if (gamepad2.dpad_up && isIntake) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.wristPose(intake, Intake.iWristStates.POSE_0)
                ));
            }
            if (gamepad2.dpad_down && isIntake) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.wristPose(intake, Intake.iWristStates.POSE_90)
                ));
            }



            // TODO SPECIMEN
            /*
            Pick specimen
            based on distance sensor, if distance less than threshold --> pick specimen and ready to hook
            */
//            if(robot.colorSensorOut.getDistance(DistanceUnit.MM) <= Globals.ColourDistanceOutake && specimen
//            && (Outtake.ElbowStateOut.SPECIMEN_PICK.equals(Outtake.ElbowStateOut.SPECIMEN_PICK))){
//                runningActions.add(new SequentialAction(
//                        new InstantAction(()-> specimenSpotted = true)
////                        new SleepAction(0.3),
////                        new InstantAction(()-> specimenSpotted = false)
//                )
//                );
//            }
            if(robot.colorSensorOut.getDistance(DistanceUnit.MM) <= Globals.ColourDistanceOutake && specTestVar)
            {
                runningActions.add(new SequentialAction(
                                SpecimenOuttakeSequence.SpecimenOuttakeAfterPick(outtake),
                new InstantAction(()-> specTestVar = false)

                        )
                );
            }
//            if(specTestVar && specimenSpottedTest && (robot.colorSensorOut.getDistance(DistanceUnit.MM) <= Globals.ColourDistanceOutake && specimen) &&(Outtake.ElbowStateOut.SPECIMEN_PICK.equals(Outtake.ElbowStateOut.SPECIMEN_PICK))){
//                runningActions.add(new SequentialAction(
//                                SpecimenOuttakeSequence.SpecimenOuttakeAfterPick(outtake),
//                                new SleepAction(0.2),
//                        new InstantAction(()->specTestVar = false)
//
//                        )
//                );
//            }


            /*
            Specimen drop and returns to pick specimen again
            * */
            if (gamepad1.b && specimen) {
                runningActions.add(new SequentialAction(
                        ScoreAndResetSequence.ScoreAndResetSequenceAction(outtake, intake),
                        P_SamplePick.intakeInit(intake, outtake),
                        new InstantAction(() -> inSpeciIntakePose = true),
                        new InstantAction(() -> specTestVar = true)

                ));
            }

            // Drop sample to obs
            if (gamepad1.y && specimen) {
                runningActions.add(new SequentialAction(
                        P_SamplePick.sampleDropOBS(intake, outtake),
                        new SleepAction(0.5),
                        P_SamplePick.SampleTransferGClosed(intake, outtake)

                ));
            }



            /*
            Manual Hanging with 3 steps
            1) Open lower and higher hanger -> 2-a
            2) First low hang -> 2-y
            3) High Hang  -> 2-right trigger
            */
            if (gamepad2.a) {
                Actions.runBlocking(new SequentialAction(
                        Hang.HangerINIT(intake, outtake),
                        new SleepAction(0.5),
                        Hang.HangerL(outtake, Outtake.HangerState.OPEN),
                        Hang.HangerELEV(outtake, Outtake.SliderStateOut.PRE_HANG)
                ));
            }
            if (gamepad2.y) {
                Actions.runBlocking(new SequentialAction(
                        Hang.HangerLCLUTCH(outtake, Outtake.HangerState.FIRST_HANG,Outtake.ClutchState.HANG_ENGAGED)

                ));
            }

            if (gamepad2.right_trigger > 0.4) {
                Actions.runBlocking(new SequentialAction(
                        Hang.HangerELEV(outtake, Outtake.SliderStateOut.HANG)
                ));
            }

            // TODO Telemetry
            printTelemetry(telemetry);
            telemetry.update();


        }
    }


    // TODO List action ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

        return newActions;
    }

    // TODO Telemetry things +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    public void printTelemetry(Telemetry telemetry){
        if(specimen){telemetry.addLine("SPECIMEN MODE------------->");}
        else if(sample){telemetry.addLine("SAMPLE MODE--------------->");}
        telemetry.addLine();

        telemetry.addData("NavX --------------------->", TwoDeadWheelLocalizer.robotHeading);
        telemetry.addLine();
        telemetry.addData("left Front current: ------>", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left Back current: ------->", drive.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Front current: ----->", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Back current: ------>", drive.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
        telemetry.addData("Vertical Motor L current ->", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Motor R current ->", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Horizontal Motor current ->", robot.horizontalExtension.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
        telemetry.addData("Vertical Motor Pose ------>", robot.verticalSliderLeft.getCurrentPosition());
        telemetry.addData("Vertical Motor Pose------->", robot.verticalSliderRight.getCurrentPosition());
        telemetry.addData("Horizontal Motor Pose----->", robot.horizontalExtension.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("O Dist ",robot.colorSensorOut.getDistance(DistanceUnit.MM));
        telemetry.addData("Grip Colour Sensor :" , Globals.ColourDistanceOutake);
        telemetry.addLine();
        telemetry.addData("STATE GRIPPER: ", outtake.gripperState);
        telemetry.addData("State ELBOW: ", outtake.elbowStateOut);
        telemetry.addData("State wrist: ", outtake.wristStateOut);
        telemetry.addData("Out State wrist: ", outtake.wristStateOut);
        telemetry.addData("Out State elbow: ", outtake.elbowStateOut);
        telemetry.addData("Slider State vertical: ", outtake.sliderStateOut);
        telemetry.addLine();




    }



    // TODO Elevator Pid +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public void elevatorPID(int target1){
        elevetorCTRL.setPID(kp,ki,kd);
        double power = elevetorCTRL.calculate(robot.verticalSliderRight.getCurrentPosition(), target1);
        robot.verticalSliderLeft.setPower(power);
        robot.verticalSliderRight.setPower(power);
    }


    // TODO FieldCentric drive control +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

    // TODO DC motor run method ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public static void extendTO (int target, DcMotorEx m){
        m.setTargetPosition(target);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(1);
    }

    // TODO INIT FLAGS RESET +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public void resetFlags(){
        Globals.test = false;
//        sample_detected = false;
//        wrong_detected = false;
//        not_transfer = false;
//        specimen_done = false;
//        done_scoring = false;
    }



}
