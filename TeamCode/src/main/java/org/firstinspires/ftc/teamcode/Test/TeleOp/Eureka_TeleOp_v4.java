//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import static org.firstinspires.ftc.teamcode.Sequences.P_SamplePick.PickToPreTransfer;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.Hardware.Globals;
//import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.Hardware.SlewRateLimiter;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Sequences.InitSequence;
//import org.firstinspires.ftc.teamcode.Sequences.P_SamplePick;
//import org.firstinspires.ftc.teamcode.Sequences.ScoreAndResetSequence;
//import org.firstinspires.ftc.teamcode.Sequences.SpecimenOuttakeSequence;
//import org.firstinspires.ftc.teamcode.Sequences.TransferSequence;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp
//@Config
//
//public class Eureka_TeleOp_v4 extends LinearOpMode {
//    public static double kp = 0.025/*0.0018*/;
//    public static double   ki = 0/*0.00001*/;
//    public static double kd = 0.0001/*0.00005*/;
//    public static int wristState = 90;
//
//    public PIDController elevetorCTRL = null;
//    public static int tarGet = 0;
//    public static RobotHardware robot = RobotHardware.getInstance();
//    public static List<Action> runningActions = new ArrayList<>();
//    Intake intake = null;
//    Outtake outtake = null;
//
//    public static int sliderLeft = 0;
//    public static int sliderRight = 0;
//    public static double Open = 0;
//    public static double Close = 0;
//    public static double ColourDistanceOutake = 9;
//    MecanumDrive drive = null;
//    private float y, x, rx;
//    double STRAFE =1, THROTTLE = 1, HEADING = 0.6;
//    YawPitchRollAngles botHeading;
//    private IMU imu;
//    public static boolean isBreak = false;
//    public static boolean sample_detected = false;
//    public static boolean wrong_detected = false;
//    public static boolean not_transfer = false;
//    public static boolean specimen_done = false;
//    public static boolean specimen_mode = false;
//    public static SlewRateLimiter fw;
//    public static SlewRateLimiter str;
//    public static boolean done_scoring = false;
//
//
//
//
//    public static double fw_r = 4;
//    public static double str_r = 4;
//
//    public static double Yscalar = 0.001;
//    public static double Xscalar = 0.001;
//
//    public static boolean toggleExtension = false;
//    public static boolean start_extension = false;
//    public static boolean toggleOrientation = false;
//    private boolean toggleScore = false;
//    private boolean start_scoring = false;
//    private boolean togglePicking = false;
//    private boolean start_picking = false;
//    private boolean transfer = false;
//    public static double distanceThreshold = 10;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Gamepad currentGamepad1 = new Gamepad();
//        Gamepad currentGamepad2 = new Gamepad();
//
//        Gamepad previousGamepad1 = new Gamepad();
//        Gamepad previousGamepad2 = new Gamepad();
//
//        robot.init(hardwareMap, telemetry);
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
//        intake = new Intake(robot);
//        outtake = new Outtake(robot);
//
//        fw = new SlewRateLimiter(fw_r);
//        str = new SlewRateLimiter(str_r);
//        elevetorCTRL = new PIDController(Globals.kp,Globals.ki,Globals.kd);
//
//        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.clutchServo.setPosition(Globals.clutchDisengaged);
//        resetFlags();
//        Globals.test = false;
//
//        while(opModeInInit()){
//            robot.verticalEncoder.reset();
//            Actions.runBlocking(InitSequence.InitSequenceAction(intake, outtake));
//            telemetry.update();
//        }
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            runningActions = updateAction();
//
//            previousGamepad1.copy(currentGamepad1);
//            previousGamepad2.copy(currentGamepad2);
//            currentGamepad1.copy(gamepad1);
//            currentGamepad2.copy(gamepad2);
//
//            if(gamepad1.back){
//                resetFlags();
//                runningActions.add(InitSequence.InitSequenceAction(intake,outtake));
//            }
//            if(!specimen_done) {
//                Globals.ColourDistanceReadings = RobotHardware.getDistance(robot);
//            }
////
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y*STRAFE,
//                            -gamepad1.left_stick_x*THROTTLE
//                    ),
//                    -gamepad1.right_stick_x*HEADING
//            ));
//
//            drive.updatePoseEstimate();
//
//            //TODO SWITCH EXTENSION ----------------------------------------------------------------
//
//            if(previousGamepad1.left_trigger > 0 && !(currentGamepad1.left_trigger>0)){
//                toggleExtension = !toggleExtension;
//                start_extension = true;
//            }
//
//            if(toggleExtension && start_extension){
//                xExt(Globals.horizontalMax);
//            }
//            else if (!toggleExtension && start_extension){
//                xExt(Globals.horizontalMedium);
//            }
//
//            //TODO SWTICH ORIENTATION OF GRIPPER ------------------------
//
//            if(previousGamepad1.right_trigger > 0 && !(currentGamepad1.right_trigger>0)){
//                toggleOrientation = !toggleOrientation;
//            }
//
//            if(toggleOrientation){
//                runningActions.add(intake.iWrist(Intake.iWristStates.POSE_0));
//            }
//            {
//                runningActions.add(intake.iWrist(Intake.iWristStates.POSE_180));
//
//            }
//
//            //TODO TOGGLE BEFORE SCORE AND SCORING FOR BASKET
//
//            if((previousGamepad1.right_bumper && !currentGamepad1.right_bumper)){
//                toggleScore = !toggleScore;
//                start_scoring = true;
//            }
//            if((toggleScore && start_scoring) && !specimen_done){
//                runningActions.add(ScoreAndResetSequence.BeforeScoreBucketSequence(outtake));
//            }
//            else if((!toggleScore && start_scoring) && !specimen_done){
//                runningActions.add(ScoreAndResetSequence.ScoreBucketAndReset(outtake));
//            }
//
//            //TODO TOGGLE FOR SPECIMEN SCORING
//            if((toggleScore && start_scoring) && specimen_done){
//                runningActions.add(SpecimenOuttakeSequence.SpecimenOuttakeBeforeScore(outtake));
//            }
//            else if((!toggleScore && start_scoring) && specimen_done){
//                runningActions.add(ScoreAndResetSequence.ScoreAndResetSequenceAction(outtake));
//            }
//
//            //TODO PICK and PICK_AND_TRANSFER TOGGLE
//            if((previousGamepad1.left_bumper && !(currentGamepad1.left_bumper))){
//                togglePicking = !togglePicking;
//                start_picking = true;
//            }
//
//            if(togglePicking && start_picking && !transfer && !specimen_mode){
//                runningActions.add(new SequentialAction(
//                        P_SamplePick.PrePickSequence(intake,outtake),
//                        new InstantAction(()-> Globals.checkColor = false)
//                        )
//                );
//
//            }
//            if(!togglePicking && start_picking && !transfer && !specimen_mode && !Globals.checkColor){
//                runningActions.add(
//                        new SequentialAction(
//                                P_SamplePick.PickSequence(intake,outtake),
//                                new SleepAction(0.1),
//                                PickToPreTransfer(intake),
//                                new InstantAction(()-> Globals.checkColor = true)
//                                )
//                );
//            }
//
//            if(Globals.checkColor && !transfer){
//                if((robot.colorSensorIn.getDistance(DistanceUnit.CM) < distanceThreshold)){
//                    transfer = true;
//                }
//                else{
//                    togglePicking = true;
//                }
//            }
//
//
//            if(transfer){
//                runningActions.add(
//                        new SequentialAction(
//                        PickToPreTransfer(intake, outtake),
//                        new SleepAction(0.5),
//                        TransferSequence.TransferOuttakeAction(outtake,intake),
//                        new SleepAction(0.3),
//                        P_SamplePick.TransferGripperClosed(intake,outtake),
//                        new SleepAction(0.1),
//                        TransferSequence.TransferOuttakeActionGclose(outtake,intake),
//                        new SleepAction(0.1),
//                        P_SamplePick.TransferGripperOppen(intake,outtake),
//                        P_SamplePick.PrePickSequenceWithoutEx(intake,outtake),
//                        outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
//                        new InstantAction(()-> resetFlags())
//                        ));
//            }
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//            // TODO SAMPLE SEQUENCE
//
////            if(gamepad1.right_bumper){
////                runningActions.add(P_SamplePick.PrePickSequence(intake,outtake));
////            }
////
////            if(gamepad1.left_bumper){
////                runningActions.add(InitSequence.InitSequenceAction(intake,outtake));
////
////            }
////            if(gamepad1.dpad_left){
////                runningActions.add(P_SamplePick.PrePickSequenceWristPose(intake,outtake,Intake.iWristStates.POSE_45));
////                wristState = 45;
////            }
////            if(gamepad1.dpad_right){
////                runningActions.add(P_SamplePick.PrePickSequenceWristPose(intake,outtake,Intake.iWristStates.POSE_135));
////                wristState = 135;
////            }
////            if(gamepad1.a){
////                runningActions.add(new SequentialAction(
//////                        ScoreAndResetSequence.ScoreBucketAndReset(intake,outtake),
////                        outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
////                        P_SamplePick.PickSequence(intake,outtake),
////                        new SleepAction(0.1),
////                        PickToPreTransfer(intake,outtake)
////                ));
////            }
////            if(gamepad1.b && !specimen_mode){
////                runningActions.add(new SequentialAction(
////                        TransferSequence.TransferOuttakeAction(outtake,intake),
////                        new SleepAction(0.3),
////                        P_SamplePick.TransferGripperClosed(intake,outtake),
////                        new SleepAction(0.1),
////                        TransferSequence.TransferOuttakeActionGclose(outtake,intake),
////                        new SleepAction(0.1),
////                        P_SamplePick.TransferGripperOppen(intake,outtake),
////                        P_SamplePick.PrePickSequenceWithoutEx(intake,outtake),
////                        outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET)
////
////                ));
////            }
////            if(gamepad1.dpad_down){
////                runningActions.add(ScoreAndResetSequence.ScoreBucketAndReset(intake,outtake));
////            }
////
////
////            if(gamepad1.dpad_up) {
////                runningActions.add(new
////                        SequentialAction(
////                        ScoreAndResetSequence.BeforeScoreBucketSequence(outtake),
//////                        new SleepAction(0.2),
////                        P_SamplePick.intakeInit(intake, outtake)
////                ));
////            }
////
////
////            if(gamepad2.dpad_down){
////                xExt(0);
////                THROTTLE =1 ;
////                HEADING = 0.6;
////                STRAFE = 1;
////            }
////
////            if(gamepad2.dpad_right){
////                xExt(700);
////                THROTTLE = 0.3;
////                HEADING = 0.3;
////                STRAFE = 0.3;
////            }
////            if(gamepad2.dpad_left){
////                xExt(150);
////                THROTTLE = 0.3;
////                HEADING = 0.3;
////                STRAFE = 0.3;
////            }
////            if(gamepad2.dpad_up){
////                xExt(300);
////                THROTTLE = 0.3;
////                HEADING = 0.3;
////                STRAFE = 0.3;
////            }
//////
//////
////
////            if(gamepad2.x){
////                runningActions.add(intake.iWrist(Intake.iWristStates.POSE_180));
//////                robot.iWrist.setPosition(Globals.iWrist_180);
////            }
////            if(gamepad2.b){
////                runningActions.add(intake.iWrist(Intake.iWristStates.POSE_0));
////            }
////            if(gamepad1.left_trigger>0){
////                THROTTLE = 0.3;
////                HEADING = 0.3;
////                STRAFE = 0.3;
////            }
////            else{
////                THROTTLE =1 ;
////                HEADING = 0.6;
////                STRAFE = 1;
////            }
////
////            //TODO SPECIMEN SEQUENCE ---------------
////            if(gamepad1.b && specimen_mode){
////                runningActions.add(
////                        new SequentialAction(ScoreAndResetSequence.ScoreAndResetSequenceAction(outtake, intake),
////                                new SleepAction(1),
////                                new InstantAction(()->specimen_done=false)));
////
////            }
////
////            if((robot.colorSensorOut.getDistance(DistanceUnit.MM) <= Globals.ColourDistanceOutake) && !specimen_done){
//////                if(gamepad1.a){
////                telemetry.addLine("PICKING SEEEQUENCE");
////                runningActions.add(new SequentialAction(
////                                SpecimenOuttakeSequence.SpecimenOuttakeAfterPick(outtake)
////                        )
////                );
////                specimen_done = true;
////            }
////
////            if(gamepad2.start){
////                specimen_mode = true;
////            }
////            if(gamepad2.back){
////                specimen_mode = false;
////            }
//
//            printTelemetry(telemetry);
//            telemetry.update();
//
//
//
//        }
//
//
//
//    }
//
//    private void resetFlags() {
//        isBreak = false;
//        sample_detected = false;
//        wrong_detected = false;
//        not_transfer = false;
//        specimen_done = false;
//        specimen_mode = false;
//        toggleExtension = false;
//        start_extension = false;
//        toggleOrientation = false;
//        toggleScore = false;
//        start_scoring = false;
//        togglePicking = false;
//        start_picking = false;
//        transfer = false;
//        Globals.checkColor = false;
//
//    }
//
//    public static List<Action> updateAction(){
//        TelemetryPacket packet = new TelemetryPacket();
//        List<Action> newActions = new ArrayList<>();
//        List<Action> RemovableActions = new ArrayList<>();
//
//        for (Action action : runningActions) {
////            action.preview(packet.fieldOverlay());
//            if (action.run(packet)) {
//                newActions.add(action);
//            }
//        }
////        runningActions.removeAll(RemovableActions);
//        return newActions;
//    }
//
//    public void printTelemetry(Telemetry telemetry){
//        telemetry.addData("Grip Colour Sensor Live:" , robot.colorSensorOut.getDistance(DistanceUnit.MM));
//        telemetry.addData("Grip Colour Sensor :" , Globals.ColourDistanceOutake);
//        telemetry.addData("is Break:", isBreak);
//        telemetry.addData("sample detected:", sample_detected);
//        telemetry.addData("wrong detected:", wrong_detected);
//        telemetry.addData("Not transfer:", not_transfer);
//        telemetry.addData("specimen done:", specimen_done);
//        telemetry.addData("Scoring done:", done_scoring);
//        telemetry.addData("Specimen mode:", specimen_mode);
//        telemetry.addData("Check color:", Globals.checkColor);
//        telemetry.addData("Toggle picking:", togglePicking);
//        telemetry.addData("Transfer:", transfer);
//        telemetry.addData("Start Picking:", start_picking);
//        telemetry.addLine();
////        telemetry.addData("State: ", robot.beamBreaker.getState());
//        telemetry.addData("Distance Out: ", robot.colorSensorOut.getDistance(DistanceUnit.CM));
//        telemetry.addData("Distance In: ", robot.colorSensorIn.getDistance(DistanceUnit.CM));
//        telemetry.addData("RED IN: ", robot.colorSensorIn.red());
//        telemetry.addData("BLUE IN: ", robot.colorSensorIn.blue());
//        telemetry.addData("GREEN IN: ", robot.colorSensorIn.green());
//        telemetry.addData("RED OUT: ", robot.colorSensorIn.red());
//        telemetry.addData("BLUE OUT: ", robot.colorSensorIn.blue());
//        telemetry.addData("GREEN OUT: ", robot.colorSensorIn.green());
//        telemetry.addLine();
//        telemetry.addData("wrist outtake:", robot.wristOut.getPosition());
//        telemetry.addData("elbow out L:", robot.elbowLeftOut.getPosition());
//        telemetry.addData("elbow out R:", robot.elbowRightOut.getPosition());
//        telemetry.addData("gripper:", robot.gripper.getPosition());
////        telemetry.addData("flapper:", robot.flapper.getPosition());
//        telemetry.addLine();
//        telemetry.addData("STATE GRIPPER: ", outtake.gripperState);
//        telemetry.addData("State ELBOW: ", outtake.elbowStateOut);
//        telemetry.addData("State wrist: ", outtake.wristStateOut);
//        telemetry.addData("Out State wrist: ", outtake.wristStateOut);
//        telemetry.addData("Out State elbow: ", outtake.elbowStateOut);
//        telemetry.addData("Slider State vertical: ", outtake.sliderStateOut);
//        telemetry.addData("Intake State ELBOW: ", intake.ielbowState);
//        telemetry.addData("Intake State wrist: ", intake.iwristState);
//        telemetry.addData("Intake shoulder: ", intake.ishoulderState);
//        telemetry.addData("Intake Gripper : ", intake.igripperState);
//        telemetry.addData("Slider State vertical: ", outtake.sliderStateOut);
////        telemetry.addData("Slider State horizontal: ", intake.sliderState);
//        telemetry.addLine();
//        telemetry.addData("Vertical Motor:", robot.verticalSliderRight.getCurrentPosition());
//        telemetry.addData("Vertical Motor:", robot.verticalSliderLeft.getCurrentPosition());
//        telemetry.addData("Horizontal Motor:", robot.horizontalExtension.getCurrentPosition());
//        telemetry.addLine();
//        telemetry.addData("left Front current: ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("left Back current: ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Right Front current: ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Right Back current: ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Vertical Motor L:", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Vertical Motor R:", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Horizontal Motor :", robot.horizontalExtension.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Vertical Motor L Power:", robot.verticalSliderLeft.getPower());
//        telemetry.addData("Vertical Motor R Power:", robot.verticalSliderRight.getPower());
//        telemetry.addData("enco val" , robot.verticalEncoder.getPosition());
//        telemetry.addData("tarGet ",tarGet);
//
//    }
//
//    public void elevatorPID(int target1){
//        elevetorCTRL.setPID(kp,ki,kd);
//        double power = elevetorCTRL.calculate(robot.verticalSliderRight.getCurrentPosition(), target1);
//        robot.verticalSliderLeft.setPower(power);
//        robot.verticalSliderRight.setPower(power);
//    }
//    public static void xExt (int target){
//        robot.horizontalExtension.setTargetPosition(target);
//        robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.horizontalExtension.setPower(1);
//    }
//
//    private double joystickScalar(double num, double min) {
//        return joystickScalar(num, min, 0.66, 4);
//    }
//
//    private double joystickScalar(double n, double m, double l, double a) {
//        return Math.signum(n) * m
//                + (1 - m) *
//                (Math.abs(n) > l ?
//                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
//                        n / a);
//    }
//
//
//}
//
//
//
//
