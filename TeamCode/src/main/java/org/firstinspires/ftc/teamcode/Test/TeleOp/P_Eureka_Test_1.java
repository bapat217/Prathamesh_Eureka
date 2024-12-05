//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import static org.firstinspires.ftc.teamcode.Sequences.P_SamplePick.PickToPreTransfer;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
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
//public class P_Eureka_Test_1 extends LinearOpMode {
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
//
//    public static boolean done_scoring = false;
//
//    private SlewRateLimiter fw;
//    private SlewRateLimiter str;
//
//
//    public static double fw_r = 4;
//    public static double str_r = 4;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, telemetry);
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
//        intake = new Intake(robot);
//        outtake = new Outtake(robot);
//
//        elevetorCTRL = new PIDController(Globals.kp,Globals.ki,Globals.kd);
////
//        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.clutchServo.setPosition(0.3789);
//        isBreak = false;
//        sample_detected = false;
//        wrong_detected = false;
//        not_transfer = false;
//        specimen_done = false;
//        Globals.test = false;
//        while(opModeInInit()){
//            robot.verticalEncoder.reset();
//            Actions.runBlocking(InitSequence.InitSequenceAction(intake, outtake));
////            telemetry.addData("ELbow itnake L:", robot.elbowLeft.getPosition());
////            telemetry.addData("ELbow itnake R:", robot.elbowRight.getPosition());
////            telemetry.addData("wrist intake:", robot.wristServo.getPosition());
////            telemetry.addData("yaw intake:", robot.yawServo.getPosition());
//            telemetry.update();
//        }
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            runningActions = updateAction();
//            // Decide pid for elevator or hanger
////            if(Globals.isElev){
////                outtake.elevatorPID();
////            }
////            if(Globals.isHang){
////                outtake.hangPID();
////            }
////            outtake.elevatorPID();
//
//
//
//            if(!specimen_done) {
//                Globals.ColourDistanceReadings = RobotHardware.getDistance(robot);
//            }
////            botHeading = imu.getRobotYawPitchRollAngles();
////
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y*STRAFE,
//                            -gamepad1.left_stick_x*THROTTLE
//                    ),
//                    -gamepad1.right_stick_x*HEADING
//            ));
//            drive.updatePoseEstimate();
//
//
//
////            elevatorPID(tarGet);
//
//
//            if(gamepad1.right_bumper){
//                runningActions.add(P_SamplePick.PrePickSequence(intake,outtake));
//            }
//<<<<<<< HEAD
//=======
//
//            if(gamepad1.left_bumper){
//                runningActions.add(InitSequence.InitSequenceAction(intake,outtake));
//
//            }
//>>>>>>> db09bd4be1a9aa6d9e1ec5fe22743c81c78f15a3
//            if(gamepad1.dpad_left){
//                runningActions.add(P_SamplePick.PrePickSequenceWristPose(intake,outtake,Intake.iWristStates.POSE_45));
//                wristState = 45;
//            }
//<<<<<<< HEAD
//
//=======
//>>>>>>> db09bd4be1a9aa6d9e1ec5fe22743c81c78f15a3
//            if(gamepad1.dpad_right){
//                runningActions.add(P_SamplePick.PrePickSequenceWristPose(intake,outtake,Intake.iWristStates.POSE_135));
//                wristState = 135;
//            }
//<<<<<<< HEAD
//            if(gamepad1.right_bumper){
//                runningActions.add(new SequentialAction(
////                        ScoreAndResetSequence.ScoreBucketAndReset(intake,outtake),
//=======
//            if(gamepad1.a){
//                runningActions.add(new SequentialAction(
////                        ScoreAndResetSequence.ScoreBucketAndReset(intake,outtake),
//                        outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET),
//>>>>>>> db09bd4be1a9aa6d9e1ec5fe22743c81c78f15a3
//                        P_SamplePick.PickSequence(intake,outtake),
//                        new SleepAction(0.1),
//                        PickToPreTransfer(intake,outtake)
//                ));
//            }
//            if(gamepad1.b){
//                runningActions.add(new SequentialAction(
//                        TransferSequence.TransferOuttakeAction(outtake,intake),
//                        new SleepAction(0.3),
//                        P_SamplePick.TransferGripperClosed(intake,outtake),
//                        new SleepAction(0.1),
//                        TransferSequence.TransferOuttakeActionGclose(outtake,intake),
//                        new SleepAction(0.1),
//                        P_SamplePick.TransferGripperOppen(intake,outtake),
//                        P_SamplePick.PrePickSequenceWithoutEx(intake,outtake),
//                        outtake.elbowOutAction(Outtake.ElbowStateOut.BEFORE_BUCKET)
//
//                ));
//            }
//            if(gamepad1.dpad_down){
//                runningActions.add(ScoreAndResetSequence.ScoreBucketAndReset(intake,outtake));
//            }
//
//
////            if(gamepad1.a){
////                runningActions.add(new SequentialAction(
////                        TransferSequence.TransferOuttakeActionGclose(outtake,intake),
////                        new SleepAction(0.5),
////                        P_SamplePick.TransferGripperOppen(intake,outtake)
////                ));
////            }
//            if(gamepad1.dpad_up) {
//                runningActions.add(new
//                        SequentialAction(
//                        ScoreAndResetSequence.BeforeScoreBucketSequence(outtake),
////                        new SleepAction(0.2),
//                        P_SamplePick.intakeInit(intake, outtake)
//                ));
//            }
//
//
//            if(gamepad2.dpad_down){
//                xExt(0);
//                THROTTLE =1 ;
//                HEADING = 0.6;
//                STRAFE = 1;
//            }
//
//            if(gamepad2.dpad_right){
//                xExt(700);
//                THROTTLE = 0.3;
//                HEADING = 0.3;
//                STRAFE = 0.3;
//            }
//            if(gamepad2.dpad_left){
//                xExt(150);
//                THROTTLE = 0.3;
//                HEADING = 0.3;
//                STRAFE = 0.3;
//            }
//            if(gamepad2.dpad_up){
//                xExt(300);
//                THROTTLE = 0.3;
//                HEADING = 0.3;
//                STRAFE = 0.3;
//            }
////
////
//
//            if(gamepad2.x){
//                runningActions.add(intake.iWrist(Intake.iWristStates.POSE_180));
////                robot.iWrist.setPosition(Globals.iWrist_180);
//            }
//            if(gamepad2.b){
//                runningActions.add(intake.iWrist(Intake.iWristStates.POSE_0));
//            }
//            if(gamepad1.left_trigger>0){
//                THROTTLE = 0.3;
//                HEADING = 0.3;
//                STRAFE = 0.3;
//            }
//            else{
//                THROTTLE =1 ;
//                HEADING = 0.6;
//                STRAFE = 1;
//            }
//
//            printTelemetry(telemetry);
//            telemetry.update();
//
//        }
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
////        telemetry.addData("Intake State ELBOW: ", intake.elbowState);
//        telemetry.addData("Out State wrist: ", outtake.wristStateOut);
//        telemetry.addData("Out State elbow: ", outtake.elbowStateOut);
////        telemetry.addData("In State flapper: ", intake.flapperState);
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
//
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
//
//}
//
//
//
//
////package org.firstinspires.ftc.teamcode.Test.TeleOp;
////
////import static org.firstinspires.ftc.teamcode.Sequences.P_SamplePick.PickToPreTransfer;
////import static org.firstinspires.ftc.teamcode.Sequences.TransferSequence.AfterTransferSequenceAction;
////
////import com.acmerobotics.dashboard.config.Config;
////import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
////import com.acmerobotics.roadrunner.Action;
////import com.acmerobotics.roadrunner.Pose2d;
////import com.acmerobotics.roadrunner.PoseVelocity2d;
////import com.acmerobotics.roadrunner.SequentialAction;
////import com.acmerobotics.roadrunner.SleepAction;
////import com.acmerobotics.roadrunner.Vector2d;
////import com.acmerobotics.roadrunner.ftc.Actions;
////import com.arcrobotics.ftclib.controller.PIDController;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.IMU;
////
////import org.firstinspires.ftc.robotcore.external.Telemetry;
////import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
////import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
////import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
////import org.firstinspires.ftc.teamcode.Hardware.Globals;
////import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
////import org.firstinspires.ftc.teamcode.Hardware.SlewRateLimiter;
////import org.firstinspires.ftc.teamcode.MecanumDrive;
////import org.firstinspires.ftc.teamcode.Sequences.InitSequence;
////import org.firstinspires.ftc.teamcode.Sequences.P_SamplePick;
////import org.firstinspires.ftc.teamcode.Sequences.ScoreAndResetSequence;
////import org.firstinspires.ftc.teamcode.Sequences.TransferSequence;
////import org.firstinspires.ftc.teamcode.Subsystems.Intake;
////import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
////
////import java.util.ArrayList;
////import java.util.List;
////
////@TeleOp
////@Config
////
////public class P_Eureka_Test_1 extends LinearOpMode {
////    public static double kp = 0.025/*0.0018*/;
////    public static double   ki = 0/*0.00001*/;
////    public static double kd = 0.0001/*0.00005*/;
////
////    public PIDController elevetorCTRL = null;
////    public static int tarGet = 0;
////    public static RobotHardware robot = RobotHardware.getInstance();
////    public static List<Action> runningActions = new ArrayList<>();
////    Intake intake = null;
////    Outtake outtake = null;
////
////    public static int sliderLeft = 0;
////    public static int sliderRight = 0;
////    public static double Open = 0;
////    public static double Close = 0;
////    public static double ColourDistanceOutake = 9;
////    MecanumDrive drive = null;
////    private float y, x, rx;
////    double STRAFE =1, THROTTLE = 1, HEADING = 0.8;
////    YawPitchRollAngles botHeading;
////    private IMU imu;
////    public static boolean isBreak = false;
////    public static boolean sample_detected = false;
////    public static boolean wrong_detected = false;
////    public static boolean not_transfer = false;
////    public static boolean specimen_done = false;
////
////    public static boolean done_scoring = false;
////
////    private SlewRateLimiter fw;
////    private SlewRateLimiter str;
////
////
////    public static double fw_r = 4;
////    public static double str_r = 4;
////
////    @Override
////    public void runOpMode() throws InterruptedException {
////        robot.init(hardwareMap, telemetry);
////        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
////        intake = new Intake(robot);
////        outtake = new Outtake(robot);
////
////        elevetorCTRL = new PIDController(Globals.kp,Globals.ki,Globals.kd);
//////
////        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.clutchServo.setPosition(0.3789);
////        isBreak = false;
////        sample_detected = false;
////        wrong_detected = false;
////        not_transfer = false;
////        specimen_done = false;
////        Globals.test = false;
////        while(opModeInInit()){
////            robot.verticalEncoder.reset();
////            Actions.runBlocking(InitSequence.InitSequenceAction(intake, outtake));
////
////            telemetry.update();
////        }
////        waitForStart();
////
////        while (opModeIsActive()) {
////            runningActions = updateAction();
////
////            if(!specimen_done) {
////                Globals.ColourDistanceReadings = RobotHardware.getDistance(robot);
////            }
////
////            drive.setDrivePowers(new PoseVelocity2d(
////                    new Vector2d(
////                            -gamepad1.left_stick_y*STRAFE,
////                            -gamepad1.left_stick_x*THROTTLE
////                    ),
////                    -gamepad1.right_stick_x*HEADING
////            ));
////            drive.updatePoseEstimate();
////
////
////
////            elevatorPID(tarGet);
////
////
////
////            if(gamepad1.left_bumper){
////                runningActions.add(P_SamplePick.PrePickSequence(intake,outtake));
////            }
////            if(gamepad1.right_bumper){
////                runningActions.add(new SequentialAction(
////                        ScoreAndResetSequence.ScoreBucketAndReset(intake,outtake),
////                        P_SamplePick.PickSequence(intake,outtake),
////                        new SleepAction(0.1),
////                        PickToPreTransfer(intake,outtake)
////                ));
////            }
////            if(gamepad1.left_trigger > 0.4){
////                runningActions.add(new SequentialAction(
////                        TransferSequence.TransferOuttakeAction(outtake,intake),
////                        new SleepAction(0.7),
////                        P_SamplePick.TransferGripperClosed(intake,outtake),
////                        new SleepAction(0.1),
////                        TransferSequence.TransferOuttakeActionGclose(outtake,intake),
////                        new SleepAction(0.1),
////                        P_SamplePick.TransferGripperOppen(intake,outtake)
////
////                ));
////            }
////            if(gamepad1.right_trigger>0.4){
////                runningActions.add(ScoreAndResetSequence.ScoreBucketAndReset(intake,outtake));
////            }
////
////
////            if(gamepad1.a){
////                runningActions.add(new SequentialAction(
////                        TransferSequence.TransferOuttakeActionGclose(outtake,intake),
////                        new SleepAction(0.5),
////                        P_SamplePick.TransferGripperOppen(intake,outtake)
////                ));
////            }
////            if(gamepad1.y) {
////                runningActions.add(new
////                        SequentialAction(
////                        ScoreAndResetSequence.BeforeScoreBucketSequence(outtake),
////                        P_SamplePick.intakeInit(intake, outtake)
////                ));
////            }
////
//////            if(gamepad1.dpad_down){
//////                xExt(300);
//////            }
//////
//////            if(gamepad1.dpad_up){
//////                xExt(700);
//////            }
//////
//////
//////
//////            if(gamepad1.left_trigger>0){
//////                THROTTLE = 0.3;
//////                HEADING = 0.3;
//////                STRAFE = 0.3;
//////            }
//////            else{
//////                THROTTLE =1 ;
//////                HEADING = 0.8;
//////                STRAFE = 1;
//////            }
////
////            printTelemetry(telemetry);
////            telemetry.update();
////
////        }
////
////    }
////
////    public static List<Action> updateAction(){
////        TelemetryPacket packet = new TelemetryPacket();
////        List<Action> newActions = new ArrayList<>();
////        List<Action> RemovableActions = new ArrayList<>();
////
////        for (Action action : runningActions) {
//////            action.preview(packet.fieldOverlay());
////            if (action.run(packet)) {
////                newActions.add(action);
////            }
////        }
//////        runningActions.removeAll(RemovableActions);
////        return newActions;
////    }
////
////    public void printTelemetry(Telemetry telemetry){
////        telemetry.addData("Grip Colour Sensor Live:" , robot.colorSensorOut.getDistance(DistanceUnit.MM));
////        telemetry.addData("Grip Colour Sensor :" , Globals.ColourDistanceOutake);
////        telemetry.addData("is Break:", isBreak);
////        telemetry.addData("sample detected:", sample_detected);
////        telemetry.addData("wrong detected:", wrong_detected);
////        telemetry.addData("Not transfer:", not_transfer);
////        telemetry.addData("specimen done:", specimen_done);
////        telemetry.addData("Scoring done:", done_scoring);
////        telemetry.addLine();
//////        telemetry.addData("State: ", robot.beamBreaker.getState());
////        telemetry.addData("Distance Out: ", robot.colorSensorOut.getDistance(DistanceUnit.CM));
////        telemetry.addData("Distance In: ", robot.colorSensorIn.getDistance(DistanceUnit.CM));
////        telemetry.addData("RED IN: ", robot.colorSensorIn.red());
////        telemetry.addData("BLUE IN: ", robot.colorSensorIn.blue());
////        telemetry.addData("GREEN IN: ", robot.colorSensorIn.green());
////        telemetry.addData("RED OUT: ", robot.colorSensorIn.red());
////        telemetry.addData("BLUE OUT: ", robot.colorSensorIn.blue());
////        telemetry.addData("GREEN OUT: ", robot.colorSensorIn.green());
////        telemetry.addLine();
////        telemetry.addData("wrist outtake:", robot.wristOut.getPosition());
////        telemetry.addData("elbow out L:", robot.elbowLeftOut.getPosition());
////        telemetry.addData("elbow out R:", robot.elbowRightOut.getPosition());
////        telemetry.addData("gripper:", robot.gripper.getPosition());
//////        telemetry.addData("flapper:", robot.flapper.getPosition());
////        telemetry.addLine();
////        telemetry.addData("STATE GRIPPER: ", outtake.gripperState);
////        telemetry.addData("State ELBOW: ", outtake.elbowStateOut);
////        telemetry.addData("State wrist: ", outtake.wristStateOut);
//////        telemetry.addData("Intake State ELBOW: ", intake.elbowState);
////        telemetry.addData("Out State wrist: ", outtake.wristStateOut);
////        telemetry.addData("Out State elbow: ", outtake.elbowStateOut);
//////        telemetry.addData("In State flapper: ", intake.flapperState);
////        telemetry.addData("Slider State vertical: ", outtake.sliderStateOut);
//////        telemetry.addData("Slider State horizontal: ", intake.sliderState);
////        telemetry.addLine();
////        telemetry.addData("Vertical Motor:", robot.verticalSliderRight.getCurrentPosition());
////        telemetry.addData("Vertical Motor:", robot.verticalSliderLeft.getCurrentPosition());
////        telemetry.addData("Horizontal Motor:", robot.horizontalExtension.getCurrentPosition());
////        telemetry.addLine();
////        telemetry.addData("left Front current: ", drive.leftFront.getCurrent(CurrentUnit.AMPS));
////        telemetry.addData("left Back current: ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
////        telemetry.addData("Right Front current: ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
////        telemetry.addData("Right Back current: ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
////        telemetry.addData("Vertical Motor L:", robot.verticalSliderLeft.getCurrent(CurrentUnit.AMPS));
////        telemetry.addData("Vertical Motor R:", robot.verticalSliderRight.getCurrent(CurrentUnit.AMPS));
////        telemetry.addData("Horizontal Motor :", robot.horizontalExtension.getCurrent(CurrentUnit.AMPS));
////        telemetry.addData("Vertical Motor L Power:", robot.verticalSliderLeft.getPower());
////        telemetry.addData("Vertical Motor R Power:", robot.verticalSliderRight.getPower());
////
////        telemetry.addData("enco val" , robot.verticalEncoder.getPosition());
////        telemetry.addData("tarGet ",tarGet);
////
////    }
////
////    public void elevatorPID(int target1){
////        elevetorCTRL.setPID(kp,ki,kd);
////        double power = elevetorCTRL.calculate(robot.verticalSliderRight.getCurrentPosition(), target1);
////        robot.verticalSliderLeft.setPower(power);
////        robot.verticalSliderRight.setPower(power);
////    }
////    public static void xExt (int target){
////        robot.horizontalExtension.setTargetPosition(target);
////        robot.horizontalExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.horizontalExtension.setPower( 1 );
////    }
////
////
////}
