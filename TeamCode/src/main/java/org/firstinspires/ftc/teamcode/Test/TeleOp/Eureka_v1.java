//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Sequences.InitSequence;
////import org.firstinspires.ftc.teamcode.Sequences.SampleIntakeSequence;
//import org.firstinspires.ftc.teamcode.Sequences.ScoreAndResetSequence;
//import org.firstinspires.ftc.teamcode.Sequences.SpecimenIntakeSequence;
//import org.firstinspires.ftc.teamcode.Sequences.SpecimenOuttakeSequence;
//import org.firstinspires.ftc.teamcode.Sequences.TransferSequence;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp
//
//public class Eureka_v1 extends LinearOpMode {
//    public RobotHardware robot = RobotHardware.getInstance();
//    public static List<Action> runningActions = new ArrayList<>();
//    Intake intake = null;
//    Outtake outtake = null;
//
//    public static int sliderLeft = 0;
//    public static int sliderRight = 0;
//    public static double Open = 0;
//    public static double Close = 0;
//    MecanumDrive drive = null;
//    private float y, x, rx;
//    double STRAFE =1, THROTTLE = 1, HEADING = 0.8;
//    YawPitchRollAngles botHeading;
//    private IMU imu;
//    public static boolean isBreak = false;
//    public static boolean sample_detected = false;
//    public static boolean wrong_detected = false;
//    public static boolean not_transfer = false;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, telemetry);
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
//        intake = new Intake(robot);
//        outtake = new Outtake(robot);
////
//        robot.verticalSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.verticalSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        isBreak = false;
//        sample_detected = false;
//        wrong_detected = false;
//        not_transfer = false;
//
//        while(opModeInInit()){
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
//            if(gamepad1.start){
//                runningActions.add(InitSequence.InitSequenceAction(intake, outtake));
//            }
//
//            if(gamepad1.left_bumper){
//                runningActions.add(SpecimenIntakeSequence.SpecimenIntakeSequenceAction(outtake));
//            }
//            if(gamepad1.right_bumper){
////                runningActions.add(new SequentialAction(
////                        SpecimenOuttakeSequence.SpecimenOuttakeAfterPick(outtake),
////                        SampleIntakeSequence.BeforePickSequence(intake)
////                        )
//                        runningActions.add(new SequentialAction(
//                        SpecimenOuttakeSequence.SpecimenOuttakeAfterPick(outtake),
////                        SampleIntakeSequence.BeforePickSequence(intake)
//                        )
//
//                );
//            }
//            if(gamepad1.b){
//                runningActions.add(ScoreAndResetSequence.ScoreAndResetSequenceAction(outtake, intake));
//            }
////            if(gamepad1.a){
////                runningActions.add(SampleIntakeSequence.IntakeSequenceAction(intake));
////            }
//            if(gamepad1.y){
//                telemetry.addLine("PERFORMING SEQUENCE....");
//                runningActions.add(
//                        new SequentialAction(
//                                TransferSequence.TransferIntakeAction(intake),
//                                new SleepAction(1),
//                                TransferSequence.TransferOuttakeAction(outtake, intake),
//                                new SleepAction(1),
//                                TransferSequence.AfterTransferSequenceAction(outtake)
//                        )
//                );
//
//
//            }
//
//            if(gamepad1.dpad_up){
//                sample_detected = false;
//                wrong_detected = false;
//                isBreak = false;
//                runningActions.add(
//                                ScoreAndResetSequence.BeforeScoreBucketSequence(outtake)
//                );
//            }
//            if(gamepad1.dpad_down){
//                runningActions.add(
//                        ScoreAndResetSequence.ScoreBucketAndReset(intake, outtake)
//                );
//                not_transfer = false;
//
//            }
//            if(gamepad1.dpad_left){
//                runningActions.add(SampleIntakeSequence.BeforePickSequence(intake));
////                runningActions.add(SampleIntakeSequence.PlaceSequenceAction(intake));
//
//            }
//
////            if(gamepad1.x){
////                runningActions.add(intake.rollerAction(Intake.RollerState.REVERSE));
////            }
////            if(gamepad1.back){
////                runningActions.add(intake.rollerAction(Intake.RollerState.OFF));
////
////            }
//
//            if(gamepad1.left_trigger>0){
//                THROTTLE = 0.3;
//                HEADING = 0.3;
//                STRAFE = 0.3;
//            }
//            else{
//                THROTTLE =1 ;
//                HEADING = 0.8;
//                STRAFE = 1;
//            }
//            if(gamepad1.right_trigger>0){
//                Intake.runSliders(robot.horizontalExtension.getCurrentPosition() + 100, 1);
//            }
//
//
//
//            // ------------------  OPERATOR CONTROL  -------------------------
//
//
//            if(gamepad1.dpad_right){
//                runningActions.add(new SequentialAction(TransferSequence.TransferIntakeAction(intake)));
//
//            }
//
//
//
//
//
//
//            if((robot.colorSensorIn.red() > robot.colorSensorIn.green() && robot.colorSensorIn.red() > robot.colorSensorIn.blue() && robot.colorSensorIn.red() > 200) || (robot.colorSensorIn.green() > robot.colorSensorIn.red() && robot.colorSensorIn.green() > robot.colorSensorIn.blue() && robot.colorSensorIn.green()> 200)){
////                runningActions.add(intake.flapperAction(Intake.FlapperState.TAKE));
//                sample_detected = true;
//                not_transfer = false;
//                telemetry.addLine("DESIRED SAMPLE");
//            }
//            else if(robot.colorSensorIn.blue() > robot.colorSensorIn.green() && robot.colorSensorIn.blue() > robot.colorSensorIn.red() && robot.colorSensorIn.blue() > 200 ){
////                runningActions.add(intake.flapperAction(Intake.FlapperState.DISCARD));
//                wrong_detected = true;
//                not_transfer = true;
//                telemetry.addLine("WRONG SAMPLE");
//            }
//            else{
//                telemetry.addLine("NO SAMPLE");
//                sample_detected = false;
//                wrong_detected = false;
////                runningActions.add(new InstantAction(()->intake.flapperAction(Intake.FlapperState.INIT)));
//            }
//
////            if(!robot.beamBreaker.getState() && !not_transfer && !sample_detected){
////                telemetry.addLine("AFTER INTAKE....");
////                runningActions.add(new SequentialAction(
////                        SampleIntakeSequence.AfterIntakeSequenceAction(intake)
////                ));
////                not_transfer = true;
////            }
//
//
//
//
////            if(!robot.beamBreaker.getState() && sample_detected){
////                telemetry.addLine("SUCCESSFUL INTAKE");
////
////            }
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
//        telemetry.addData("is Break:", isBreak);
//        telemetry.addData("sample detected:", sample_detected);
//        telemetry.addData("wrong detected:", wrong_detected);
//        telemetry.addData("Not transfer:", not_transfer);
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
////        telemetry.addData("ELbow itnake L:", robot.elbowLeft.getPosition());
////        telemetry.addData("ELbow itnake R:", robot.elbowRight.getPosition());
////        telemetry.addData("wrist intake:", robot.wristServo.getPosition());
////        telemetry.addData("yaw intake:", robot.yawServo.getPosition());
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
//    }
//
//
//}