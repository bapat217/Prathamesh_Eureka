//package org.firstinspires.ftc.teamcode.Test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp
//public class TeleopAction extends LinearOpMode {
//    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();
//    RobotHardware robot = RobotHardware.getInstance();
//    DcMotorEx motorEx;
//    Servo s1;
//    public MecanumDrive drive = null;
//    Intake intake = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
////        motorEx=hardwareMap.get(DcMotorEx.class,"m1");
//        robot.init(hardwareMap);
////        s1 = hardwareMap.get(Servo.class, "s1");
//        intake = new Intake(robot);
//
//        waitForStart();
//
//        while(opModeIsActive()){
//            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));
//            drive.updatePoseEstimate();
////            TelemetryPacket packet = new TelemetryPacket();
////
////            // updated based on gamepads
////
////            // update running actions
////            List<Action> newActions = new ArrayList<>();
////            for (Action action : runningActions) {
//////            action.preview(packet.fieldOverlay());
////                if (action.run(packet)) {
////                    newActions.add(action);
////                }
////            }
//
////            runningActions = newActions;
//            runningActions = SampleSeq.updateAction();
//            runningActions = SampleSeq2.updateAction();
//
//            if (gamepad1.a) {
//                runningActions.add(new SequentialAction(
//                        new InstantAction(()->robot.flapper.setPosition(0)),
//                        new SleepAction(2),
//                         new InstantAction(()->robot.flapper.setPosition(1))
//
////                        new InstantAction(()->motorEx.setPower(0))
//
//
//                ));
//            }
//            if (gamepad1.b){
//                new SampleSeq(intake);
//            }
//            if(gamepad1.x){
//                new SampleSeq2(intake);
//            }
//
////            dash.sendTelemetryPacket(packet);
//        }
//    }
//
//
//
//
//}