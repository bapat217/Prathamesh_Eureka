//package org.firstinspires.ftc.teamcode.Test.TeleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Hardware.Globals;
//import org.firstinspires.ftc.teamcode.Hardware.GripperIntakeGlobals;
//import org.firstinspires.ftc.teamcode.Hardware.GripperIntakeRobotHardware;
//
//@TeleOp
//public class GripperIntakeTest extends LinearOpMode {
//    GripperIntakeRobotHardware robot = GripperIntakeRobotHardware.getInstance();
//    private double servoLeftPosition = 0.5;
//    private double servoRightPosition = 0.5;
//    private static double wristSer    = 0.5;//0.7994;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, telemetry);
//        robot.gripperServo.setPosition(GripperIntakeGlobals.gripperOpen);
//        robot.elbowServo.setPosition(GripperIntakeGlobals.elbowInit);
//        robot.shoulderServo.setPosition(GripperIntakeGlobals.shoudlerInit);
//        robot.wristServo.setPosition(GripperIntakeGlobals.wristINIT);
//        robot.yawServo.setPosition(GripperIntakeGlobals.yawPick);
//
//        robot.elbowLeftOut.setPosition(servoLeftPosition);
//        robot.elbowRightOut.setPosition(servoRightPosition);
//        robot.wristOut.setPosition(wristSer);
//        robot.gripperOut.setPosition(Globals.gripperServoOpen);
//        waitForStart();
//        while (opModeIsActive()) {
//            if(gamepad1.a){
//                robot.wristServo.setPosition(GripperIntakeGlobals.wristPick);
//                robot.shoulderServo.setPosition(GripperIntakeGlobals.shoudlerPick);
//                robot.elbowServo.setPosition(0.3);
//                sleep(500);
//                robot.elbowServo.setPosition(GripperIntakeGlobals.elbowPick);
//                robot.yawServo.setPosition(GripperIntakeGlobals.yawPick);
//            }
//
//            if(gamepad1.y){
//                robot.wristServo.setPosition(GripperIntakeGlobals.wristPostPick);
//                robot.shoulderServo.setPosition(GripperIntakeGlobals.shoudlerPostPick);
//                robot.elbowServo.setPosition(0.3);
//                sleep(500);
//                robot.elbowServo.setPosition(GripperIntakeGlobals.elbowPostPick);
//                robot.yawServo.setPosition(GripperIntakeGlobals.yawPostPick);
//            }
//
//            if(gamepad1.x){
//                robot.gripperServo.setPosition(GripperIntakeGlobals.gripperClose);
//            }
//            if(gamepad1.b){
//                robot.gripperServo.setPosition(GripperIntakeGlobals.gripperOpen);
//            }
//
//            if(gamepad1.left_bumper){
//                robot.elbowLeftOut.setPosition(GripperIntakeGlobals.elbowOutTransferL);
//                robot.elbowRightOut.setPosition(GripperIntakeGlobals.elbowOutTransferR);
//                robot.wristOut.setPosition(GripperIntakeGlobals.wristTransferOut);
//            }
//
//            if(gamepad1.right_bumper){
//                robot.wristServo.setPosition(GripperIntakeGlobals.wristTransfer);
//                robot.shoulderServo.setPosition(GripperIntakeGlobals.shoudlerTransfer);
//                robot.elbowServo.setPosition(GripperIntakeGlobals.elbowTransfer);
//            }
//
//            if(gamepad1.dpad_up){
//                robot.gripperOut.setPosition(GripperIntakeGlobals.gripperServoClose);
//                sleep(1000);
//                robot.gripperServo.setPosition(GripperIntakeGlobals.gripperShortOpen);
//                sleep(500);
//                robot.shoulderServo.setPosition(GripperIntakeGlobals.shoudlerAfterTransfer);
//            }
//            telemetry.addData("gripper :", robot.gripperServo.getPosition());
//            telemetry.addData("wrist:", robot.wristServo.getPosition());
//            telemetry.addData("elbow:", robot.elbowServo.getPosition());
//            telemetry.addData("shoulder:", robot.shoulderServo.getPosition());
//            telemetry.addData("yaw:",robot.yawServo.getPosition());
//            telemetry.update();
//        }
//    }
//
//}
