//package org.firstinspires.ftc.teamcode.Test;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp
//public class intakeTestAction extends LinearOpMode {
//    RobotHardware robot = RobotHardware.getInstance();
//    Intake intake = null;
//    public static boolean stop = false;
//    public static boolean red_reverse = false;
//    public static boolean red_detected = false;
//    public static boolean blue_detected = false;
//    public static boolean yellow_detected = false;
//    public static boolean toggleStart = false;
//    public static boolean finish_reverse = false;
//
//    private List<Action> runningActions = new ArrayList<>();
//
//    // 0.62 out
//    //0.85
//    //for red red = > 100 blue < 100
//    //for blue red < 100 blue > 100
//    //for yello green > 200
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, telemetry);
//        intake = new Intake(robot);
//        Gamepad currentGamepad = new Gamepad();
//        Gamepad prevGamepad = new Gamepad();
//
//        stop = false;
//        toggleStart = false;
//        finish_reverse = false;
////        Actions.runBlocking(intake.flapperAction(Intake.FlapperState.INIT));
//////        while(opModeInInit()) {
//////            telemetry.addData("Color Values RED ", robot.colorSensor.red());
////            telemetry.addData("Color Values BLUE", robot.colorSensor.blue());
////            telemetry.addData("Color Values GREEN", robot.colorSensor.green());
////            telemetry.update();
////        }
//        waitForStart();
//
//        while(opModeIsActive()){
//
//            TelemetryPacket packet = new TelemetryPacket();
//
//            // updated based on gamepads
//
//            // update running actions
//            List<Action> newActions = new ArrayList<>();
//            for (Action action : runningActions) {
//                if (action.run(packet)) {
//                    newActions.add(action);
//                }
//            }
//            runningActions = newActions;
//
//            prevGamepad.copy(currentGamepad);
//            currentGamepad.copy(gamepad1);
//
//            if(prevGamepad.start && !currentGamepad.start){
//                toggleStart = !toggleStart;
//            }
//
//            if(gamepad1.x){
//                runningActions.add(intake.rollerAction(Intake.RollerState.REVERSE));
//            }
//
//
//            if(!stop){
//                telemetry.addLine("Motor is runnning..");
//
//                runningActions.add(intake.rollerAction(Intake.RollerState.ON));
//            }
//            else{
//                runningActions.add(intake.rollerAction(Intake.RollerState.OFF));
//            }
//
//
//            //red
//            if((robot.colorSensorIn.red() > 200 && robot.colorSensorIn.green() < 250) ){
//                telemetry.addLine("red detected");
//                red_detected = true;
//                runningActions.add(intake.flapperAction(Intake.FlapperState.INIT));
//
//            }
//            //blue
//            else if((robot.colorSensorIn.red() < 100 && robot.colorSensorIn.blue() > 400)){
//                telemetry.addLine("blue detected");
//                blue_detected = true;
//                runningActions.add(intake.flapperAction(Intake.FlapperState.INIT));
//
//            }
//            else if((robot.colorSensorIn.green() > 800 && robot.colorSensorIn.red()> 500)){
//                telemetry.addLine("yellow detected");
//                yellow_detected = true;
//                runningActions.add(intake.flapperAction(Intake.FlapperState.TAKE));
//            }
//            else{
//                telemetry.addLine("Nothing detected");
////                finish_reverse = false;
//                red_reverse = false;
//                red_detected = false;
//                blue_detected = false;
//                yellow_detected = false;
//            }
//
//            if(!robot.beamBreaker.getState() && red_detected && !stop){
//                runningActions.add(intake.flapperAction(Intake.FlapperState.DISCARD));
//                if(red_reverse) {
//                    stop = false;
//                }
//                else{
//                    stop = true;
//                    toggleStart = false;
//                    finish_reverse = false;
//                }
//            }
//
//            if(toggleStart && stop && !finish_reverse){
//                runningActions.add(new SequentialAction(intake.rollerAction(Intake.RollerState.REVERSE), intake.flapperAction(Intake.FlapperState.DISCARD)));
//                if(robot.beamBreaker.getState() && red_detected){
//                    red_reverse = true;
//                    finish_reverse= true;
//                    telemetry.addLine("Reverse complete");
//                    telemetry.update();
//                }
//                telemetry.addLine("INSIDE REVERSE BLOCK");
//                telemetry.update();
//            }
//
//
//
//            telemetry.addData("Color Values RED ", robot.colorSensorIn.red());
//            telemetry.addData("Color Values BLUE", robot.colorSensorIn.blue());
//            telemetry.addData("Color Values GREEN", robot.colorSensorIn.green());
//            telemetry.addData("flapper values", robot.flapper.getPosition());
////            telemetry.addData("flapper values inc ", inc);
//            telemetry.addData(" red detected", red_detected);
//            telemetry.addData("red reverse", red_reverse);
//            telemetry.addData("Beam breaker ", robot.beamBreaker.getState());
//            telemetry.addData("stop ", stop);
//            telemetry.addData("toggle ", toggleStart);
//            telemetry.addData("finish ", finish_reverse);
//            telemetry.update();
//        }
//    }
//
//}
