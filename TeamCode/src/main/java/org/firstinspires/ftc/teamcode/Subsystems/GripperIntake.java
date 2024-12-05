//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//
//import org.firstinspires.ftc.teamcode.Hardware.NewIntakeGlobals;
//import org.firstinspires.ftc.teamcode.Hardware.NewIntakeRobotHardware;
//
//public class GripperIntake {
//    public static NewIntakeRobotHardware robot;
//
//
//    public GripperIntake(NewIntakeRobotHardware robot){
//        this.robot = robot;
//    }
//
//    public enum GripperState{
//        OPEN,
//        CLOSE,
//        TRANSFER,
//
//    }
//    public void update(GripperState  state){
//        switch (state){
//            case OPEN:
//            robot.gripper.setPosition(NewIntakeGlobals.gripperOpen);
//                break;
//            case CLOSE:
//                robot.gripper.setPosition(NewIntakeGlobals.gripperClose);
//                break;
//            case TRANSFER:
//                robot.gripper.setPosition(NewIntakeGlobals.gripperTransfer);
//                break;
//        }
//    }
//
//    public enum WristStatNew{
//            INTAKE,
//            PREPICK,
//            INIT,
//    }
//    public void update(WristStatNew state){
//        switch (state){
//            case INIT:
//                robot.wristServo.setPosition(NewIntakeGlobals.wristINIT);
//                break;
//            case PREPICK:
//                robot.wristServo.setPosition(NewIntakeGlobals.wristPrePick);
//                break;
//            case INTAKE:
//                robot.wristServo.setPosition(NewIntakeGlobals.wristIntake);
//                break;
//        }
//    }
//
//    public enum YawStateNew {
//        PREPICK,
//        INTAKE,
//        LEFT90,
//        RIGHT90,
//    }
//    public void update(YawStateNew state){
//        switch (state){
//            case INTAKE:
//                robot.yawServo.setPosition(NewIntakeGlobals.yawIntake);
//                break;
//            case LEFT90:
//                robot.yawServo.setPosition(NewIntakeGlobals.yawLeft90);
//                break;
//            case PREPICK:
//                robot.yawServo.setPosition(NewIntakeGlobals.yawPrePick);
//                break;
//            case RIGHT90:
//                robot.yawServo.setPosition(NewIntakeGlobals.yawRight90);
//                break;
//        }
//    }
//    public Action yawAction(YawStateNew state){
//        return new InstantAction(()->update(state));
//    }
//    public Action wristAction(WristStatNew state){
//        return new InstantAction(()->update(state));
//    }
//    public Action gripperAction(GripperState state){
//        return new InstantAction(()->update(state));
//    }
//
//
//
//}
