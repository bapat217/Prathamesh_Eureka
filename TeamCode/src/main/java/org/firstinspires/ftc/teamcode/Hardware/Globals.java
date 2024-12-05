package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

    public static boolean test = false;
    //TODO ----------- INTAKE -------------

    // TODO Intake Gripper
    public static double iGripper_Init = 0.5;
    public static double iGripper_Close = 0.2089;
    public static double iGripper_Open = 0.6694;
    //    public static double iGripper_Init = 0.45;
    // TODO Intake Wrist
    public static double iWrist_0 = 0.8394;//
    public static double iWrist_45 = 0.3706;
    public static double iWrist_90 = 0.5383;// transfer
    public static double iWrist_135 = 0.7;
    public static double iWrist_180 = 0.2;

    // TODO Intake Elbow
    public static double iElbow_Init = 0.385;
    public static double iElbow_BeforePick = 0.0411;//0.0839;//0.20;
    public static double iElbow_Pick = 00544;
    public static double iElbow_AfterPick =0.2;//0.5883;// 0.20;
    public static double iElbow_Transfer = 0.49;
    public static double iElbow_Post_Transfer = 0.3728;
    public static double iElbow_Obs_Drop = 0.385;//0.2622;
//    public static double iElbowChangePos = 0;


    // TODO Intake Shoulder
    public static double iShoulder_Init = 0.45;// 0.3467;
    public static double iShoulder_BeforePick = 0.137;//0.1361;//0.1167;//0.1639;
    public static double iShoulder_Pick = 0.1083;//0.1639;
    public static double iShoulder_AfterPick = 0.1228;//0.1139;//0.152;
    public static double iShoulder_Transfer = 0.2794;
    public static double iShoulder_Obs_Drop = 0.1994;


    // TODO Intake Wiper
    public static double iWiper_Init = 0.5;
    public static double iWiper_BeforePick = 0.5;
    public static double iWiper_Pick = 0.5;
    public static double iWiper_AfterPick = 0.5;
    public static double iWiper_Transfer = 0.5;
    public static double iWiper_Left = 0.5;
    public static double iWiper_Right = 0.5;
    public static double iWiper_Obs_Drop = 0.5561;



    // TODO Intake X extension
    public static int iXext_Init = 0;
    public static int iXext_Close = 0;
    public static int iXext_Mid = 300;
    public static int iXext_Last = 700;
    public static int iXext_AutoMidYellow = 500;






    // TODO PID For elevator - clutch;
    public static double kp = 0.025/*0.0018*/, ki = 0/*0.00001*/, kd = 0.0001/*0.00005*/;
    public static int target = 0;







    //TODO OUTTAKE
    // TODO GRIPPER OUTTAKE
    public static double oGripperOpen = 0.3733;
    public static double oGripperClosed = 0.61;//0.3594;// 0.3444;

    // TODO - TWIST
    public static double TWIST_INIT = 0.2194;
    public static double TWIST_0 = 0.2194;
    public static double TWIST_180 = 0.7994;


    // TODO WRIST
    // Wrist at specimen pick
    public static double wristPickSpecimen = 0.3778;//0.6056;//0.3433;//0.3;//

    // wrist after pick specimen
    public static double wristAfterPickSpecimen = 0.1244;//0.1656;//0.2889;

    // wrist before scoring specimen
    public static double wristBeforeScoreSpecimen = 0.575;

    // Specimen scoring
    public static double wristPlaceSpecimen = 0.575;//0.8483;

    // Bucket
    public static double wristBeforePlaceBucket = 0.7439;
    public static double wristPlaceBucket = 1;

    public static double wristInitOutTake = 0.7172;


    // Wrist before transfer
    public static double beforeTransferWristOut = 0.3344;

    // Wrist transfer
    public static double transferWristOut = 0.1783;//0.5472;//0.3344;// 0.1911;




    // TODO ELBOW
    // Elbow Init
    public static double elbowInitOutL = 0.5;
    public static double elbowInitOutR = 0.5;

    // Elbow Specimen pick
    public static double elbowPickSpecimenL = 0.7739;//0.8717;//0.8772;
    public static double elbowPickSpecimenR = 0.2556;//0.1939;//0.1811;

    // Elbow after pick
    public static double elbowAfterPickSpecimenL = 0.5161;//0.7144;  0.1694
    public static double elbowAfterPickSpecimenR = 0.5161;//0.2844;

    // Elbow before speci score
    public static double elbowBeforeScoreSpecimenL = elbowAfterPickSpecimenL;//0.9789;//0.9239; 0.1933
    public static double elbowBeforeScoreSpecimenR = elbowAfterPickSpecimenR;//0.7417;//0.8444;

    // Specimen scoring High
    public static double elbowPlaceSpecimenR = 0.65;
    public static double elbowPlaceSpecimenL = 0.3794;

    // Elbow before transfer
    public static double beforeTransferelbowRightOut = 0.5456;// 0.6028;//0.6322;//0.6422;
    public static double beforeTransferelbowLeftOut = 0.5306;//0.4689;//0.45;//0.4389 ;

    // Transfer
    public static double transferelbowRightOut = 0.5783;// 0.6028;//0.6322;//0.6422;
    public static double transferelbowLeftOut = 0.5033;//0.4689;//0.45;//0.4389 ;



    // Bucket
    // Before Bucket
    public static double elbowBeforePlaceBucketL = 0.625;
    public static double elbowBeforePlaceBucketR = 0.4361;//0.8883;

    // Score Bucket
    public static double elbowPlaceBucketR =  0.3944;//0.4256;
    public static double elbowPlaceBucketL = 0.645;//0.6578;


    // High chamber slider value for picking specimen
    public static int SHighChamberValue = -200;
    // High chamber slider value
    public static int HighChamberValue = -100;

    public static int LowChamberValue = 0;
    public static int SpecimenScoreHigh = 0;

    public static int afterTransfer = -200;

    public static int BucketValue = -1200;

    public static int BucketValueThroughBore = 0;



    // TODO HANGING Values

    public static int lowerHang_INIT = 0;
    public static int lowerHang_OPEN = 9393;
    public static int lowerHang_First = 7640;
    public static int lowerHang_Second = 7640;

    public static int ELEV_PRE_HANG = -1250;
    public static int ELEV_HANG = 3600;

    public static double HANG_ENGAGED = 0.5767;
    public static double ELEV_ENGAGED = 0.3789;
    public static double DISENGAGED = 0.3789;


    //TODO ----------- SENSOR VALUES ------------
    public static int redValue = 0;
    public static int blueValue = 0;
    public static int greenValue = 0;
    public static double ColourDistanceOutake = 30;
    public static double ColourDistanceReadings = 0;


//    public static double kp = 0.0003, ki = 0, kd = 0;

    public static double maxDeltaPower = 0.02; // Adjust for smoothness (smaller = smoother)

    // Bapat

    public static double elbowPreAfterPickSpecimenL = 0.6794;//0.6239;//0.7817;
    public static double elbowPreAfterPickSpecimenR = 0.3533;//0.4333;// 0.2778;

    public static int elevTarget = 0;

    public static double clutchDisengaged = 0.3789;

    public static boolean checkColor = false;








}











//package org.firstinspires.ftc.teamcode.Hardware;
//
//import com.acmerobotics.dashboard.config.Config;
//
//@Config
//public class Globals {
//
//    public static boolean test = false;
//    //TODO ----------- INTAKE -------------
///*
//*     public static double wristSer    = 0.7994;
//    public static double yawSer      = 0.8667;
//    public static double elbowSerL   = 0.525;
//    public static double elbowSerR   =  0.3972;
//    public static double flapSer     = 0.6;
//* */
//
//    // TODO Intake Gripper
//    public static double iGripper_Init = 0.5;
//    public static double iGripper_Close = 0.2089;
//    public static double iGripper_Open = 0.6694;
////    public static double iGripper_Init = 0.45;
//    // TODO Intake Wrist
//    public static double iWrist_0 = 0.8394;//
//    public static double iWrist_45 = 0.3706;
//    public static double iWrist_90 = 0.5383;// transfer
//    public static double iWrist_135 = 0.7;
//    public static double iWrist_180 = 0.2;
//
//    // TODO Intake Elbow
//    public static double iElbow_Init = 0.385;
//    public static double iElbow_BeforePick = 0.0411;//0.0839;//0.20;
//    public static double iElbow_Pick = 00544;
//    public static double iElbow_AfterPick =0.2;//0.5883;// 0.20;
//    public static double iElbow_Transfer = 0.49;
//    public static double iElbow_Post_Transfer = 0.3728;
//    public static double iElbow_Obs_Drop = 0.385;//0.2622;
//
//
//    // TODO Intake Shoulder
//    public static double iShoulder_Init = 0.45;// 0.3467;
//    public static double iShoulder_BeforePick = 0.137;//0.1361;//0.1167;//0.1639;
//    public static double iShoulder_Pick = 0.1083;//0.1639;
//    public static double iShoulder_AfterPick = 0.1228;//0.1139;//0.152;
//    public static double iShoulder_Transfer = 0.2794;
//    public static double iShoulder_Obs_Drop = 0.1994;
//
//
//    // TODO Intake Wiper
//    public static double iWiper_Init = 0.5;
//    public static double iWiper_BeforePick = 0.5;
//    public static double iWiper_Pick = 0.5;
//    public static double iWiper_AfterPick = 0.5;
//    public static double iWiper_Transfer = 0.5;
//    public static double iWiper_Left = 0.5;
//    public static double iWiper_Right = 0.5;
//    public static double iWiper_Obs_Drop = 0.5561;
//
//
//
//    // TODO Intake X extension
//    public static int iXext_Init = 0;
//    public static int iXext_Close = 0;
//    public static int iXext_Mid = 300;
//    public static int iXext_Last = 700;
//    public static int iXext_AutoMidYellow = 500;
//
//
//
//
//
//
//    // TODO PID For elevator - clutch;
//    public static double kp = 0.025/*0.0018*/, ki = 0/*0.00001*/, kd = 0.0001/*0.00005*/;
//    public static double Hkp = 0/*0.0018*/, Hki = 0/*0.00001*/, Hkd = 0/*0.00005*/;
//    public static int target = 0;
//
//    public static boolean isElev = true; // initially true
//    public static boolean isHang = false; // initially false
//    public static int hangTarget = 0;
//
//    public static double intakePowerL = 0.5;
//    public static double intakePowerR = -1;
//
//    public static double elbowInitL = 0.16;
//    public static double elbowBeforePickL = 0.2767;
//    public static double clearSubmersibleInitL = 0.6444;
//    public static double clearSubmersibleL = 0.6328;
//    public static double elbowPickSubmersibleL = 0.67;
//    public static double elbowAfterPickSubmersibleL = 0.4106;
//    public static double elbowAfterPickL = 0.7106;
//    public static double elbowPlaceObservationL = 1;
//
//    public static double elbowInitR = 0.675;
//    public static double elbowBeforePickR = 0.5561;
//    public static double clearSubmersibleInitR = 0.1067;
//    public static double clearSubmersibleR = 0.1111;
//    public static double elbowPickSubmersibleR = 0.1617;
//    public static double elbowAfterPickSubmersibleR = 0.3967;
//    public static double elbowAfterPickR = 0.4039;
//    public static double elbowPlaceObservationR = 0.8144;
//
//    public static double wristInitIntake = 0.7172;
//    public static double wristBeforePick = 0.6989; // 0.3956
//    public static double wristClearSubmersibleInit = 0.7322;
//    public static double wristClearSubmersible = 0.8078;
//    public static double wristPickSubmersibleIntake = 0.6356;
//    public static double wristAfterPickSubmersibleIntake = 0.7322;
//    public static double wristAfterPickIntake = 0.7811;
//    public static double wristPlaceObservation= 0.2672;
//
//
//
//    public static double flapperDiscardSample  = 0.2859;
//    public static double flapperHoldSample = 0.5178;
//    public static double flapperReleaseSample = 0.6833;
//    public static double flapperTakeSample  = 0.5261;
//    public static double flapperInit  = 0.5261;
//
//
//    public static double clutchServoOff = 0;
//    public static double clutchServoOn = 0;
//
//    public static double yawServoInit = 0.8767;
//    public static double yawServoPick = 0.8767;
//    public static double yawServoPlace = 0.8767;
//    public static double yawServoLeft = 0;
//    public static double yawServoRight = 1;
//
//    public static double horizontalValue = 0;
//
//    //TODO ------------- TRANSFER -------------
//    public static double transferelbowLeft = 0.2833;
//    public static double transferelbowRight = 0.5461;
//
//
//    public static double transferWrist = 0.9206;
//    public static double beforeTransferWristOut = 0.3344;
//    public static double transferWristOut = 0.1783;//0.5472;//0.3344;// 0.1911;
//    public static double beforeTransferelbowRightOut = 0.5456;// 0.6028;//0.6322;//0.6422;
//    public static double beforeTransferelbowLeftOut = 0.5306;//0.4689;//0.45;//0.4389 ;
//
//
//    //TODO OUTTAKE
//    public static double gripperServoOpen = 0.4788;
//    public static double gripperServoClose = 0.3594;// 0.3444;
//
//    public static double wristInitOutake = 0.5;
//    public static double wristPickSpecimen = 0.3433;//0.3;//
//    public static double wristAfterPickSpecimen = 0.1656;//0.2889;
//    public static double wristBeforeScoreSpecimen = 0.3333;
//    public static double wristPlaceSpecimen = 0.8511;//0.8483;
//    public static double wristBeforePlaceBucket = 0.7439;
//    public static double wristPlaceBucket = 1;
//    public static double wristTransferOut = 0.8333;
//
//    public static double elbowInitOutL = 0.5;
//    public static double elbowPickSpecimenL = 0.8717;//0.8772;
//    public static double elbowPickSpecimenR = 0.1939;//0.1811;
//    public static double elbowAfterPickSpecimenL = 0.9239;//0.7144;  0.1694
//    public static double elbowBeforeScoreSpecimenL = 0.9789;//0.9239; 0.1933
//    public static double SAMelbowAfterPickSpecimenL = 0.855;//0.865;//0.1694;
//    public static double SAMelbowAfterPickSpecimenR =  0.8572;//0.8672;//0.1933;
//    public static double INBET_elbowAfterPickSpecimenR = 0.5028;
//    public static double INBET_elbowAfterPickSpecimenL = 0.5356;
//    public static double elbowOutTransferL =  0.5817;//0.9239;
//
//
//
//
//    public static double transferelbowRightOut = 0.5783;// 0.6028;//0.6322;//0.6422;
//    public static double transferelbowLeftOut = 0.5033;//0.4689;//0.45;//0.4389 ;
//    public static double elbowInitOutR = 0.5;
//
//
//    public static double elbowAfterPickSpecimenR = 0.8444;//0.2844;
//    public static double elbowBeforeScoreSpecimenR = 0.7417;//0.8444;
//
//    public static double elbowPlaceBucketR =  0.3944;//0.4256; // Bucket score;
//    public static double elbowPlaceBucketL = 0.645;//0.6578;
//
//    public static double elbowBeforePlaceBucketL = 0.625;  // Before bucket
//    public static double elbowBeforePlaceBucketR = 0.4361;//0.8883;
//    //0.8883;
//    public static double elbowPlaceSpecimenR = 0.8972;
//    public static double elbowPlaceSpecimenL = 0.8011;
////    public static double elbowPlaceSpecimenR = 0.9317;//0.9872;//0.9472;//0.8972; // SAM
////    public static double elbowPlaceSpecimenL = 0.7722;//0.8911;//0.8511;//0.8011; // SAM
//    public static double elbowOutTransferR = 0.82;
//
//    public static int HighChamberValue = -100;
//    public static int SHighChamberValue = -200;
//    public static int LowChamberValue = 0;
//    public static int SpecimenScoreHigh = 0;
//    public static int horizontalLow = 150;
//    public static int horizontalMedium = 150;
//    public static int horizontalMax = 700;
//    public static int afterTransfer = -200;
//
//    public static int BucketValue = -1200;
//
//    public static int BucketValueThroughBore = 0;
//
//
//    //TODO ----------- SENSOR VALUES ------------
//    public static int redValue = 0;
//    public static int blueValue = 0;
//    public static int greenValue = 0;
//    public static double ColourDistanceOutake = 30;
//    public static double ColourDistanceReadings = 0;
//
//
////    public static double kp = 0.0003, ki = 0, kd = 0;
//
//    public static double maxDeltaPower = 0.02; // Adjust for smoothness (smaller = smoother)
//
//    // Bapat
//
//    public static double elbowPreAfterPickSpecimenL = 0.6794;//0.6239;//0.7817;
//    public static double elbowPreAfterPickSpecimenR = 0.3533;//0.4333;// 0.2778;
//
//    public static int elevTarget = 0;
//
//    public static double clutchDisengaged = 0.3789;
//
//    public static boolean checkColor = false;
//    public static double iElbowChangePos = 0;
//
//
//
//    // TODO HANGING Values
//
//    public static int lowerHang_INIT = 0;
//    public static int lowerHang_OPEN = 9393;
//    public static int lowerHang_First = 7640;
//    public static int lowerHang_Second = 7640;
//
//    public static int ELEV_PRE_HANG = -1250;
//    public static int ELEV_HANG = 3600;
//
//    public static double HANG_ENGAGED = 0.5767;
//    public static double ELEV_ENGAGED = 0.3789;
//    public static double DISENGAGED = 0.3789;
//
//
//    // TODO - Wrist
//    public static double TWIST_INIT = 0.2194;
//    public static double TWIST_0 = 0;
//    public static double TWIST_180 = 0;
//
//}
