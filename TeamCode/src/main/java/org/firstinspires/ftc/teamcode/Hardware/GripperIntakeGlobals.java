package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class GripperIntakeGlobals {

    //TODO INTAKE
    public static double gripperOpen = 0.6206;
    public static double gripperTransfer = 0;
    public static double gripperClose = 0.36;
    public static double gripperShortOpen = 0.39;

    public static  double wristPrePick = 0.6056;
    public static  double wristINIT = 0.6061;
    public static double wristPick = 0.6056;
    public static double wristPostPick = 0.6056;
    public static double wristTransfer = 0.6678;
    //0.6056

    public static  double yawPrePick = 0;
    public static  double yawLeft90 = 0;
    public static  double yawRight90 = 0;
    public static double yawPick = 0.5;
    public static double yawPostPick = 0.5;
    public static double yawTransfer = 0.5;
// 0.5

    public static double elbowPostPick = 0.5617;
    public static double elbowPick = 0;
    public static double elbowPrePick = 0.3;
    public static double elbowInit = 0.2811;
    public static double elbowTransfer= 0.3728;
    //0.3596

    public static double shoudlerPostPick = 0.1522;
    public static double shoulderPrePick = 0;
    public static double shoudlerPick = 0.1311;
    public static double shoudlerInit = 0.4328;
    public static double shoudlerTransfer = 0.2461;
    public static double shoudlerAfterTransfer = 0.22;
    //0.2356

    //TODO OUTTAKE
    public static double gripperServoOpen = 0.4788;
    public static double gripperServoClose = 0.3444;

    public static double wristInitOutake = 0.5;
    public static double wristPickSpecimen = 0.3433;//0.3;//
    public static double wristAfterPickSpecimen = 0.1656;//0.2889;
    public static double wristBeforeScoreSpecimen = 0.3333;
    public static double wristPlaceSpecimen = 0.8511;//0.8483;
    public static double wristBeforePlaceBucket = 0.7439;
    public static double wristPlaceBucket = 1;
    public static double wristTransferOut = 0.8333;

    public static double elbowInitOutL = 0.5;
    public static double elbowPickSpecimenL = 0.8772;
    public static double elbowAfterPickSpecimenL = 0.9239;//0.7144;
    public static double elbowBeforeScoreSpecimenL = 0.9789;//0.9239;
    public static double elbowOutTransferL =  0.5817;//0.9239;

    public static double elbowPlaceSpecimenL = 0.8011;
    public static double elbowBeforePlaceBucketL = 0.625;
    public static double elbowPlaceBucketL = 0.6578;

    public static double elbowInitOutR = 0.5;
    public static double elbowPickSpecimenR = 0.1811;
    public static double elbowAfterPickSpecimenR = 0.8444;//0.2844;
    public static double elbowBeforeScoreSpecimenR = 0.7417;//0.8444;
    public static double elbowBeforePlaceBucketR = 0.4361;//0.8883;
    public static double elbowPlaceBucketR = 0.4256;//0.8883;
    public static double elbowPlaceSpecimenR = 0.8972;
    public static double elbowOutTransferR = 0.82;

    public static int HighChamberValue = -100;
    public static int LowChamberValue = 0;
    public static int SpecimenScoreHigh = 0;
    public static int horizontalLow = 350;
    public static int horizontalMax = 700;
    public static int afterTransfer = -200;
    public static int BucketValue = -1000;
    public static int BucketValueThroughBore = 0;

}
