package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SampleMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(0), 11.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
//                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-8, -59), Math.toRadians(0))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(29, -50), Math.toRadians(0))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-45, -53), Math.toRadians(0))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-45, -67), Math.toRadians(0))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-45, -67), Math.toRadians(0))
                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-39, -10), Math.toRadians(0))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-20, -10), Math.toRadians(0))
                        .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-54, -43.1), Math.toRadians(90))
//                        .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-54, -43.11), Math.toRadians(120))
//                        .waitSeconds(2)
//                        .turn(Math.toRadians(90))
//                        .waitSeconds(2)
//                    .turn(Math.toRadians(70))
                .waitSeconds(2)
               .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}