package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // RED_F3 up to the basket.
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-14.75, -62.25, Math.toRadians(0)))

                .strafeTo(new Vector2d(-30, -45)) // get away from the wall
                // previous .strafeTo(new Vector2d(-14.75, -62.25 + 20))
                //.waitSeconds(1)
                // one of the next 2 is required before the splineTo
                .setReversed(true)
                //.setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-54, -53.375), Math.toRadians(-135)) // previous
                //.strafeToSplineHeading(new Vector2d(-54, -53.375), Math.toRadians(45))
                .waitSeconds(1)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}