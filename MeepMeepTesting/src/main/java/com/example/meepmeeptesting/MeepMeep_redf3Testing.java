package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep_redf3Testing {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        // RED_F3 up to the basket.
        myBot1.runAction(myBot1.getDrive().actionBuilder(new Pose2d(8.5, -62.25, Math.toRadians(90)))
                .lineToY(-43.5)
                .lineToY(-45)
                .splineToConstantHeading(new Vector2d(35, -36), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(40,-10),Math.toRadians(0))
                .lineToX(46)
                .waitSeconds(0.1)
                .lineToY(-55)
                .splineToConstantHeading(new Vector2d(48, -10), Math.toRadians(-90))
                .waitSeconds(0.1)
                //.lineToX(55)
                .splineToConstantHeading(new Vector2d(58,-55),Math.toRadians(-90))
                .waitSeconds(0.1)
                .lineToY(-11.5)
                .splineToConstantHeading(new Vector2d(61.3,-55),Math.toRadians(-90))

                //.lineToX(75)
                .build());

        //RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                //.build();

        // RED_F3 up to the bas0ket.
        //myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(48, -10, Math.toRadians(90)))

                //.lineToX(48)
                //.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot1)
                //.addEntity(myBot2)
                .start();
    }

}