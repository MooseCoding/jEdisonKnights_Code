package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //Blue bottom left
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 72, -Math.PI/2))
                                .splineTo(new Vector2d(-45, 34), -Math.PI/2)
                                .back(7)
                                .turn(-Math.PI/2)
                                .back(10)

                                .strafeLeft(30)
                                .back(84)
                                .strafeRight(25)
                                .forward(15)
                                .splineTo(new Vector2d(56, 10), Math.PI/2)
                                .build());
            */

        //Blue bottom Middle
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 72-18, -Math.PI/2))
                                .forward(48)
                                .turn(Math.PI)
                                .forward(10)
                                .back(7)
                                .turn(Math.PI/2)
                                .back(85)
                                .strafeRight(24)
                                .forward(10)
                                .splineTo(new Vector2d(56, 10), Math.PI)
                                .build());
        */
        //Blue bottom left
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 72-18, -Math.PI/2))
                                .splineTo(new Vector2d(-25, 34), -Math.PI/2)
                                .turn(Math.PI/2)
                                .back(10)
                                .strafeRight(25)
                                .turn(Math.PI)
                                .back(83)
                                .strafeRight(20)
                                .forward(15)
                                .splineTo(new Vector2d(56, 10), Math.PI/2)
                                .build());
        */


        //Red bottom right
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -72+18, Math.PI/2))
                                .forward(5)
                                .splineTo(new Vector2d(-24, -30), Math.PI/4)
                                .back(10)
                                .turn(-Math.PI/4)

                        .back(10)
                                .strafeLeft(24)
                                .turn(Math.PI)
                                .back(89)
                                .strafeLeft(26)
                                .forward(15)
                                .splineTo(new Vector2d(56, -10), Math.PI)

                                .build());

            */
        //Red bottom middle

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(40, 22, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -72+18, Math.PI/2))
                                .forward(50)
                                .turn(Math.PI)
                                .forward(4)
                                .back(7)
                                .turn(-Math.PI/2)
                                .back(85)
                                .strafeLeft(16)

                                .forward(10)
                        .splineTo(new Vector2d(56, -10), Math.PI)
                        .build());


        //Red bottom left
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -72+18, Math.PI/2))
                                .splineTo(new Vector2d(-46, -35), Math.PI/2)
                                .back(6)
                                .turn(-Math.PI/2)
                                .forward(12)
                                .turn(Math.PI/2)
                                .forward(31)
                                .turn(Math.PI/2)
                                .back(83)
                                .strafeLeft(20)
                                .forward(10)
                                .splineTo(new Vector2d(56, -10), Math.PI/2)
                                .build());
           */
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}