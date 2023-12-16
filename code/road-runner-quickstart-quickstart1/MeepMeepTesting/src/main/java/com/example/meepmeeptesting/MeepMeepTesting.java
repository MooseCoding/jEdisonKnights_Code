package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        /* Blue bottom left
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 72, -Math.PI/2))
                                .splineTo(new Vector2d(-45, 34), -Math.PI/2)
                                .back(2)
                                .turn(Math.PI/2)
                                .splineTo(new Vector2d(-16, 35), Math.toRadians(0))
                                .splineTo(new Vector2d(65-23, 35), Math.PI)
                                .splineTo(new Vector2d(65-30, 35), Math.PI)
                                .splineTo(new Vector2d(56, 10), Math.PI/2)
                                .build());
        */

        //Blue bottom Middle

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 72, -Math.PI/2))
                                .splineTo(new Vector2d(-35, 34), Math.PI/2)
                                .splineTo(new Vector2d(-16, 35), Math.toRadians(0))
                                .splineTo(new Vector2d(65-23, 35), Math.PI)
                                .splineTo(new Vector2d(65-30, 35), Math.PI)
                                .splineTo(new Vector2d(56, 10), Math.PI/2)
                                .build());

        /* Blue bottom left
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 72, -Math.PI/2))
                                .splineTo(new Vector2d(-25, 34), -Math.PI/2)
                                .turn(Math.PI/2)
                                .splineTo(new Vector2d(-16, 35), Math.toRadians(0))
                                .splineTo(new Vector2d(65-23, 35), Math.PI)
                                .splineTo(new Vector2d(65-30, 35), Math.PI)
                                .splineTo(new Vector2d(56, 10), Math.PI/2)
                                .build());
        */


        //Red bottom right
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -72, Math.PI/2))
                                .splineTo(new Vector2d(-28, -30), Math.PI/2)
                                .back(4)
                                .turn(-Math.PI/2)
                                .splineTo(new Vector2d(-16, -34), 0)
                                .splineTo(new Vector2d(65-20, -34), Math.PI)

                                .splineTo(new Vector2d(65-24, -34), Math.PI)
                                .splineTo(new Vector2d(56, -10), Math.PI)
                                .build());
        */

        //Red bottom middle
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -72, Math.PI/2))
                                .splineTo(new Vector2d(-34, -32), Math.PI/2)
                                .splineTo(new Vector2d(-34, -32-15), Math.PI/2)
                                .splineTo(new Vector2d(-34, -32-15+7), Math.PI/2)
                        .splineTo(new Vector2d(-16, -34), 0)
                        .splineTo(new Vector2d(65-20, -34), Math.PI)

                                .splineTo(new Vector2d(65-24, -34), Math.PI)
                        .splineTo(new Vector2d(56, -10), Math.PI)
                        .build());
        */

        //Red bottom left
        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(43.3, 30, Math.toRadians(180), Math.toRadians(180), 18.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -72, Math.PI/2))
                                .splineTo(new Vector2d(-48, -33), Math.PI/2)
                                .back(6)
                                .turn(-Math.PI/2)
                                .splineTo(new Vector2d(-16, -34), 0)
                                .splineTo(new Vector2d(65-20, -34), 0)
                                .turn(Math.PI)
                                .splineTo(new Vector2d(65-24, -34), Math.PI)
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