package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

public class position {
    private static Pose2d currentPos;
    private static SampleMecanumDrive d;
    public static void setUp(SampleMecanumDrive dr, Pose2d pos) {
        d = dr;
        currentPos = pos;
    }
    public static Pose2d getPosition() {
        updatePos();
        return currentPos;
    }

    public static double getHeading() {
        updatePos();
        if (currentPos.getHeading() < 0) {
            return 2*Math.PI + currentPos.getHeading();
        }
        return currentPos.getHeading();
    }
    public static void updatePos() {
        currentPos = d.getPoseEstimate();
    }
}
