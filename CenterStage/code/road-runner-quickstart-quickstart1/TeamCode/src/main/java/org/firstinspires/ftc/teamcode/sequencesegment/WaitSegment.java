package org.firstinspires.ftc.teamcode.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.List;
@Disabled
public final class WaitSegment extends SequenceSegment {
    public WaitSegment(Pose2d pose, double seconds, List<TrajectoryMarker> markers) {
        super(seconds, pose, pose, markers);
    }
}
