package util;

import common.control.*;
import common.math.Rotation2;
import common.math.Vector2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory autonPlaybackTrajectory;
    private Trajectory PathfinderTestPartOne;
    private Trajectory PathfinderTestPartTwo;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(1.0);     // 1m/s
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(1.0); // 1m/s^2

        autonPlaybackTrajectory = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.ZERO)
                        .lineTo(new Vector2(0,0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        PathfinderTestPartOne = new Trajectory(
            new SimplePathBuilder(new Vector2(0,0), Rotation2.ZERO)
                    .lineTo(new Vector2(1.25,0))
                    .build(),
            slowConstraints, SAMPLE_DISTANCE
        );
        PathfinderTestPartTwo = new Trajectory(
            new SimplePathBuilder(new Vector2(1.25,0), Rotation2.fromDegrees(0.0))
                    .lineTo(new Vector2(0,0))
                    .build(),
            slowConstraints, SAMPLE_DISTANCE
        );
    }

    public Trajectory getAutonPlaybackTrajectory() {
        return autonPlaybackTrajectory;    
    }

    public Trajectory getPathfinderTestPartOne() {
        return PathfinderTestPartOne;
    }
    public Trajectory getPathfinderTestPartTwo() {
        return PathfinderTestPartTwo;
    }
}