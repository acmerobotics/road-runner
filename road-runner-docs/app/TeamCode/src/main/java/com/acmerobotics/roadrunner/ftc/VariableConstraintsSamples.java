package com.acmerobotics.roadrunner.ftc;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;

public class VariableConstraintsSamples {
    public void makePath() {
        final Pose2d beginPose = new Pose2d(0, 0, 0);
        final double eps = 1e-6;
        final double beginEndVel = 0.0;
        final double dispResolution = 0.25;
        final double angResolution = 1e-2;

        final TrajectoryBuilderParams builderParams = new TrajectoryBuilderParams(
                eps, new ProfileParams(dispResolution, angResolution, eps)
        );

        {
            // sample: slowedSegmentBase
            VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(50.0),
                    new AngularVelConstraint(Math.PI / 2)
            ));
            AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);
            // end sample

            TrajectoryBuilder builder = new TrajectoryBuilder(builderParams, beginPose, beginEndVel,
                    baseVelConstraint, baseAccelConstraint);

            // sample: uniformConstraints
            builder
                    .splineTo(new Vector2d(0.0, 48.0), 0.0)
                    .splineTo(new Vector2d(48.0, 0.0), -Math.PI / 2)
                    .splineTo(new Vector2d(0.0, -48.0), -Math.PI)
                    .splineTo(new Vector2d(-48.0, 0.0), Math.PI / 2)
                    .build();
            // end sample

            // sample: slowedSegment
            builder
                    .splineTo(new Vector2d(0.0, 48.0), 0.0)
                    .splineTo(new Vector2d(48.0, 0.0), -Math.PI / 2,
                            // only override velocity constraint
                            new TranslationalVelConstraint(20.0))
                    .splineTo(new Vector2d(0.0, -48.0), -Math.PI,
                            // skip velocity constraint and only override acceleration constraint
                            null,
                            new ProfileAccelConstraint(-10.0, 10.0))
                    // revert back to the base constraints
                    .splineTo(new Vector2d(-48.0, 0.0), Math.PI / 2)
                    .build();
            // end sample
        }

        {
            // sample: slowedHalfBase
            VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
                if (robotPose.position.x.value() > 24.0) {
                    return 20.0;
                } else {
                    return 50.0;
                }
            };
            // end sample

            AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);
            TrajectoryBuilder builder = new TrajectoryBuilder(builderParams, beginPose, beginEndVel,
                    baseVelConstraint, baseAccelConstraint);

            // sample: slowedHalf
            builder
                    .splineTo(new Vector2d(0.0, 48.0), 0.0)
                    .splineTo(new Vector2d(48.0, 0.0), -Math.PI / 2)
                    .splineTo(new Vector2d(0.0, -48.0), -Math.PI)
                    .splineTo(new Vector2d(-48.0, 0.0), Math.PI / 2)
                    .build();
            // end sample
        }
    }
}
