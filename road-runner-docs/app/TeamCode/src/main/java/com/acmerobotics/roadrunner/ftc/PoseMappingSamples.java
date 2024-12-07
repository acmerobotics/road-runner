package com.acmerobotics.roadrunner.ftc;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

public class PoseMappingSamples {
    public void makePath() {
        final Pose2d beginPose = new Pose2d(0, 0, 0);
        final double eps = 1e-6;
        final double beginEndVel = 0.0;
        final TranslationalVelConstraint baseVelConstraint = new TranslationalVelConstraint(30.0);
        final ProfileAccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);
        final double dispResolution = 0.25;
        final double angResolution = 1e-2;

        // using the variables in scope
        // sample: xReflection
        new TrajectoryBuilder(new TrajectoryBuilderParams(eps,
                new ProfileParams(dispResolution, angResolution, eps)),
                beginPose, beginEndVel,
                baseVelConstraint, baseAccelConstraint,
                pose -> new Pose2dDual<>(
                        pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse()));
        // end sample
    }
}
