package com.acmerobotics.roadrunner

import kotlin.test.Test
import kotlin.math.PI
import kotlin.math.abs

object KinematicsTest {
    @Test
    fun testMecWheelVelocityLimiting() {
        val kinematics = MecanumKinematics(10.0)

        val posPath = PositionPathBuilder(
            Position2(0.0, 0.0),
            Rotation2.exp(0.0)
        )
            .splineTo(
                Position2(15.0, 15.0),
                Rotation2.exp(PI),
            )
            .splineTo(
                Position2(5.0, 35.0),
                Rotation2.exp(PI / 3),
            )
            .build()

        val path = TangentPath(posPath, Rotation2.exp(0.0))
        val constraint = kinematics.maxRobotVel(10.0)
        val profile = profile(path.length, 0.0,
            { constraint(path[it, 2]) }, { Interval(-5.0, 5.0) }, 0.01)

        val trajectory = TimeTrajectory(DisplacementTrajectory(path, profile))

        val t = range(0.0, profile.disps.last(), 100)
        val maxWheelVelMag = t.maxOf { time ->
            val pose = trajectory[time, 2]
            kinematics.inverse(pose.inverse() * pose.velocity())
                .all()
                .map { it.constant() }
                .maxOf { abs(it) }
        }
        assert(maxWheelVelMag < 10.1)

        saveChart("mecProfile", chartProfile(profile))
    }
}
