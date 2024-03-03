package com.acmerobotics.roadrunner

import kotlin.math.PI
import kotlin.math.abs
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals

class KinematicsTest {
    @Test
    fun testMecKinematicsComposition() {
        val kinematics = MecanumKinematics(10.0)

        val r = Random.Default
        repeat(100) {
            val t = PoseVelocity2d(
                Vector2d(r.nextDouble(), r.nextDouble()),
                r.nextDouble()
            )

            val vs = kinematics.inverse(PoseVelocity2dDual.constant<Time>(t, 1)).all()

            val t2 = kinematics.forward(
                MecanumKinematics.WheelIncrements(
                    vs[0], vs[1], vs[2], vs[3],
                )
            ).value()

            assertEquals(t.linearVel.x, t2.line.x, 1e-6)
            assertEquals(t.linearVel.y, t2.line.y, 1e-6)
            assertEquals(t.angVel, t2.angle, 1e-6)
        }
    }

    @Test
    fun testMecWheelVelocityLimiting() {
        val kinematics = MecanumKinematics(10.0)

        val posPath = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0),
            1e-6,
        )
            .splineTo(
                Vector2d(15.0, 15.0),
                Rotation2d.exp(PI),
            )
            .splineTo(
                Vector2d(5.0, 35.0),
                Rotation2d.exp(PI / 3),
            )
            .build()
            .first()

        val path = TangentPath(posPath, 0.0)
        val profile = profile(
            TEST_PROFILE_PARAMS,
            path, 0.0,
            kinematics.WheelVelConstraint(10.0),
            ProfileAccelConstraint(-10.0, 10.0),
        ).baseProfile

        val ts = range(0.0, profile.disps.last(), 100)
        val maxWheelVelMag = ts.maxOf { time ->
            val s = profile[time]
            val pose = path[s.value(), 2].reparam(s)
            kinematics.inverse(pose.inverse() * pose.velocity())
                .all()
                .map { it.value() }
                .maxOf { abs(it) }
        }
        assert(maxWheelVelMag < 10.1)

        saveProfiles("mec", TimeProfile(profile))
    }

    @Test
    fun testTankKinematicsComposition() {
        val kinematics = TankKinematics(10.0)

        val r = Random.Default
        repeat(100) {
            val t = PoseVelocity2d(
                Vector2d(r.nextDouble(), 0.0),
                r.nextDouble()
            )

            val vs = kinematics.inverse(PoseVelocity2dDual.constant<Time>(t, 1)).all()

            val t2 = kinematics.forward(
                TankKinematics.WheelIncrements(
                    vs[0], vs[1],
                )
            ).value()

            assertEquals(t.linearVel.x, t2.line.x, 1e-6)
            assertEquals(t.linearVel.y, t2.line.y, 1e-6)
            assertEquals(t.angVel, t2.angle, 1e-6)
        }
    }

    @Test
    fun testTankWheelVelocityLimiting() {
        val kinematics = TankKinematics(10.0)

        val posPath = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0),
            1e-6,
        )
            .splineTo(
                Vector2d(15.0, 15.0),
                Rotation2d.exp(PI),
            )
            .splineTo(
                Vector2d(5.0, 35.0),
                Rotation2d.exp(PI / 3),
            )
            .build()
            .first()

        val path = TangentPath(posPath, 0.0)
        val profile = profile(
            TEST_PROFILE_PARAMS,
            path, 0.0,
            kinematics.WheelVelConstraint(10.0),
            ProfileAccelConstraint(-10.0, 10.0),
        ).baseProfile

        val ts = range(0.0, profile.disps.last(), 100)
        val maxWheelVelMag = ts.maxOf { time ->
            val s = profile[time]
            val pose = path[s.value(), 2].reparam(s)
            kinematics.inverse(pose.inverse() * pose.velocity())
                .all()
                .map { it.value() }
                .maxOf { abs(it) }
        }
        assert(maxWheelVelMag < 10.1)

        saveProfiles("tank", TimeProfile(profile))
    }
}
