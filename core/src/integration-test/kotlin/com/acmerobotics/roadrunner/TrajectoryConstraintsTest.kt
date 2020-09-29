package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectoryConstraintsTest {
    @Test
    fun testBuilderConstraintsOverride() {
        val trajectory = TrajectoryBuilder(Pose2d(),
            baseVelConstraint = MinVelocityConstraint(listOf(
                TranslationalVelocityConstraint(20.0),
                AngularVelocityConstraint(PI / 2)
            )),
            baseAccelConstraint = ProfileAccelerationConstraint(10.0))
            .lineTo(Vector2d(30.0, 0.0))
            .lineTo(Vector2d(50.0, 0.0),
                velConstraintOverride = MinVelocityConstraint(listOf(
                    TranslationalVelocityConstraint(10.0),
                    AngularVelocityConstraint(PI / 2)
                )), accelConstraintOverride = null)
            .build()
        GraphUtil.saveMotionProfile("constraintsOverride", trajectory.profile)
    }
}
