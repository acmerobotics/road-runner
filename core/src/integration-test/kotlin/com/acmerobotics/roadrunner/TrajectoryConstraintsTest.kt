package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.DoubleProgression
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.QuickChart
import org.knowm.xchart.style.MatlabTheme
import java.util.*
import kotlin.math.PI
import kotlin.math.max

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TrajectoryConstraintsTest {
    @Test
    fun testBuilderConstraintsOverride() {
        val trajectory = TrajectoryBuilder(Pose2d(),
            constraints = DriveConstraints(20.0, 10.0, 0.0, PI, PI, 0.0))
            .lineTo(Vector2d(30.0, 0.0))
            .lineTo(Vector2d(50.0, 0.0),
                constraintsOverride = DriveConstraints(10.0, 10.0, 0.0, PI, PI, 0.0))
            .build()
        GraphUtil.saveMotionProfile("constraintsOverride", trajectory.profile)
    }
}
