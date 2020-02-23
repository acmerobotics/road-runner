package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

private val INITIAL_POSE = Pose2d(0.0, 0.0, 2.0)

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class LocalizerTest {
    @Test
    fun testLocalizerQuiescence() {
        val localizer = MecanumDrive.MecanumLocalizer(object : MecanumDrive(0.0, 0.0, 0.0, 1.0) {
            override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
                throw UnsupportedOperationException()
            }

            override fun getWheelPositions() = listOf(0.0, 0.0, 0.0, 0.0)

            override val rawExternalHeading = 0.01
        })

        localizer.poseEstimate = INITIAL_POSE

        repeat(100_000) {
            localizer.update()
        }

        assert(localizer.poseEstimate epsilonEqualsHeading INITIAL_POSE)
    }
}
