package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathBuilder
import com.acmerobotics.roadrunner.path.PathBuilderException
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PathBuilderTest {
    private fun assertPathBuilderException(generatePath: () -> Unit) {
        try {
            generatePath()
            assert(false)
        } catch (e: PathBuilderException) {
            assert(true)
        } catch (t: Throwable) {
            assert(false)
        }
    }

    @Test
    fun testTangentReversal() {
        assertPathBuilderException {
            PathBuilder(Pose2d())
                .lineTo(Vector2d(10.0, 0.0))
                .lineTo(Vector2d(-10.0, 0.0))
                .build()
            assert(false)
        }
    }

    @Test
    fun testImproperHeadingInterpSeq() {
        assertPathBuilderException {
            PathBuilder(Pose2d())
                .lineToLinearHeading(Pose2d(10.0, 0.0, PI / 2))
                .lineToLinearHeading(Pose2d(-10.0, 0.0, 0.0))
                .build()
            assert(false)
        }
    }
}
