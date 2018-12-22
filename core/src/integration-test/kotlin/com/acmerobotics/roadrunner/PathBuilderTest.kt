package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.PathBuilder
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PathBuilderTest {

    private fun testPath(name: String, path: Path) {
        GraphUtil.savePath("pathBuilder/$name", path)
        GraphUtil.savePathPositions("pathBuilder/${name}Pos", path)
        GraphUtil.savePathDerivatives("pathBuilder/${name}Deriv", path)
    }

    @Test
    fun testLines() {
        testPath("lines", PathBuilder(Pose2d())
                .lineTo(Vector2d(10.0, 20.0))
                .lineTo(Vector2d(-23.0, 43.0))
                .build())
    }

    @Test
    fun testStrafeLines() {
        testPath("strafeLines", PathBuilder(Pose2d())
                .strafeTo(Vector2d(10.0, 20.0))
                .strafeTo(Vector2d(-23.0, 43.0))
                .build())
    }

    @Test
    fun testLineHelpers1() {
        testPath("lineHelpers1", PathBuilder(Pose2d())
                .strafeLeft(10.0)
                .forward(20.0)
                .strafeRight(5.0)
                .build())
    }

    @Test
    fun testLineHelpers2() {
        testPath("lineHelpers2", PathBuilder(Pose2d())
                .strafeLeft(15.0)
                .back(20.0)
                .strafeRight(5.0)
                .build())
    }
}