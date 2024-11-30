package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import kotlin.math.PI

class BuildersChartTest {
    @Test
    fun chartPath() {
        val paths = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0),
            1e-6,
        )
            .lineToX(10.0)
            .splineTo(Vector2d(20.0, -10.0), -PI / 2)
            .build()
        require(paths.size == 1)
        val path = paths.first()

        val posePath = PosePathSeqBuilder(path, 0.0)
            .linearUntil(3.0, PI / 2)
            .splineUntil(5.0, PI / 2)
            .constantUntil(10.0)
            .splineUntil(12.0, PI / 3)
            .tangentUntilEnd()
            .first()

        saveChartPanel(
            "panels/basic", 1000,
            listOf(
                chartPosePathXY(posePath),
                chartPosePathHeading(posePath)
            )
        )
    }

    // https://github.com/acmerobotics/road-runner/issues/107
    @Test
    fun chartIssue107() {
        val paths = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0),
            1e-6,
        )
            .splineTo(Vector2d(30.0, 30.0), PI / 2)
            .splineTo(Vector2d(0.0, 60.0), PI)
            .build()
        require(paths.size == 1)
        val path = paths.first()

        val posePath = PosePathSeqBuilder(path, 0.0)
            .tangentUntilEnd()
            .first()

        saveChartPanel(
            "panels/issue107", 1000,
            listOf(
                // chartPosePathXY(posePath),
                chartPosePath(posePath),
                chartPosePathHeading(posePath),
            )
        )
    }
}
