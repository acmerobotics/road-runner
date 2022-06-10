package com.acmerobotics.roadrunner

import org.junit.jupiter.api.Test
import org.knowm.xchart.QuickChart
import org.knowm.xchart.XYChart
import kotlin.math.PI
import kotlin.math.sqrt
import kotlin.random.Random
import kotlin.test.assertEquals

fun <Param> chartPosPath(posPath: PositionPath<Param>) : XYChart {
    val params = range(-1.0, posPath.length + 1.0, 1000)
    val positions = params.map { posPath[it, 4] }

    return QuickChart.getChart(
            "Path", "param", "",
            arrayOf(
//                "x", "y",
                "x'", "y'", "x''", "y''",
                "x'''", "y'''",
                ),
            params.toDoubleArray(),
            arrayOf(
//                positions.map { it.x[0] }.toDoubleArray(),
//                positions.map { it.y[0] }.toDoubleArray(),
                positions.map { it.x[1] }.toDoubleArray(),
                positions.map { it.y[1] }.toDoubleArray(),
                positions.map { it.x[2] }.toDoubleArray(),
                positions.map { it.y[2] }.toDoubleArray(),
                positions.map { it.x[3] }.toDoubleArray(),
                positions.map { it.y[3] }.toDoubleArray(),
            )
        )
    }

fun chartPosePath(posePath: PosePath) : XYChart {
    val params = range(-1.0, posePath.length + 1.0, 1000)
    val poses = params.map { posePath[it, 3] }

    return QuickChart.getChart(
        "Path", "param", "",
        arrayOf(
//                "x", "y", "theta",
            "x'", "y'", "theta'",
            "x''", "y''", "theta''",
        ),
        params.toDoubleArray(),
        arrayOf(
//            poses.map { it.translation.x[0] }.toDoubleArray(),
//            poses.map { it.translation.y[0] }.toDoubleArray(),
//            poses.map { it.rotation.log()[0] }.toDoubleArray(),
            poses.map { it.translation.x[1] }.toDoubleArray(),
            poses.map { it.translation.y[1] }.toDoubleArray(),
            poses.map { it.rotation.velocity()[0] }.toDoubleArray(),
            poses.map { it.translation.x[2] }.toDoubleArray(),
            poses.map { it.translation.y[2] }.toDoubleArray(),
            poses.map { it.rotation.velocity()[1] }.toDoubleArray(),
        )
    )
}

class BuildersTest {
    @Test
    fun test() {
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

        saveChart("posPathBuilder", chartPosPath(posPath))

        val posePath = PosePathBuilder(posPath, Rotation2.exp(0.0))
            .tangentTo((posPath.offsets[0] + posPath.offsets[1]) / 2)
            .splineTo(posPath.length, Rotation2.exp(-PI / 3))
            .build()

//        println(posePath.offsets)
//        posePath.paths.forEach { println(it.length) }
        println(posePath)

        saveChart("posePathBuilder", chartPosePath(posePath))
    }
}
