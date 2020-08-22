package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathBuilder
import com.acmerobotics.roadrunner.trajectory.SpatialMarker
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import org.knowm.xchart.Histogram
import org.knowm.xchart.QuickChart
import kotlin.math.PI
import kotlin.random.Random

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class MarkerTest {
    @Test
    fun testSpatialMarkerProj() {
        val trials = 1000

        val seed = System.currentTimeMillis()
        println("seed: $seed")
        val random = Random(seed)

        val constraints = DriveConstraints(25.0, 50.0, 50.0, 1.0, 1.0, 1.0)

        val distances = mutableListOf<Double>()
        val path = PathBuilder(Pose2d(10 * random.nextDouble(), 10 * random.nextDouble(), 2 * PI * random.nextDouble()))
            .splineTo(Vector2d(10 * random.nextDouble(), 10 * random.nextDouble()), 2 * PI * random.nextDouble())
            .splineTo(Vector2d(10 * random.nextDouble(), 10 * random.nextDouble()), 2 * PI * random.nextDouble())
            .splineTo(Vector2d(10 * random.nextDouble(), 10 * random.nextDouble()), 2 * PI * random.nextDouble())
            .splineTo(Vector2d(10 * random.nextDouble(), 10 * random.nextDouble()), 2 * PI * random.nextDouble())
            .build()

        repeat(trials) {
            val displacement = path.length() * random.nextDouble()
            val position = path[displacement].vec()
            val trajectory = TrajectoryGenerator.generateTrajectory(path, constraints, spatialMarkers = listOf(
                SpatialMarker(position) { }
            ), resolution = 0.01)
            val distance = position distTo trajectory[trajectory.markers[0].time].vec()
            if (distance > 0.01) {
                println(position)
            }
            distances.add(distance)
        }

        val hist = Histogram(distances, 15)
        val chart = QuickChart.getChart("Spatial Marker Projection Error",
            "Error", "Frequency", "Error",
            hist.getxAxisData(), hist.getyAxisData())
        GraphUtil.saveGraph("markerError", chart)

        GraphUtil.savePath("markerPath", path)

        assert(distances.max()!! < 0.1)
    }
}
