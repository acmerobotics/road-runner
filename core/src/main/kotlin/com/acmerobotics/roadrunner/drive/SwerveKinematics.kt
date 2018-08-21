package com.acmerobotics.roadrunner.drive

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.cos
import kotlin.math.sin

object SwerveKinematics {
    fun robotToModuleVelocityVectors(robotPoseVelocity: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth): List<Vector2d> {
        val x = wheelBase / 2
        val y = trackWidth / 2

        val vx = robotPoseVelocity.x
        val vy = robotPoseVelocity.y
        val omega = robotPoseVelocity.heading

        return listOf(
                Vector2d(vx - omega * y, vy + omega * x),
                Vector2d(vx - omega * y, vy - omega * x),
                Vector2d(vx + omega * y, vy - omega * x),
                Vector2d(vx + omega * y, vy + omega * x)
        )
    }

    fun robotToWheelVelocities(robotPoseVelocity: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth) =
            robotToModuleVelocityVectors(robotPoseVelocity, trackWidth, wheelBase).map(Vector2d::norm)

    fun robotToModuleOrientations(robotPoseVelocity: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth) =
            robotToModuleVelocityVectors(robotPoseVelocity, trackWidth, wheelBase).map(Vector2d::angle)

    fun robotToModuleAccelerationVectors(robotPoseAcceleration: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth): List<Vector2d> {
        val x = wheelBase / 2
        val y = trackWidth / 2

        val ax = robotPoseAcceleration.x
        val ay = robotPoseAcceleration.y
        val alpha = robotPoseAcceleration.heading

        return listOf(
                Vector2d(ax - alpha * y, ay + alpha * x),
                Vector2d(ax - alpha * y, ay - alpha * x),
                Vector2d(ax + alpha * y, ay - alpha * x),
                Vector2d(ax + alpha * y, ay + alpha * x)
        )
    }

    fun robotToWheelAccelerations(robotPoseVelocity: Pose2d, robotPoseAcceleration: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth) =
            robotToModuleVelocityVectors(robotPoseVelocity, trackWidth, wheelBase)
                    .zip(robotToModuleAccelerationVectors(robotPoseAcceleration, trackWidth, wheelBase))
                    .map { (it.first.x * it.second.x + it.first.y * it.second.y) / it.first.norm() }

    fun robotToModuleAngularVelocities(robotPoseVelocity: Pose2d, robotPoseAcceleration: Pose2d, trackWidth: Double, wheelBase: Double = trackWidth) =
            robotToModuleVelocityVectors(robotPoseVelocity, trackWidth, wheelBase)
                    .zip(robotToModuleAccelerationVectors(robotPoseAcceleration, trackWidth, wheelBase))
                    .map { (it.first.x * it.second.y - it.first.y * it.second.x) / (it.first.x * it.first.x + it.first.y * it.first.y) }

    fun wheelToRobotVelocities(wheelVelocities: List<Double>, moduleOrientations: List<Double>, trackWidth: Double, wheelBase: Double = trackWidth): Pose2d {
        val x = wheelBase / 2
        val y = trackWidth / 2

        val vectors = wheelVelocities
                .zip(moduleOrientations)
                .map { Vector2d(it.first * cos(it.second), it.first * sin(it.second)) }

        val vx = vectors.sumByDouble { it.x } / 4
        val vy = vectors.sumByDouble { it.y } / 4
        val omega = (y * (vectors[2].x + vectors[3].x - vectors[0].x - vectors[1].x)
                + x * (vectors[0].y + vectors[2].y - vectors[1].y - vectors[3].y)) / (4 * (x * x + y * y))

        return Pose2d(vx, vy, omega)
    }
}