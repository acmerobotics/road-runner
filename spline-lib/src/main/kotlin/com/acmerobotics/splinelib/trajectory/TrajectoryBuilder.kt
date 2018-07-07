package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Angle
import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.Vector2d
import com.acmerobotics.splinelib.Waypoint
import com.acmerobotics.splinelib.path.*

class TrajectoryBuilder(private var currentPose: Pose2d, private val constraints: DriveConstraints) {
    private val trajectorySegments = mutableListOf<TrajectorySegment>()
    private var paths = mutableListOf<Path>()
    private var composite = false

    @JvmOverloads
    fun lineTo(pos: Vector2d, interpolator: HeadingInterpolator = TangentInterpolator()): TrajectoryBuilder {
        val line = Path(LineSegment(currentPose.pos(), pos), interpolator)
        if (composite) {
            paths.add(line)
        } else {
            trajectorySegments.add(PathTrajectorySegment(listOf(line), listOf(constraints)))
        }
        currentPose = line.end()
        return this
    }

    fun turn(angle: Double): TrajectoryBuilder {
        return turnTo(Angle.norm(currentPose.heading + angle))
    }

    fun turnTo(heading: Double): TrajectoryBuilder {
        val pointTurn = PointTurn(currentPose, heading, constraints)
        trajectorySegments.add(pointTurn)
        currentPose = Pose2d(currentPose.x, currentPose.y, heading)
        return this
    }

    fun forward(distance: Double): TrajectoryBuilder {
        return lineTo(currentPose.pos() + Vector2d(
                distance * Math.cos(currentPose.heading),
                distance * Math.sin(currentPose.heading)
        ))
    }

    fun back(distance: Double): TrajectoryBuilder {
        return forward(-distance)
    }

    fun strafeLeft(distance: Double): TrajectoryBuilder {
        return lineTo(currentPose.pos() + Vector2d(
                distance * Math.cos(currentPose.heading + Math.PI / 2),
                distance * Math.sin(currentPose.heading + Math.PI / 2)
        ))
    }

    fun strafeRight(distance: Double): TrajectoryBuilder {
        return strafeLeft(-distance)
    }

    @JvmOverloads
    fun splineTo(pose: Pose2d, interpolator: HeadingInterpolator = TangentInterpolator()): TrajectoryBuilder {
        val derivMag = (currentPose.pos() distanceTo pose.pos())
        val spline = Path(QuinticSplineSegment(
                Waypoint(currentPose.x, currentPose.y, derivMag * Math.cos(currentPose.heading), derivMag * Math.sin(currentPose.heading)),
                Waypoint(pose.x, pose.y, derivMag * Math.cos(pose.heading), derivMag * Math.sin(pose.heading))),
                interpolator
        )
        if (composite) {
            paths.add(spline)
        } else {
            trajectorySegments.add(PathTrajectorySegment(listOf(spline), listOf(constraints)))
        }
        currentPose = spline.end()
        return this
    }

    fun waitFor(duration: Double): TrajectoryBuilder {
        trajectorySegments.add(WaitSegment(currentPose, duration))
        return this
    }

    fun beginComposite(): TrajectoryBuilder {
        if (composite) {
            closeComposite()
        }
        composite = true
        return this
    }

    fun closeComposite(): TrajectoryBuilder {
        composite = false
        trajectorySegments.add(PathTrajectorySegment(paths, paths.map { constraints }))
        paths = mutableListOf()
        return this
    }

    fun build(): Trajectory {
        if (composite) {
            closeComposite()
        }
        return Trajectory(trajectorySegments)
    }
}
