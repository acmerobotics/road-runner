package com.acmerobotics.splinelib.trajectory

import com.acmerobotics.splinelib.Angle
import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.Vector2d
import com.acmerobotics.splinelib.Waypoint
import com.acmerobotics.splinelib.path.*

class TrajectoryBuilder(private var currentPose: Pose2d, private val globalConstraints: DriveConstraints) {
    private val trajectorySegments = mutableListOf<TrajectorySegment>()
    private var paths = mutableListOf<Path>()
    private var constraintsList = mutableListOf<TrajectoryConstraints>()
    private var composite = false
    private var reversed = false

    // TODO: is there a better solution?
    fun reverse(): TrajectoryBuilder {
        reversed = !reversed
        return this
    }

    @JvmOverloads
    fun lineTo(pos: Vector2d, interpolator: HeadingInterpolator = TangentInterpolator(), constraintsOverride: TrajectoryConstraints? = null): TrajectoryBuilder {
        val constraints = constraintsOverride ?: globalConstraints
        val line = if (reversed) {
            Path(LineSegment(pos, currentPose.pos()), interpolator, true)
        } else {
            Path(LineSegment(currentPose.pos(), pos), interpolator, false)
        }
        if (composite) {
            paths.add(line)
            constraintsList.add(constraints)
        } else {
            trajectorySegments.add(PathTrajectorySegment(listOf(line), listOf(constraints)))
        }
        currentPose = Pose2d(pos, currentPose.heading)
        return this
    }

    @JvmOverloads
    fun turn(angle: Double, constraintsOverride: DriveConstraints? = null): TrajectoryBuilder {
        return turnTo(Angle.norm(currentPose.heading + angle), constraintsOverride)
    }

    @JvmOverloads
    fun turnTo(heading: Double, constraintsOverride: DriveConstraints? = null): TrajectoryBuilder {
        val pointTurn = PointTurn(currentPose, heading, constraintsOverride ?: globalConstraints)
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
    fun splineTo(pose: Pose2d, interpolator: HeadingInterpolator = TangentInterpolator(), constraintsOverride: TrajectoryConstraints? = null): TrajectoryBuilder {
        val constraints = constraintsOverride ?: this.globalConstraints
        val derivMag = (currentPose.pos() distanceTo pose.pos())
        val spline = if (reversed) {
            Path(
                    QuinticSplineSegment(
                            Waypoint(pose.x, pose.y, derivMag * Math.cos(pose.heading), derivMag * Math.sin(pose.heading)),
                            Waypoint(currentPose.x, currentPose.y, derivMag * Math.cos(currentPose.heading), derivMag * Math.sin(currentPose.heading))
                    ),
                    interpolator,
                    true
            )
        } else {
            Path(
                    QuinticSplineSegment(
                            Waypoint(currentPose.x, currentPose.y, derivMag * Math.cos(currentPose.heading), derivMag * Math.sin(currentPose.heading)),
                            Waypoint(pose.x, pose.y, derivMag * Math.cos(pose.heading), derivMag * Math.sin(pose.heading))
                    ),
                    interpolator,
                    false
            )
        }
        if (composite) {
            paths.add(spline)
            constraintsList.add(constraints)
        } else {
            trajectorySegments.add(PathTrajectorySegment(listOf(spline), listOf(constraints)))
        }
        currentPose = pose
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
        trajectorySegments.add(PathTrajectorySegment(paths, constraintsList))
        paths = mutableListOf()
        constraintsList = mutableListOf()
        return this
    }

    fun build(): Trajectory {
        if (composite) {
            closeComposite()
        }
        return Trajectory(trajectorySegments)
    }
}
