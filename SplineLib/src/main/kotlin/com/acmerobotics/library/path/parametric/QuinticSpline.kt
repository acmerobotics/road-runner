package com.acmerobotics.library.path.parametric

import com.acmerobotics.library.Pose2d
import com.acmerobotics.library.Waypoint

object QuinticSpline {
    fun fromWaypoints(vararg waypoints: Waypoint) =
        (0 until waypoints.lastIndex).map { QuinticSplineSegment(waypoints[it], waypoints[it+1]) }

    fun fromPoses(vararg poses: Pose2d) = fromWaypoints(*poses.map { it.toWaypoint() }.toTypedArray())
}