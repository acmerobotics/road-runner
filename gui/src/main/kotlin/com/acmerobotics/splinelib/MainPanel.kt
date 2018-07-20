package com.acmerobotics.splinelib

import com.acmerobotics.splinelib.trajectory.DriveConstraints
import com.acmerobotics.splinelib.trajectory.Trajectory
import com.acmerobotics.splinelib.trajectory.TrajectoryBuilder
import javax.swing.BoxLayout
import javax.swing.JPanel
import javax.swing.JTabbedPane
import kotlin.math.abs

class MainPanel : JPanel() {
    companion object {
        val DEFAULT_CONSTRAINTS = DriveConstraints(25.0, 40.0, Math.toRadians(180.0), Math.toRadians(360.0), 40.0)

        fun posesToTrajectory(poses: List<Pose2d>, constraints: DriveConstraints) =
        // update the trajectory
                if (poses.isEmpty()) {
                    Trajectory()
                } else {
                    val builder = TrajectoryBuilder(poses.first(), constraints)
                    for (i in 1 until poses.size) {
                        val startPose = poses[i - 1]
                        val endPose = poses[i]
                        if (abs(startPose.x - endPose.x) < FieldPanel.EPSILON && abs(startPose.y - endPose.y) < FieldPanel.EPSILON) {
                            // this is probably a turn
                            builder.turnTo(endPose.heading)
                        } else {
                            builder.beginComposite()
                            val diff = endPose - startPose
                            val cosAngle = (Math.cos(endPose.heading) * diff.x + Math.sin(endPose.heading) * diff.y) / diff.pos().norm()

                            builder.setReversed(cosAngle < 0)

                            if (abs(startPose.heading - endPose.heading) < FieldPanel.EPSILON && abs(1 - abs(cosAngle)) < FieldPanel.EPSILON) {
                                // this is probably a line
                                builder.lineTo(endPose.pos())
                            } else {
                                // this is probably a spline
                                builder.splineTo(endPose)
                            }
                        }
                    }
                    builder.build()
                }
    }

    private val fieldPanel = FieldPanel()
    private val trajectoryInfoPanel = TrajectoryInfoPanel()
    private val poseEditorPanel = PoseEditorPanel()
    private val constraintsPanel = ConstraintsPanel()

    private var poses = listOf<Pose2d>()
    private var constraints = DEFAULT_CONSTRAINTS

    fun updateTrajectory(poses: List<Pose2d>, constraints: DriveConstraints) {
        this.poses = poses
        this.constraints = constraints

        val trajectory = posesToTrajectory(poses, constraints)
        fieldPanel.updateTrajectoryAndPoses(trajectory, poses)
        trajectoryInfoPanel.updateTrajectory(trajectory)
    }

    init {
        poseEditorPanel.onPosesUpdateListener = { updateTrajectory(it, constraints) }
        constraintsPanel.onConstraintsUpdateListener = { updateTrajectory(poses, it) }

        constraintsPanel.updateConstraints(DEFAULT_CONSTRAINTS)

        val tabbedPane = JTabbedPane()
        val panel = JPanel()
        panel.add(poseEditorPanel)
        tabbedPane.addTab("Poses", panel)
        tabbedPane.addTab("Constraints", constraintsPanel)

        layout = BoxLayout(this, BoxLayout.PAGE_AXIS)
        add(fieldPanel)
        add(trajectoryInfoPanel)
        add(tabbedPane)
    }
}