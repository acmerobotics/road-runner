package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import java.io.File
import javax.swing.BoxLayout
import javax.swing.JPanel
import javax.swing.JTabbedPane
import kotlin.math.max
import kotlin.math.min

private val DEFAULT_CONSTRAINTS = DriveConstraints(25.0, 40.0, 0.0,
    Math.toRadians(180.0), Math.toRadians(360.0), 0.0)
private const val DEFAULT_RESOLUTION = 0.25
private const val MIN_RESOLUTION = 5.0
private const val MAX_RESOLUTION = 0.01

/**
 * Main GUI window panel.
 */
class MainPanel : JPanel() {

    var onTrajectoryUpdateListener: (() -> Unit)? = null

    private val fieldPanel = FieldPanel()
    private val trajectoryGraphPanel = TrajectoryGraphPanel()
    private val trajectoryInfoPanel = TrajectoryInfoPanel()
    private val poseEditorPanel = PoseEditorPanel()
    private val constraintsPanel = ConstraintsPanel()

    private var poses = listOf<Pose2d>()
    private var constraints = DEFAULT_CONSTRAINTS
    private var resolution = DEFAULT_RESOLUTION

    init {
        poseEditorPanel.onPosesUpdateListener = { updateTrajectory(it, constraints, resolution) }
        constraintsPanel.onConstraintsUpdateListener = { updateTrajectory(poses, it, resolution) }
        trajectoryInfoPanel.onResolutionUpdateListener = { updateTrajectory(poses, constraints, it) }

        constraintsPanel.updateConstraints(DEFAULT_CONSTRAINTS)
        trajectoryInfoPanel.updateResolution(DEFAULT_RESOLUTION)

        val upperTabbedPane = JTabbedPane()
        upperTabbedPane.addTab("Field", fieldPanel)
        upperTabbedPane.addTab("Trajectory", trajectoryGraphPanel)

        val lowerTabbedPane = JTabbedPane()
        val panel = JPanel()
        panel.add(poseEditorPanel)
        lowerTabbedPane.addTab("Poses", panel)
        lowerTabbedPane.addTab("Constraints", constraintsPanel)

        layout = BoxLayout(this, BoxLayout.PAGE_AXIS)
        add(upperTabbedPane)
        add(trajectoryInfoPanel)
        add(lowerTabbedPane)
    }

    private fun updateTrajectory(poses: List<Pose2d>, constraints: DriveConstraints, resolution: Double) {
        this.poses = poses
        this.constraints = constraints
        this.resolution = min(MIN_RESOLUTION, max(MAX_RESOLUTION, resolution))

        Thread {
            val trajectory = TrajectoryConfig(this.poses, this.constraints, this.resolution).toTrajectory()

            poseEditorPanel.trajectoryValid = trajectory != null

            if (trajectory != null) {
                fieldPanel.updateTrajectoryAndPoses(trajectory, poses)
                trajectoryInfoPanel.updateTrajectory(trajectory)
                trajectoryGraphPanel.updateTrajectory(trajectory)

                onTrajectoryUpdateListener?.invoke()
            }
        }.start()
    }

    fun clearTrajectory() {
        updateTrajectory(listOf(), DEFAULT_CONSTRAINTS, DEFAULT_RESOLUTION)
    }

    fun save(file: File) {
        TrajectoryLoader.saveConfig(TrajectoryConfig(poses, constraints, resolution), file)
    }

    fun load(file: File) {
        val trajectoryConfig = TrajectoryLoader.loadConfig(file)
        updateTrajectory(trajectoryConfig.poses, trajectoryConfig.constraints, trajectoryConfig.resolution)
        poseEditorPanel.updatePoses(poses)
        constraintsPanel.updateConstraints(constraints)
        trajectoryInfoPanel.updateResolution(resolution)
    }
}
