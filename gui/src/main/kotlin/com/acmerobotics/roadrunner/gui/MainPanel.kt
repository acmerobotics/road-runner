package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import java.awt.BorderLayout
import java.awt.Dimension
import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.Future
import javax.swing.*
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
    private val statusLabel = JLabel()

    private var status: String = ""
        set(value) {
            statusLabel.text = "status: $value"
            field = value
        }

    private var poses = listOf<Pose2d>()
    private var constraints = DEFAULT_CONSTRAINTS
    private var resolution = DEFAULT_RESOLUTION

    private var trajGenExecutor = Executors.newSingleThreadExecutor()
    private var trajGenFuture: Future<*>? = null

    init {
        poseEditorPanel.onPosesUpdateListener = { updateTrajectoryInBackground(it, constraints, resolution) }
        constraintsPanel.onConstraintsUpdateListener = { updateTrajectoryInBackground(poses, it, resolution) }
        trajectoryInfoPanel.onResolutionUpdateListener = { updateTrajectoryInBackground(poses, constraints, it) }

        constraintsPanel.updateConstraints(DEFAULT_CONSTRAINTS)
        trajectoryInfoPanel.updateResolution(DEFAULT_RESOLUTION)

        layout = BorderLayout()

        val content = JPanel()
        content.layout = BoxLayout(content, BoxLayout.PAGE_AXIS)

        val upperTabbedPane = JTabbedPane()
        upperTabbedPane.addTab("Field", fieldPanel)
        upperTabbedPane.addTab("Trajectory", trajectoryGraphPanel)
        content.add(upperTabbedPane)

        content.add(trajectoryInfoPanel)

        val lowerTabbedPane = JTabbedPane()
        val panel = JPanel()
        panel.add(poseEditorPanel)
        lowerTabbedPane.addTab("Poses", panel)
        lowerTabbedPane.addTab("Constraints", constraintsPanel)
        content.add(lowerTabbedPane)

        add(content, BorderLayout.CENTER)

        val statusPanel = JPanel()
        statusPanel.preferredSize = Dimension(width, 16)
        statusPanel.layout = BoxLayout(statusPanel, BoxLayout.X_AXIS)

        statusLabel.horizontalAlignment = SwingConstants.LEFT
        statusPanel.add(statusLabel)

        add(statusPanel, BorderLayout.SOUTH)

        status = "ready"
    }

    private fun updateTrajectoryInBackground(poses: List<Pose2d>, constraints: DriveConstraints, resolution: Double) {
        this.poses = poses
        this.constraints = constraints
        this.resolution = min(MIN_RESOLUTION, max(MAX_RESOLUTION, resolution))

        if (trajGenFuture?.isDone == false) {
            trajGenExecutor.shutdownNow()
            status = "interrupted"

            trajGenExecutor = Executors.newSingleThreadExecutor()
        }

        trajGenFuture = trajGenExecutor.submit {
            status = "generating trajectory..."

            val trajectory = TrajectoryConfig(this.poses, this.constraints, this.resolution).toTrajectory()

            poseEditorPanel.trajectoryValid = trajectory != null

            if (trajectory != null) {
                fieldPanel.updateTrajectoryAndPoses(trajectory, poses)
                trajectoryInfoPanel.updateTrajectory(trajectory)
                trajectoryGraphPanel.updateTrajectory(trajectory)

                onTrajectoryUpdateListener?.invoke()
            }

            status = "done"
        }
    }

    fun clearTrajectory() {
        updateTrajectoryInBackground(listOf(), DEFAULT_CONSTRAINTS, DEFAULT_RESOLUTION)
    }

    fun save(file: File) {
        TrajectoryLoader.saveConfig(TrajectoryConfig(poses, constraints, resolution), file)
    }

    fun load(file: File) {
        val trajectoryConfig = TrajectoryLoader.loadConfig(file)
        updateTrajectoryInBackground(trajectoryConfig.poses, trajectoryConfig.constraints, trajectoryConfig.resolution)
        poseEditorPanel.updatePoses(poses)
        constraintsPanel.updateConstraints(constraints)
        trajectoryInfoPanel.updateResolution(resolution)
    }
}
