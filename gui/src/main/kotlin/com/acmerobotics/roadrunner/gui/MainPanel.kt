package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.EmptyPathException
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
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
    private val pathStepPanel = PathStepEditorPanel()
    private val configPanel = ConfigPanel()
    private val statusLabel = JLabel()

    private var status: String = ""
        set(value) {
            statusLabel.text = "status: $value"
            field = value
        }

    private var trajectoryConfig: TrajectoryConfig = TrajectoryConfig(
        Pose2d(),
        null,
        emptyList(),
        DEFAULT_RESOLUTION,
        false
    )
    private var groupConfig: TrajectoryGroupConfig = TrajectoryGroupConfig(
        DEFAULT_CONSTRAINTS,
        TrajectoryGroupConfig.DistanceUnit.INCH,
        TrajectoryGroupConfig.DriveType.GENERIC,
        null,
        null,
        null
    )

    private var trajGenExecutor = Executors.newSingleThreadExecutor()
    private var trajGenFuture: Future<*>? = null

    init {
        pathStepPanel.onUpdateListener = { startPose, startHeading, steps ->
            trajectoryConfig.startPose = startPose
            trajectoryConfig.startHeading = startHeading
            trajectoryConfig.steps = steps

            updateTrajectoryInBackground()
        }
        trajectoryInfoPanel.onResolutionUpdateListener = {
            trajectoryConfig.resolution = min(MIN_RESOLUTION, max(it, MAX_RESOLUTION))

            updateTrajectoryInBackground()
        }
        configPanel.onUpdateListener = {
            groupConfig = it

            updateTrajectoryInBackground()
        }

        pathStepPanel.update(trajectoryConfig.startPose, trajectoryConfig.startHeading, trajectoryConfig.steps)
        trajectoryInfoPanel.updateResolution(trajectoryConfig.resolution)
        configPanel.update(groupConfig)

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
        panel.add(pathStepPanel)
        lowerTabbedPane.addTab("Path", panel)
        lowerTabbedPane.addTab("Config", configPanel)
        content.add(lowerTabbedPane)

        add(content, BorderLayout.CENTER)

        val statusPanel = JPanel()
        statusPanel.preferredSize = Dimension(width, 16)
        statusPanel.layout = BoxLayout(statusPanel, BoxLayout.X_AXIS)

        statusLabel.horizontalAlignment = SwingConstants.LEFT
        statusPanel.add(statusLabel)

        add(statusPanel, BorderLayout.SOUTH)
    }

    private fun updateTrajectoryInBackground() {
        if (trajGenFuture?.isDone == false) {
            trajGenExecutor.shutdownNow()
            status = "interrupted"

            trajGenExecutor = Executors.newSingleThreadExecutor()
        }

        trajGenFuture = trajGenExecutor.submit {
            status = "generating trajectory..."

            try {
                val trajectory = trajectoryConfig.toTrajectory(groupConfig)

                pathStepPanel.trajectoryValid = trajectory != null

                if (trajectory != null) {
                    fieldPanel.updateTrajectoryAndConfig(trajectory, trajectoryConfig)
                    trajectoryInfoPanel.updateTrajectory(trajectory)
                    trajectoryGraphPanel.updateTrajectory(trajectory)

                    onTrajectoryUpdateListener?.invoke()
                }

                status = "done"
            } catch (e: EmptyPathException) {
                status = "error: empty path"

                pathStepPanel.trajectoryValid = false
            } catch (e: EmptyPathSegmentException) {
                status = "error: empty path segment"

                pathStepPanel.trajectoryValid = false
            } catch (e: PathContinuityViolationException) {
                status = "error: invalid sequence of heading interpolators"

                pathStepPanel.trajectoryValid = false
            } catch (e: IllegalArgumentException) {
                status = "error: tank is constrained to tangent heading interpolation"

                pathStepPanel.trajectoryValid = false
            } catch (t: Throwable) {
                status = "error: ${t.javaClass}"

                t.printStackTrace()

                pathStepPanel.trajectoryValid = false
            }
        }
    }

    fun clearTrajectory() {
//        updateTrajectoryInBackground(listOf(), DEFAULT_CONSTRAINTS, DEFAULT_RESOLUTION)
    }

    fun save(file: File) {
//        TrajectoryConfigManager.saveConfig(
//            LegacyTrajectoryConfig(
//                poses,
//                constraints,
//                resolution
//            ), file)
    }

    fun load(file: File) {
//        val trajectoryConfig = TrajectoryConfigManager.loadConfig(file) ?: return
//        val trajectoryGroupConfig = TrajectoryConfigManager.loadGroupConfig(file) ?: return
//        updateTrajectoryInBackground(trajectoryConfig.steps.map { it.pose }, trajectoryGroupConfig.specificConstraints, trajectoryConfig.resolution)
//        poseEditorPanel.update(trajectoryConfig.startPose, trajectoryConfig.startHeading, trajectoryConfig.steps)
//        configPanel.update(trajectoryGroupConfig)
//        trajectoryInfoPanel.updateResolution(resolution)
    }
}
