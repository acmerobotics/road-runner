package com.acmerobotics.roadrunner.gui

import DEFAULT_GROUP_CONFIG
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import com.acmerobotics.roadrunner.util.epsilonEquals
import java.awt.*
import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.Future
import javax.swing.*
import kotlin.math.max
import kotlin.math.min

const val MIN_RESOLUTION = 5.0
const val MAX_RESOLUTION = 0.01

/**
 * Main GUI window panel.
 */
class MainPanel : JPanel() {
    private val trajectoryListPanel = TrajectoryListPanel()
    private val fieldPanel = FieldPanel()
    private val trajectoryGraphPanel = TrajectoryGraphPanel()
    private val trajectoryInfoPanel = TrajectoryInfoPanel()
    private val pathEditorPanel = PathEditorPanel()
    private val configPanel = ConfigPanel()
    private val statusLabel = JLabel()

    private var status: String = ""
        set(value) {
            statusLabel.text = "status: $value"
            field = value
        }

    private var trajectoryConfig: TrajectoryConfig? = null
        set(value) {
            if (value != null) {
                updateTrajectoryInBackground(value, groupConfig)

                pathEditorPanel.config = PathConfig(value.startPose, value.startTangent, value.waypoints)
                trajectoryInfoPanel.resolution = value.resolution

                val diskConfig = trajectoryListPanel.diskConfig
                if (diskConfig != null) {
                    trajectoryListPanel.diskConfig = diskConfig.copy(
                        config = value,
                        dirty = true
                    )
                }
            }

            field = value
        }

    private var groupConfig: TrajectoryGroupConfig = DEFAULT_GROUP_CONFIG
        set(value) {
            val trajectoryConfig = trajectoryConfig
            if (trajectoryConfig != null) {
                updateTrajectoryInBackground(trajectoryConfig, value)
            }

            trajectoryListPanel.groupConfig = value

            field = value
        }

    private var trajGenExecutor = Executors.newSingleThreadExecutor()
    private var trajGenFuture: Future<*>? = null

    init {
        trajectoryListPanel.onConfigChange = {
            trajectoryConfig = it?.config
        }
        pathEditorPanel.onConfigChange = { (startPose, startTangent, waypoints) ->
            trajectoryConfig = trajectoryConfig?.copy(startPose = startPose, startTangent = startTangent, waypoints = waypoints)
        }
        trajectoryInfoPanel.onResolutionChange = {
            trajectoryConfig = trajectoryConfig?.copy(resolution = min(MIN_RESOLUTION, max(it, MAX_RESOLUTION)))
        }
        configPanel.onConfigChange = {
            groupConfig = it
        }

        layout = BorderLayout()

        val content = JPanel()
        content.layout = BoxLayout(content, BoxLayout.PAGE_AXIS)

        val upperContent = JPanel()
        upperContent.layout = BoxLayout(upperContent, BoxLayout.LINE_AXIS)
        upperContent.add(JPanel().apply {
            add(trajectoryListPanel)
        })

        val trajContent = JPanel()
        trajContent.layout = BoxLayout(trajContent, BoxLayout.PAGE_AXIS)

        val upperRightTabbedPane = JTabbedPane()
        upperRightTabbedPane.addTab("Field", fieldPanel)
        upperRightTabbedPane.addTab("Trajectory", trajectoryGraphPanel)
        trajContent.add(upperRightTabbedPane)

        trajContent.add(trajectoryInfoPanel)

        upperContent.add(trajContent)

        content.add(upperContent)

        val lowerTabbedPane = JTabbedPane()
        lowerTabbedPane.minimumSize = Dimension(0, 0)
        lowerTabbedPane.addTab("Path", pathEditorPanel)
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

    private fun updateTrajectoryInBackground(trajectoryConfig: TrajectoryConfig, groupConfig: TrajectoryGroupConfig) {
        val c = groupConfig.constraints
        if (c.maxVel epsilonEquals 0.0 || c.maxAccel epsilonEquals 0.0 ||
            c.maxAngVel epsilonEquals 0.0 || c.maxAngAccel epsilonEquals 0.0) {
            status = "bad constraints"

            return
        }

        if (trajGenFuture?.isDone == false) {
            trajGenExecutor.shutdownNow()
            status = "interrupted"

            trajGenExecutor = Executors.newSingleThreadExecutor()
        }

        trajGenFuture = trajGenExecutor.submit {
            status = "generating trajectory..."

            if (trajectoryConfig.waypoints.isEmpty()) {
                status = "error: empty path"

                pathEditorPanel.valid = false

                return@submit
            }

            try {
                val newTrajectory = trajectoryConfig.toTrajectory(groupConfig)

                pathEditorPanel.valid = newTrajectory != null

                if (newTrajectory != null) {
                    fieldPanel.apply {
                        waypoints = listOf(trajectoryConfig.startPose.vec()) +
                            trajectoryConfig.waypoints.map { it.position }
                        robotDimensions = RobotDimensions(groupConfig.robotLength, groupConfig.robotWidth)
                        trajectory = newTrajectory
                    }
                    trajectoryInfoPanel.duration = newTrajectory.duration()
                    trajectoryGraphPanel.updateTrajectory(newTrajectory)
                }

                status = "done"
            } catch (e: EmptyPathSegmentException) {
                status = "error: empty path segment"

                pathEditorPanel.valid = false
            } catch (e: PathContinuityViolationException) {
                status = "error: invalid sequence of heading interpolators"

                pathEditorPanel.valid = false
            } catch (e: IllegalArgumentException) {
                status = "error: tank is constrained to tangent heading interpolation"

                pathEditorPanel.valid = false
            } catch (t: Throwable) {
                status = "error: ${t.javaClass}"

                t.printStackTrace()

                pathEditorPanel.valid = false
            }
        }
    }

    fun setProjectDir(dir: File) {
        trajectoryListPanel.setGroupDir(dir)
    }

    fun saveAll() {
        trajectoryListPanel.saveAll()
    }

    fun close(): Boolean {
        return if (trajectoryListPanel.dirty) {
            val result = JOptionPane.showConfirmDialog(this, "Save unsaved changes?")
            when (result) {
                JOptionPane.YES_OPTION -> {
                    saveAll()
                    true
                }
                JOptionPane.NO_OPTION -> {
                    true
                }
                else -> {
                    false
                }
            }
        } else {
            true
        }
    }
}
