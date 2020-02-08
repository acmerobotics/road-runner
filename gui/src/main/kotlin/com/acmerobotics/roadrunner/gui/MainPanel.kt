package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.EmptyPathException
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.epsilonEquals
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
private const val DEFAULT_ROBOT_SIZE = 18.0
private const val DEFAULT_RESOLUTION = 0.25
private const val MIN_RESOLUTION = 5.0
private const val MAX_RESOLUTION = 0.01
private val DEFAULT_TRAJECTORY_CONFIG = TrajectoryConfig(
    Pose2d(),
    null,
    emptyList(),
    DEFAULT_RESOLUTION
)

/**
 * Main GUI window panel.
 */
class MainPanel : JPanel() {

    data class DiskTrajectoryConfig(
        val config: TrajectoryConfig,
        val file: File,
        val dirty: Boolean = false
    ) {
        override fun toString() = "${file.nameWithoutExtension}${if (dirty) "*" else ""}"
    }

    private val trajListModel = DefaultListModel<DiskTrajectoryConfig>()
    private val dirty: Boolean
        get() = trajListModel.toArray().any { (it as DiskTrajectoryConfig).dirty }

    var onTrajectoryUpdateListener: (() -> Unit)? = null

    private val trajList = JList(trajListModel)
    private val trajTextField = JTextField()
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

    private var trajectoryConfig: TrajectoryConfig = DEFAULT_TRAJECTORY_CONFIG
        set(value) {
            pathStepPanel.update(value.startPose, value.startHeading, value.steps)
            trajectoryInfoPanel.updateResolution(value.resolution)
            field = value
        }

    private var groupConfig: TrajectoryGroupConfig = TrajectoryGroupConfig(
        DEFAULT_CONSTRAINTS,
        DEFAULT_ROBOT_SIZE,
        DEFAULT_ROBOT_SIZE,
        TrajectoryGroupConfig.DriveType.GENERIC,
        null,
        null,
        null
    )
        set(value) {
            configPanel.update(value)
            field = value
        }

    private var trajGenExecutor = Executors.newSingleThreadExecutor()
    private var trajGenFuture: Future<*>? = null

    private var updating = false

    private var projectDir: File? = null

    init {
        trajList.addListSelectionListener {
            if (trajList.selectedValue == null) {
                return@addListSelectionListener
            }

            updating = true

            trajTextField.text = trajList.selectedValue.file.nameWithoutExtension

            trajectoryConfig = trajList.selectedValue.config

            updateTrajectoryInBackground()

            updating = false
        }

        pathStepPanel.onUpdateListener = { startPose, startHeading, steps ->
            trajectoryConfig.startPose = startPose
            trajectoryConfig.startHeading = startHeading
            trajectoryConfig.steps = steps

            markCurrentTrajDirty()

            updateTrajectoryInBackground()
        }
        trajectoryInfoPanel.onResolutionUpdateListener = {
            trajectoryConfig.resolution = min(MIN_RESOLUTION, max(it, MAX_RESOLUTION))

            markCurrentTrajDirty()

            updateTrajectoryInBackground()
        }
        configPanel.onUpdateListener = {
            groupConfig.constraints = it.constraints
            groupConfig.driveType = it.driveType
            groupConfig.lateralMultiplier = it.lateralMultiplier
            groupConfig.robotLength = it.robotLength
            groupConfig.robotWidth = it.robotWidth
            groupConfig.trackWidth = it.trackWidth
            groupConfig.wheelBase = it.wheelBase

            markCurrentTrajDirty()

            updateTrajectoryInBackground()
        }

        layout = BorderLayout()

        val content = JPanel()
        content.layout = BoxLayout(content, BoxLayout.PAGE_AXIS)

        val upperContent = JPanel()
        upperContent.layout = BoxLayout(upperContent, BoxLayout.LINE_AXIS)

        val upperLeftContent = JPanel()
        upperLeftContent.layout = BoxLayout(upperLeftContent, BoxLayout.PAGE_AXIS)

        trajList.border = BorderFactory.createEmptyBorder(30, 30, 30, 30)
        trajList.background = upperContent.background

        trajTextField.maximumSize = Dimension(150, trajTextField.preferredSize.height)
        trajTextField.addChangeListener {
            markCurrentTrajDirty()
        }
        upperLeftContent.add(trajTextField)

        val buttonPanel = JPanel()
        buttonPanel.layout = BoxLayout(buttonPanel, BoxLayout.LINE_AXIS)
        val saveButton = JButton("Save")
        saveButton.addActionListener {
            if (trajTextField.text != trajList.selectedValue.file.nameWithoutExtension) {
                rename(trajList.selectedIndex, trajTextField.text)
            } else {
                save(trajList.selectedIndex)
            }
        }

        val removeButton = JButton("Remove")
        removeButton.addActionListener {
            delete(trajList.selectedIndex)
        }

        val addButton = JButton("Add")
        addButton.addActionListener {
            add()
        }

        buttonPanel.add(saveButton)
        buttonPanel.add(removeButton)

        upperLeftContent.add(buttonPanel)

        upperLeftContent.add(addButton)

        upperLeftContent.add(trajList)

        upperContent.add(upperLeftContent)

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

    private fun markCurrentTrajDirty() {
        if (trajList.selectedIndex != -1 && !updating) {
            trajListModel.set(trajList.selectedIndex, trajList.selectedValue.copy(dirty = true))
        }
    }

    fun setProjectDir(dir: File): Boolean {
        val fileList = dir.listFiles { file ->
            file.extension == "yaml" && !file.name.startsWith("_")
        } ?: return false
        if (fileList.isEmpty()) {
            return false
        }
        if (!trajListModel.isEmpty && dirty) {
            val result = JOptionPane.showConfirmDialog(this, "Discard unsaved changes?")
            if (result != JOptionPane.OK_OPTION) {
                return false
            }
        }
        val newGroupConfig = TrajectoryConfigManager.loadGroupConfig(dir)
        if (newGroupConfig != null) {
            groupConfig = newGroupConfig
        }
        for (file in fileList) {
            trajListModel.addElement(DiskTrajectoryConfig(
                TrajectoryConfigManager.loadConfig(file) ?: return false,
                file
            ))
        }
        trajList.selectedIndex = 0
        projectDir = dir
        return true
    }

    private fun updateTrajectoryInBackground() {
        val c = groupConfig.constraints
        if (c.maxVel epsilonEquals 0.0 || c.maxAccel epsilonEquals 0.0 ||
            c.maxAngVel epsilonEquals 0.0 || c.maxAngAccel epsilonEquals 0.0) {
            return
        }

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
                    fieldPanel.updateTrajectoryAndConfig(trajectory, trajectoryConfig, groupConfig)
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

    fun delete(trajIndex: Int) {
        val diskTrajectoryConfig = trajListModel[trajIndex]
        diskTrajectoryConfig.file.delete()
        trajListModel.remove(trajIndex)
        if (!trajListModel.isEmpty && trajListModel.size() == trajIndex) {
            trajList.selectedIndex = trajIndex - 1
        } else {
            trajList.selectedIndex = trajIndex
        }
    }

    fun rename(trajIndex: Int, newName: String) {
        val diskTraj = trajListModel[trajIndex]
        val newFile = File(diskTraj.file.parent, "$newName.${diskTraj.file.extension}")
        diskTraj.file.delete()
        trajListModel.set(trajIndex, diskTraj.copy(file = newFile))
        save(trajIndex)
    }

    fun save(trajIndex: Int, saveGroupConfig: Boolean = true) {
        val diskTrajectoryConfig = trajListModel[trajIndex]
        if (diskTrajectoryConfig.dirty) {
            TrajectoryConfigManager.saveConfig(diskTrajectoryConfig.config, diskTrajectoryConfig.file)
            trajListModel.set(trajIndex, diskTrajectoryConfig.copy(dirty = false))
        }
        if (saveGroupConfig) {
            TrajectoryConfigManager.saveGroupConfig(groupConfig, diskTrajectoryConfig.file.parentFile)
        }
    }

    fun add() {
        val trajectories = trajListModel.toArray()
            .map { it as DiskTrajectoryConfig }
            .map { it.file.nameWithoutExtension }
        val prefix = "untitled"
        var name = prefix
        var i = 1
        while (true) {
            if (name !in trajectories) {
                break
            }
            i++
            name = "$prefix$i"
        }

        val config = DiskTrajectoryConfig(
            DEFAULT_TRAJECTORY_CONFIG.copy(),
            File(projectDir ?: return, "$name.yaml"),
            true
        )

        trajListModel.addElement(config)
        trajList.setSelectedValue(config, true)
    }

    fun load(file: File) {
        val trajectoryConfig = TrajectoryConfigManager.loadConfig(file) ?: return
        val trajectoryGroupConfig = TrajectoryConfigManager.loadGroupConfig(file) ?: return

        pathStepPanel.update(trajectoryConfig.startPose, trajectoryConfig.startHeading, trajectoryConfig.steps)
        configPanel.update(trajectoryGroupConfig)
        trajectoryInfoPanel.updateResolution(trajectoryConfig.resolution)

        updateTrajectoryInBackground()
    }

    fun saveAll() {
        for (i in 0 until trajListModel.size()) {
            save(i, i == 0)
        }
    }

    fun close(): Boolean {
//        JOptionPane.showMessageDialog(this, "Are you sure you want to close?")
        return true
    }
}
