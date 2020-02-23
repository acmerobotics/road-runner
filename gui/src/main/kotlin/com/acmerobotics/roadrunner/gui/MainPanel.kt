package com.acmerobotics.roadrunner.gui

import DEFAULT_GROUP_CONFIG
import DEFAULT_TRAJECTORY_CONFIG
import com.acmerobotics.roadrunner.path.EmptyPathException
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import com.acmerobotics.roadrunner.util.epsilonEquals
import java.awt.BorderLayout
import java.awt.Dimension
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

    data class DiskTrajectoryConfig(
        val config: TrajectoryConfig,
        val file: File,
        val dirty: Boolean = false
    ) {
        override fun toString() = "${file.nameWithoutExtension}${if (dirty) "*" else ""}"
    }

    private val trajListModel = DefaultListModel<DiskTrajectoryConfig>()
    private val dirty: Boolean
        get() = trajListModel.asList().any { it.dirty }

    var onTrajectoryUpdate: (() -> Unit)? = null

    private val trajList = JList(trajListModel)
    private val trajTextField = JTextField()
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

    private var trajectoryConfig: TrajectoryConfig = DEFAULT_TRAJECTORY_CONFIG
        set(value) {
            pathEditorPanel.config = PathConfig(value.startPose, value.startHeading, value.steps)
            trajectoryInfoPanel.resolution = value.resolution

            val i = trajList.selectedIndex
            trajListModel.setElementAt(trajListModel[i].copy(config = value), i)

            field = value
        }

    private var groupConfig: TrajectoryGroupConfig = DEFAULT_GROUP_CONFIG
        set(value) {
            configPanel.config = value
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

        pathEditorPanel.onConfigUpdate = { (startPose, startHeading, steps) ->
            trajectoryConfig = trajectoryConfig.copy(startPose = startPose, startHeading = startHeading, steps = steps)

            markCurrentTrajDirty()

            updateTrajectoryInBackground()
        }
        trajectoryInfoPanel.onResolutionUpdate = {
            trajectoryConfig = trajectoryConfig.copy(resolution = min(MIN_RESOLUTION, max(it, MAX_RESOLUTION)))

            markCurrentTrajDirty()

            updateTrajectoryInBackground()
        }
        configPanel.onConfigUpdate = {
            groupConfig = it

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
            if (trajList.selectedIndex == -1) {
                return@addActionListener
            }
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

    private fun markCurrentTrajDirty() {
        if (trajList.selectedIndex != -1 && !updating) {
            trajListModel.set(trajList.selectedIndex, trajList.selectedValue.copy(dirty = true))
        }
    }

    fun setProjectDir(dir: File): Boolean {
        dir.mkdirs()
        val fileList = dir.listFiles { file ->
            file.extension == "yaml" && !file.name.startsWith("_")
        } ?: return false
        if (!trajListModel.isEmpty && dirty) {
            val result = JOptionPane.showConfirmDialog(this, "Save unsaved changes?")
            when (result) {
                JOptionPane.YES_OPTION -> {
                    saveAll()
                }
                else -> {
                    return false
                }
            }
        }
        val newGroupConfig = TrajectoryConfigManager.loadGroupConfig(dir)
        if (newGroupConfig != null) {
            groupConfig = newGroupConfig
        }
        trajListModel.clear()
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

            try {
                val trajectory = trajectoryConfig.toTrajectory(groupConfig)

                pathEditorPanel.valid = trajectory != null

                if (trajectory != null) {
                    fieldPanel.updateTrajectoryAndConfig(trajectory, trajectoryConfig, groupConfig)
                    trajectoryInfoPanel.duration = trajectory.duration()
                    trajectoryGraphPanel.updateTrajectory(trajectory)

                    onTrajectoryUpdate?.invoke()
                }

                status = "done"
            } catch (e: EmptyPathException) {
                status = "error: empty path"

                pathEditorPanel.valid = false
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

    fun delete(trajIndex: Int) {
        if (trajIndex == -1) {
            return
        }

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
        if (trajIndex == -1) {
            return
        }

        val diskTraj = trajListModel[trajIndex]
        val newFile = File(diskTraj.file.parent, "$newName.${diskTraj.file.extension}")
        diskTraj.file.delete()
        trajListModel.set(trajIndex, diskTraj.copy(file = newFile))
        save(trajIndex)
    }

    fun save(trajIndex: Int, saveGroupConfig: Boolean = true) {
        if (trajIndex == -1) {
            return
        }

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
        val trajectories = trajListModel.asList().map { it.file.nameWithoutExtension }
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

    fun saveAll() {
        for (i in 0 until trajListModel.size()) {
            save(i, i == 0)
        }
    }

    fun close(): Boolean {
        return if (dirty) {
            val result = JOptionPane.showConfirmDialog(this, "Save unsaved changes??")
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
