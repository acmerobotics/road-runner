package com.acmerobotics.roadrunner.gui

import DEFAULT_GROUP_CONFIG
import DEFAULT_TRAJECTORY_CONFIG
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import java.awt.Component
import java.awt.Dimension
import java.io.File
import java.lang.RuntimeException
import javax.swing.*
import javax.swing.event.ListDataEvent
import javax.swing.event.ListDataListener


data class DiskTrajectoryConfig(
    val config: TrajectoryConfig,
    val file: File,
    val dirty: Boolean = false
) {
    val name = file.nameWithoutExtension

    override fun toString() = "${name}${if (dirty) "*" else ""}"
}


class TrajectoryListPanel : JPanel() {
    var onConfigChange: (() -> Unit)? = null

    private var _trajectoryConfig: TrajectoryConfig? = null
    var trajectoryConfig
        get() = _trajectoryConfig
        set(value) {
            val i = trajList.selectedIndex
            if ((i == -1) != (value == null)) {
                throw RuntimeException()
            }

            if (value != null) {
                trajListModel.setElementAt(trajListModel[i].copy(
                    config = value,
                    dirty = true
                ), i)
            }

            _trajectoryConfig = value
        }

    private val trajListModel = DefaultListModel<DiskTrajectoryConfig>()
    val dirty: Boolean
        get() = trajListModel.asList().any { it.dirty }

    private val trajList: JList<DiskTrajectoryConfig?> = JList(trajListModel)
    private val trajTextField = JTextField()

    private var groupConfigDirty = false
    private var _groupConfig: TrajectoryGroupConfig = DEFAULT_GROUP_CONFIG
    var groupConfig
        get() = _groupConfig
        set(value) {
            groupConfigDirty = true

            _groupConfig = value
        }

    private var groupDir: File? = null

    init {
        trajTextField.isEnabled = false

        val removeButton = JButton("Remove")
        removeButton.addActionListener {
            delete(trajList.selectedIndex)
        }
        removeButton.isEnabled = false

        val saveButton = JButton("Save")
        saveButton.addActionListener {
            val diskTraj = trajList.selectedValue ?: return@addActionListener
            if (trajTextField.text != diskTraj.file.nameWithoutExtension) {
                rename(trajList.selectedIndex, trajTextField.text)
            } else {
                save(trajList.selectedIndex)
            }
        }
        saveButton.isEnabled = false

        trajList.selectionMode = ListSelectionModel.SINGLE_SELECTION

        trajList.addListSelectionListener {
            val selectedConfig = trajList.selectedValue
            if (selectedConfig == null) {
                trajTextField.text = ""
                trajTextField.isEnabled = false
                removeButton.isEnabled = false
                saveButton.isEnabled = false
            } else {
                trajTextField.text = selectedConfig.name
                trajTextField.isEnabled = true
                removeButton.isEnabled = true
                saveButton.isEnabled = true
            }

            _trajectoryConfig = selectedConfig?.config

            onConfigChange?.invoke()
        }

        layout = BoxLayout(this, BoxLayout.PAGE_AXIS).apply {
            alignmentY = Component.TOP_ALIGNMENT
        }

        trajList.border = BorderFactory.createEmptyBorder(30, 30, 30, 30)
        trajList.background = background

        trajTextField.maximumSize = Dimension(150, trajTextField.preferredSize.height)
        trajTextField.addChangeListener {
            val diskTraj = trajList.selectedValue ?: return@addChangeListener
            trajListModel.set(trajList.selectedIndex, diskTraj.copy(
                dirty = true
            ))
        }
        add(trajTextField)

        val buttonPanel = JPanel()
        buttonPanel.layout = BoxLayout(buttonPanel, BoxLayout.LINE_AXIS)

        val addButton = JButton("Add")
        addButton.addActionListener {
            add()
        }

        buttonPanel.add(saveButton)
        buttonPanel.add(removeButton)

        add(buttonPanel)

        add(addButton)

        add(trajList)
    }

    fun setGroupDir(dir: File): Boolean {
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
                    // TODO: are there separate cancel and no responses?
                    return false
                }
            }
        }
        val newGroupConfig = TrajectoryConfigManager.loadGroupConfig(dir)
        if (newGroupConfig != null) {
            _groupConfig = newGroupConfig
            groupConfigDirty = false
        } else {
            groupConfigDirty = true
        }

        trajListModel.clear()
        for (file in fileList) {
            trajListModel.addElement(DiskTrajectoryConfig(
                TrajectoryConfigManager.loadConfig(file) ?: return false,
                file
            ))
        }

        groupDir = dir
        return true
    }

    fun delete(trajIndex: Int) {
        val diskTrajectoryConfig = trajListModel[trajIndex]
        diskTrajectoryConfig.file.delete()
        trajListModel.remove(trajIndex)
    }

    fun rename(trajIndex: Int, newName: String) {
        val diskTraj = trajListModel[trajIndex]
        val newFile = File(diskTraj.file.parent, "$newName.yaml")
        diskTraj.file.delete()
        trajListModel.set(trajIndex, diskTraj.copy(
            file = newFile
        ))
        save(trajIndex)
    }

    fun save(trajIndex: Int) {
        val diskTrajectoryConfig = trajListModel[trajIndex]
        if (diskTrajectoryConfig.dirty) {
            TrajectoryConfigManager.saveConfig(diskTrajectoryConfig.config, diskTrajectoryConfig.file)
            trajListModel.set(trajIndex, diskTrajectoryConfig.copy(
                dirty = false
            ))
        }
        if (groupConfigDirty) {
            TrajectoryConfigManager.saveGroupConfig(groupConfig, diskTrajectoryConfig.file.parentFile)
            groupConfigDirty = false
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
            File(groupDir ?: return, "$name.yaml"),
            true
        )

        trajListModel.addElement(config)
        trajList.setSelectedValue(config, true)
    }

    fun saveAll() {
        for (i in 0 until trajListModel.size()) {
            save(i)
        }
    }
}
