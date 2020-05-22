package com.acmerobotics.roadrunner.gui

import DEFAULT_GROUP_CONFIG
import DEFAULT_TRAJECTORY_CONFIG
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import java.awt.Component
import java.awt.Dimension
import java.io.File
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
    var onConfigChange: ((DiskTrajectoryConfig?) -> Unit)? = null

    private var externalUpdate = false
    private var updating = false
    private var _diskConfig: DiskTrajectoryConfig? = null
        set(value) {
            if (updating) return

            updating = true

            val i = trajList.selectedIndex

            if (value != null) {
                trajListModel.setElementAt(value, i)
            }

            field = value

            if (!externalUpdate) {
                onConfigChange?.invoke(value)
            }

            updating = false
        }

    var diskConfig
        get() = _diskConfig
        set(value) {
            externalUpdate = true

            _diskConfig = value

            externalUpdate = false
        }

    private val trajListModel = DefaultListModel<DiskTrajectoryConfig>()
    val dirty: Boolean
        get() = trajListModel.asList().any { it.dirty } || groupConfigDirty

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
            // this thins out duplicate events
            if (it.valueIsAdjusting) {
                return@addListSelectionListener
            }

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

            _diskConfig = selectedConfig
        }

        layout = BoxLayout(this, BoxLayout.PAGE_AXIS).apply {
            alignmentY = Component.TOP_ALIGNMENT
        }

        trajList.border = BorderFactory.createEmptyBorder(30, 30, 30, 30)
        trajList.background = background

        trajTextField.maximumSize = Dimension(150, trajTextField.preferredSize.height)
        trajTextField.addChangeListener {
            val diskConfig = trajList.selectedValue ?: return@addChangeListener
            _diskConfig = diskConfig.copy(
                dirty = true
            )
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

    private fun delete(trajIndex: Int) {
        val diskTrajectoryConfig = trajListModel[trajIndex]
        diskTrajectoryConfig.file.delete()
        trajListModel.remove(trajIndex)
    }

    private fun rename(trajIndex: Int, newName: String) {
        val diskTraj = trajListModel[trajIndex]
        val newFile = File(diskTraj.file.parent, "$newName.yaml")
        diskTraj.file.delete()
        trajListModel.set(trajIndex, diskTraj.copy(
            file = newFile
        ))
        save(trajIndex)
    }

    private fun save(trajIndex: Int) {
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

    private fun add() {
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
