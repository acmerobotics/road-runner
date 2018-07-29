package com.acmerobotics.plugin

import com.acmerobotics.splinelib.MainPanel
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.module.ModuleManager
import com.intellij.openapi.project.Project
import java.awt.event.FocusEvent
import java.awt.event.FocusListener
import java.awt.event.KeyEvent
import java.awt.event.KeyListener
import java.io.File
import java.nio.file.Files
import javax.swing.*


class SplineDesignerPanel(private val project: Project) : JPanel() {
    private val logger = Logger.getInstance(this::class.java)
    private val moduleManager = ModuleManager.getInstance(project)

    private val moduleComboBox = JComboBox<String>()
    private val trajectoryComboBox = JComboBox<String>()

    private val nameTextField = JTextField()

    private val mainPanel = MainPanel()

    private val currentModule: String
        get() = moduleComboBox.selectedItem as String

    private val currentTrajectory: String
        get() = trajectoryComboBox.selectedItem as String

    init {
        val groupLayout = GroupLayout(this)
        layout = groupLayout

        val modules = moduleManager.modules
                .filter { it.moduleFilePath.startsWith(project.basePath!!) }
                .filter { it.name != project.name }
                .map { it.name }
                .sorted()
                .toMutableList()

        val moduleLabel = JLabel("Module", SwingConstants.RIGHT)
        moduleComboBox.model = DefaultComboBoxModel(modules.toTypedArray())
        moduleComboBox.addActionListener {
            selectModule(currentModule)
        }
        if (modules.isNotEmpty()) {
            selectModule(modules.first())
        }
        val phantomLabel = JLabel()

        val trajectoryLabel = JLabel("Trajectory", SwingConstants.RIGHT)
        trajectoryComboBox.addActionListener {
            if (currentTrajectory.isNotBlank()) {
                selectTrajectory(currentModule, currentTrajectory)
            }
        }

        val addButton = JButton("Add")
        addButton.addActionListener {
            addTrajectory()
        }

        nameTextField.addFocusListener(object : FocusListener {
            override fun focusLost(e: FocusEvent?) {
                if (nameTextField.text != currentTrajectory) {
                    renameTrajectory(currentModule, currentTrajectory, nameTextField.text)
                }
            }

            override fun focusGained(e: FocusEvent?) {
                // do nothing
            }

        })
        nameTextField.addKeyListener(object : KeyListener {
            override fun keyTyped(e: KeyEvent?) {
                // do nothing
            }

            override fun keyPressed(e: KeyEvent?) {
                // do nothing
            }

            override fun keyReleased(e: KeyEvent?) {
                if (e?.keyCode == KeyEvent.VK_ENTER) {
                    if (nameTextField.text != currentTrajectory) {
                        renameTrajectory(currentModule, currentTrajectory, nameTextField.text)
                    }
                }
            }
        })

        val saveButton = JButton("Save")
        saveButton.addActionListener {
            saveTrajectory(currentModule, currentTrajectory)
        }

        val deleteButton = JButton("Delete")
        deleteButton.addActionListener {
            deleteTrajectory(currentModule, currentTrajectory)
        }

        groupLayout.autoCreateGaps = true
        groupLayout.autoCreateContainerGaps = true

        groupLayout.setVerticalGroup(groupLayout.createSequentialGroup()
                .addGroup(groupLayout.createParallelGroup(GroupLayout.Alignment.CENTER)
                        .addComponent(moduleLabel)
                        .addComponent(moduleComboBox)
                        .addComponent(phantomLabel))
                .addGroup(groupLayout.createParallelGroup(GroupLayout.Alignment.CENTER)
                        .addComponent(trajectoryLabel)
                        .addComponent(trajectoryComboBox)
                        .addComponent(addButton))
                .addGroup(groupLayout.createParallelGroup(GroupLayout.Alignment.CENTER)
                        .addComponent(nameTextField)
                        .addComponent(saveButton)
                        .addComponent(deleteButton))
                .addComponent(mainPanel))

        groupLayout.setHorizontalGroup(groupLayout.createParallelGroup(GroupLayout.Alignment.CENTER)
                .addGroup(groupLayout.createSequentialGroup()
                        .addComponent(moduleLabel)
                        .addComponent(moduleComboBox)
                        .addComponent(phantomLabel))
                .addGroup(groupLayout.createSequentialGroup()
                        .addComponent(trajectoryLabel)
                        .addComponent(trajectoryComboBox)
                        .addComponent(addButton))
                .addGroup(groupLayout.createSequentialGroup()
                        .addComponent(nameTextField)
                        .addComponent(saveButton)
                        .addComponent(deleteButton))
                .addComponent(mainPanel))

        groupLayout.linkSize(moduleLabel, trajectoryLabel)
        groupLayout.linkSize(moduleComboBox, trajectoryComboBox, nameTextField)
        groupLayout.linkSize(phantomLabel, addButton)
    }

    private fun saveTrajectory(module:String, trajectory: String) {
        val dir = getTrajectoryAssetsDir(module) ?: return
        Files.createDirectories(dir.toPath())
        mainPanel.save(File(dir, "$trajectory.yaml"))
    }

    private fun nextUntitledName(): String {
        val trajectories = listTrajectoryAssets(currentModule).toMutableList()
        val prefix = "untitled"
        var name = prefix
        var i = 1
        while (true) {
            if (name !in trajectories) {
                return name
            }
            i++
            name = "$prefix$i"
        }
    }

    private fun addTrajectory(nameArg: String? = null) {
        // TODO: should we save the old one?
        // we'll go ahead and overwrite if necessary
        val trajectories = listTrajectoryAssets(currentModule).toMutableList()
        val name = nameArg ?: nextUntitledName()
        trajectories.add(name)
        trajectories.sort()
        mainPanel.clearTrajectory()
        saveTrajectory(currentModule, name)
        trajectoryComboBox.model = DefaultComboBoxModel(trajectories.toTypedArray())
        trajectoryComboBox.selectedItem = name
        selectTrajectory(currentModule, name)
    }

    private fun deleteTrajectory(module: String, trajectory: String) {
        val trajectoryFile = File(getTrajectoryAssetsDir(module), "$trajectory.yaml")
        trajectoryFile.delete()
        val trajectories = listTrajectoryAssets(module).filter { it != trajectory }
        trajectoryComboBox.model = DefaultComboBoxModel(trajectories.toTypedArray())
        if (trajectories.isNotEmpty()) {
            selectTrajectory(module, currentTrajectory)
        } else {
            clearSelectedTrajectory()
        }
    }

    private fun renameTrajectory(module: String, oldTrajectory: String, newTrajectory: String) {
        saveTrajectory(module, oldTrajectory)
        val assetsDir = getTrajectoryAssetsDir(module)
        val oldTrajectoryFile = File(assetsDir, "$oldTrajectory.yaml")
        val newTrajectoryFile = File(assetsDir, "$newTrajectory.yaml")
        oldTrajectoryFile.copyTo(newTrajectoryFile)
        oldTrajectoryFile.delete()
        val trajectories = listTrajectoryAssets(module).map { if (it == oldTrajectory) newTrajectory else it }
        trajectoryComboBox.model = DefaultComboBoxModel(trajectories.toTypedArray())
        trajectoryComboBox.selectedItem = newTrajectory
        selectTrajectory(module, newTrajectory)
    }

    private fun selectModule(module: String) {
        val trajectories = listTrajectoryAssets(module)
        trajectoryComboBox.model = DefaultComboBoxModel(trajectories.toTypedArray())
        if (trajectories.isNotEmpty()) {
            selectTrajectory(module, trajectories.first())
        } else {
            clearSelectedTrajectory()
        }
    }

    private fun selectTrajectory(module: String, trajectory: String) {
        mainPanel.load(File(getTrajectoryAssetsDir(module), "$trajectory.yaml"))
        nameTextField.text = trajectory
    }

    private fun listTrajectoryAssets(moduleString: String): List<String> {
        val trajectoryAssetsDir = getTrajectoryAssetsDir(moduleString) ?: return listOf()
        if (trajectoryAssetsDir.exists()) {
            return trajectoryAssetsDir
                    .listFiles { _, name -> name.endsWith(".yaml") }
                    .map { it.nameWithoutExtension }
                    .sorted()
        }
        return listOf()
    }

    private fun getTrajectoryAssetsDir(moduleString: String): File? {
        val module = moduleManager.modules.firstOrNull { it.name == moduleString } ?: return null
        val moduleFile = File(module.moduleFilePath)
        return File(moduleFile.parent, "src/main/assets/trajectory")
    }

    private fun clearSelectedTrajectory() {
        nameTextField.text = ""
        mainPanel.updateTrajectory(listOf(), MainPanel.DEFAULT_CONSTRAINTS)
    }
}