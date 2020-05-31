package com.acmerobotics.roadrunner.plugin

import com.acmerobotics.roadrunner.gui.MainPanel
import com.intellij.openapi.module.ModuleManager
import com.intellij.openapi.project.Project
import java.awt.Dimension
import java.io.File
import javax.swing.*

/**
 * Main window for IntelliJ plugin.
 */
class PluginPanel(private val project: Project) : JPanel() {
    private val moduleManager = ModuleManager.getInstance(project)

    private val moduleComboBox = JComboBox<String>()

    private val mainPanel = MainPanel()

    private var currentModule: String? = null

    private val comboBoxModule: String?
        get() = moduleComboBox.selectedItem as String?

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
        val prefSize = moduleComboBox.preferredSize
        moduleComboBox.maximumSize = Dimension(3 * prefSize.width, prefSize.height)
        moduleComboBox.model = DefaultComboBoxModel(modules.toTypedArray())
        moduleComboBox.addActionListener {
            if (currentModule != comboBoxModule) {
                updateModuleSelection()
            }
        }
        updateModuleSelection()
        val phantomLabel = JLabel()

        groupLayout.autoCreateGaps = true
        groupLayout.autoCreateContainerGaps = true

        groupLayout.setVerticalGroup(groupLayout.createSequentialGroup()
                .addGroup(groupLayout.createParallelGroup(GroupLayout.Alignment.CENTER)
                        .addComponent(moduleLabel)
                        .addComponent(moduleComboBox)
                        .addComponent(phantomLabel))
                .addComponent(mainPanel))

        groupLayout.setHorizontalGroup(groupLayout.createParallelGroup(GroupLayout.Alignment.CENTER)
                .addGroup(groupLayout.createSequentialGroup()
                        .addComponent(moduleLabel)
                        .addComponent(moduleComboBox)
                        .addComponent(phantomLabel))
                .addComponent(mainPanel))
    }

    private fun updateModuleSelection() {
        if (currentModule == comboBoxModule) return
        val comboBoxModule = comboBoxModule ?: return
        if (mainPanel.setProjectDir(getTrajectoryAssetsDir(comboBoxModule) ?: return)) {
            currentModule = comboBoxModule
        } else {
            moduleComboBox.selectedItem = currentModule
        }
    }

    private fun getTrajectoryAssetsDir(moduleString: String): File? {
        val module = moduleManager.modules.firstOrNull { it.name == moduleString } ?: return null
        val moduleFile = File(module.moduleFilePath)
        val path = File(moduleFile.parent, "src/main/assets/trajectory").absolutePath
        val trajDir = File(path.replace(".idea/modules/", ""))
        trajDir.mkdirs()
        return trajDir
    }
}
