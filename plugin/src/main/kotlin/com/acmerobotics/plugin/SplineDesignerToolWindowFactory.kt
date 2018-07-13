package com.acmerobotics.plugin

import com.acmerobotics.splinelib.FieldPanel
import com.acmerobotics.splinelib.PoseEditor
import com.intellij.openapi.project.Project
import com.intellij.openapi.wm.ToolWindow
import com.intellij.openapi.wm.ToolWindowFactory
import javax.swing.BoxLayout
import javax.swing.JPanel


class SplineDesignerToolWindowFactory : ToolWindowFactory {
    override fun createToolWindowContent(project: Project, toolWindow: ToolWindow) {
        val poseEditor = PoseEditor()
        val fieldPanel = FieldPanel()

        poseEditor.onUpdateListener = fieldPanel::updatePoses

        val panel = JPanel()
        val panel2 = JPanel()
        val layout = BoxLayout(panel, BoxLayout.PAGE_AXIS)
        panel.layout = layout
        panel2.add(poseEditor)
        panel.add(fieldPanel)
        panel.add(panel2)

        val content = toolWindow.contentManager.factory.createContent(panel, "", true)
        toolWindow.contentManager.addContent(content)
    }
}