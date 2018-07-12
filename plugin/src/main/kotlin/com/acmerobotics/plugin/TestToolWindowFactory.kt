package com.acmerobotics.plugin

import com.acmerobotics.splinelib.EditorPanel
import com.acmerobotics.splinelib.FieldPanel
import com.acmerobotics.splinelib.trajectory.Trajectory
import com.intellij.openapi.project.Project
import com.intellij.openapi.wm.ToolWindow
import com.intellij.openapi.wm.ToolWindowFactory
import javax.swing.BoxLayout
import javax.swing.JPanel


class TestToolWindowFactory : ToolWindowFactory {
    override fun createToolWindowContent(project: Project, toolWindow: ToolWindow) {
        val editorPanel = EditorPanel()
        val fieldPanel = FieldPanel()

        editorPanel.trajectoryListener = {
            trajectory: Trajectory ->
            fieldPanel.trajectory = trajectory
            fieldPanel.repaint()
        }

        val panel = JPanel()
        val panel2 = JPanel()
        val layout = BoxLayout(panel, BoxLayout.PAGE_AXIS)
        panel.layout = layout
        panel2.add(editorPanel)
        panel.add(fieldPanel)
        panel.add(panel2)

        val content = toolWindow.contentManager.factory.createContent(panel, "", true)
        toolWindow.contentManager.addContent(content)
    }
}