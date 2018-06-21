package com.acmerobotics.plugin

import com.intellij.openapi.project.Project
import com.intellij.openapi.wm.ToolWindow
import com.intellij.openapi.wm.ToolWindowFactory
import com.intellij.ui.content.ContentFactory
import com.intellij.ui.layout.panel
import javax.swing.JCheckBox
import javax.swing.JTextField


class TestToolWindowFactory : ToolWindowFactory {
    override fun createToolWindowContent(project: Project, toolWindow: ToolWindow) {
//        val panel = panel {
//            PathPanel()
//        }
        val contentFactory = ContentFactory.SERVICE.getInstance()
        val content = contentFactory.createContent(PathPanel(), "", false)
        toolWindow.contentManager.addContent(content)
    }
}