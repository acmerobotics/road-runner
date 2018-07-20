package com.acmerobotics.plugin

import com.acmerobotics.splinelib.MainPanel
import com.intellij.openapi.project.Project
import com.intellij.openapi.wm.ToolWindow
import com.intellij.openapi.wm.ToolWindowFactory


class SplineDesignerToolWindowFactory : ToolWindowFactory {
    override fun createToolWindowContent(project: Project, toolWindow: ToolWindow) {
        val content = toolWindow.contentManager.factory.createContent(MainPanel(), "", true)
        toolWindow.contentManager.addContent(content)
    }
}