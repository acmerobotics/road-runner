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
        val userField = JTextField()
        val passwordField = JTextField()
        val rememberCheckBox = JCheckBox()
        val panel = panel {
            noteRow("Login to get notified when the submitted\nexceptions are fixed.")
            row("Username:") { userField() }
            row("Password:") { passwordField() }
            row {
                rememberCheckBox()
                right {
                    link("Forgot password?") { /* custom action */ }
                }
            }
            noteRow("""Do not have an account? <a href="https://account.jetbrains.com/login">Sign Up</a>""")
        }
        val contentFactory = ContentFactory.SERVICE.getInstance()
        val content = contentFactory.createContent(panel, "", false)
        toolWindow.contentManager.addContent(content)
    }
}