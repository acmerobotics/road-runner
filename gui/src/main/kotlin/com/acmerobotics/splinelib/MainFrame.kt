package com.acmerobotics.splinelib

import java.awt.Dimension
import javax.swing.BoxLayout
import javax.swing.JFrame
import javax.swing.JPanel

class MainFrame : JFrame() {
    init {
        title = "Spline Designer"
        size = Dimension(650, 1000)
        defaultCloseOperation = JFrame.EXIT_ON_CLOSE

        val editorPanel = PoseEditor()
        val fieldPanel = FieldPanel()

        editorPanel.onUpdateListener = fieldPanel::updatePoses

        val panel = JPanel()
        val panel2 = JPanel()
        val layout = BoxLayout(panel, BoxLayout.PAGE_AXIS)
        panel.layout = layout
        panel2.add(editorPanel)
        panel.add(fieldPanel)
        panel.add(panel2)
        contentPane = JPanel()
        contentPane.add(panel)
    }
}