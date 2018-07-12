package com.acmerobotics.splinelib

import com.acmerobotics.splinelib.trajectory.Trajectory
import java.awt.Dimension
import javax.swing.BoxLayout
import javax.swing.JFrame
import javax.swing.JPanel

class MainFrame : JFrame() {
    init {
        title = "Spline Designer"
        size = Dimension(650, 1000)
        defaultCloseOperation = JFrame.EXIT_ON_CLOSE

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
        contentPane = JPanel()
        contentPane.add(panel)
    }
}