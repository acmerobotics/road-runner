package com.acmerobotics.splinelib

import java.awt.Dimension
import javax.swing.JFrame

class MainFrame : JFrame() {
    init {
        title = "Spline Designer"
        size = Dimension(500, 500)
        defaultCloseOperation = JFrame.EXIT_ON_CLOSE

        contentPane = PathPanel()
    }
}