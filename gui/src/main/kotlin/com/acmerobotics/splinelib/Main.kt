package com.acmerobotics.splinelib

import javax.swing.SwingUtilities
import javax.swing.UIManager

object Main {
    @JvmStatic
    fun main(args: Array<String>) {
        SwingUtilities.invokeLater {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName())

            val mainFrame = MainFrame()
            mainFrame.isVisible = true
        }
    }
}
