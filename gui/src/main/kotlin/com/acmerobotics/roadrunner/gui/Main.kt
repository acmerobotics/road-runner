package com.acmerobotics.roadrunner.gui

import javax.swing.SwingUtilities
import javax.swing.UIManager

/**
 * Entrypoint for the GUI.
 */
object Main {
    @JvmStatic
    fun main(args: Array<String>) {
        SwingUtilities.invokeLater {
            System.setProperty("apple.laf.useScreenMenuBar", "true")
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName())

            OpenFrame().isVisible = true
        }
    }
}
