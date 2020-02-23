package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.trajectory.Trajectory
import javax.swing.SwingUtilities
import javax.swing.UIManager

/**
 * Entrypoint for the GUI.
 */
object TrajectoryGUI {
    @JvmStatic
    fun showTrajectory(trajectory: Trajectory) {
        SwingUtilities.invokeLater {
            System.setProperty("apple.laf.useScreenMenuBar", "true")
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName())

            TrajectoryFrame(trajectory).isVisible = true
        }
    }
}
