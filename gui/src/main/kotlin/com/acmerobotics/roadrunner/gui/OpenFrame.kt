package com.acmerobotics.roadrunner.gui

import javax.swing.*

/**
 * Main window for standalone GUI application.
 */
class OpenFrame : JFrame() {

    init {
        title = "Open Project"
        defaultCloseOperation = EXIT_ON_CLOSE
        isResizable = false

        val instructionsLabel = JLabel("Select the location of your project.")
        instructionsLabel.border = BorderFactory.createEmptyBorder(10, 10, 10, 10)

        val browseButton = JButton("Browse...")
        browseButton.addActionListener {
            val fileChooser = JFileChooser()
            fileChooser.fileSelectionMode = JFileChooser.DIRECTORIES_ONLY
            fileChooser.isMultiSelectionEnabled = false
            if (fileChooser.showDialog(this, "Open Project") == JFileChooser.APPROVE_OPTION) {
                val mainFrame = MainFrame(fileChooser.selectedFile)
                mainFrame.isVisible = true
                dispose()
            }
        }

        contentPane = JPanel()
        val layout = BoxLayout(contentPane, BoxLayout.Y_AXIS)
        contentPane.layout = layout
        contentPane.add(instructionsLabel)
        contentPane.add(browseButton)

        pack()
    }
}
