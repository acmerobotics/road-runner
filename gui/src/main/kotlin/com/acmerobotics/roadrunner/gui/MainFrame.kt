package com.acmerobotics.roadrunner.gui

import java.awt.BorderLayout
import java.awt.Dimension
import java.awt.Toolkit
import java.awt.event.KeyEvent
import java.awt.event.WindowEvent
import java.awt.event.WindowListener
import java.io.File
import javax.swing.*
import kotlin.system.exitProcess

private val COMMAND_MASK = Toolkit.getDefaultToolkit().menuShortcutKeyMask

/**
 * Main window for standalone GUI application.
 */
class MainFrame(
    dir: File
) : JFrame() {
    init {
        title = "Path Designer"
        size = Dimension(1000, 800)
        defaultCloseOperation = DO_NOTHING_ON_CLOSE
        isResizable = false

        val mainPanel = MainPanel()
        mainPanel.setProjectDir(dir)

        addWindowListener(object : WindowListener {
            override fun windowDeiconified(e: WindowEvent?) {

            }

            override fun windowClosing(e: WindowEvent?) {
                if (mainPanel.close()) {
                    exitProcess(0)
                }
            }

            override fun windowClosed(e: WindowEvent?) {

            }

            override fun windowActivated(e: WindowEvent?) {

            }

            override fun windowDeactivated(e: WindowEvent?) {

            }

            override fun windowOpened(e: WindowEvent?) {

            }

            override fun windowIconified(e: WindowEvent?) {

            }
        })

        contentPane = JPanel()
        contentPane.layout = BorderLayout()
        contentPane.add(mainPanel)

        val fileMenu = JMenu("File")

        val saveMenuItem = JMenuItem("Save All")
        saveMenuItem.addActionListener {
            mainPanel.saveAll()
        }
        saveMenuItem.accelerator = KeyStroke.getKeyStroke(KeyEvent.VK_S, COMMAND_MASK)
        fileMenu.add(saveMenuItem)

        val closeMenuItem = JMenuItem("Close")
        closeMenuItem.addActionListener {
            if (mainPanel.close()) {
                exitProcess(0)
            }
        }

        val menuBar = JMenuBar()
        menuBar.add(fileMenu)

        jMenuBar = menuBar
    }
}
