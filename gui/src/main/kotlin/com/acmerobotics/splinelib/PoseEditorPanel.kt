package com.acmerobotics.splinelib

import java.awt.Dimension
import java.awt.GridLayout
import javax.swing.*
import kotlin.math.min


class PoseEditorPanel : JPanel() {
    override fun getPreferredSize(): Dimension {
        val size = super.getPreferredSize()

        return Dimension(size.width, min(150, size.height))
    }

    private class MutablePose2d(var x: Double, var y: Double, var heading: Double) {
        constructor(pose: Pose2d) : this(pose.x, pose.y, pose.heading)

        fun immutable() = Pose2d(x, y, heading)
    }

    private val headerPanel = JPanel()
    private val scrollPanel = JPanel()

    var onPosesUpdateListener: ((List<Pose2d>) -> Unit)? = null
    private val poses = mutableListOf<MutablePose2d>()

    init {
        headerPanel.layout = GridLayout(1, 4, 5, 0)
        scrollPanel.layout = GridLayout(0, 4, 5, 0)

        headerPanel.add(JLabel("X", SwingConstants.CENTER))
        headerPanel.add(JLabel("Y", SwingConstants.CENTER))
        headerPanel.add(JLabel("Heading", SwingConstants.CENTER))
        val addButton = JButton("Add")
        addButton.addActionListener { addPose(poses.lastOrNull()?.immutable() ?: Pose2d(0.0, 0.0, 0.0)) }
        headerPanel.add(addButton)

        addPose(Pose2d(0.0, 0.0, 0.0))

        layout = BoxLayout(this, BoxLayout.PAGE_AXIS)

        val scrollPane = JScrollPane(ScrollPanelHost(scrollPanel))
        scrollPane.border = BorderFactory.createEmptyBorder()

        add(headerPanel)
        add(scrollPane)
    }

    fun fireUpdate() {
        onPosesUpdateListener?.invoke(poses.map(MutablePose2d::immutable))
    }

    fun makeNumField(initialVal: Double): JTextField {
        val numField = JTextField(initialVal.toString())
        numField.horizontalAlignment = SwingConstants.CENTER
        return numField
    }

    fun addPose(pose: Pose2d) {
        val mutablePose = MutablePose2d(pose)

        val xField = makeNumField(pose.x)
        val yField = makeNumField(pose.y)
        val headingField = makeNumField(pose.heading.toDegrees())

        xField.addChangeListener {
            mutablePose.x = xField.text.toDoubleOrNull() ?: mutablePose.x
            fireUpdate()
        }
        yField.addChangeListener {
            mutablePose.y = yField.text.toDoubleOrNull() ?: mutablePose.y
            fireUpdate()
        }
        headingField.addChangeListener {
            mutablePose.heading = headingField.text.toDoubleOrNull()?.toRadians() ?: mutablePose.heading
            fireUpdate()
        }

        val removeButton = JButton("Remove")

        val uiComponents = listOf<JComponent>(xField, yField, headingField, removeButton)
        for (comp in uiComponents) {
            scrollPanel.add(comp)
        }

        poses.add(mutablePose)

        removeButton.addActionListener {
            for (comp in uiComponents) {
                scrollPanel.remove(comp)
            }
            poses.remove(mutablePose)

            revalidate()

            fireUpdate()
        }

        revalidate()

        fireUpdate()
    }
}