package com.acmerobotics.splinelib

import java.awt.GridLayout
import javax.swing.*


class PoseEditorPanel : JPanel() {
    private class MutablePose2d(var x: Double, var y: Double, var heading: Double) {
        constructor(pose: Pose2d) : this(pose.x, pose.y, pose.heading)

        fun immutable() = Pose2d(x, y, heading)
    }

    var onPosesUpdateListener: ((List<Pose2d>) -> Unit)? = null
    private val poses = mutableListOf<MutablePose2d>()

    init {
        layout = GridLayout(0, 4, 5, 0)

        add(JLabel("X", SwingConstants.CENTER))
        add(JLabel("Y", SwingConstants.CENTER))
        add(JLabel("Heading", SwingConstants.CENTER))
        val addButton = JButton("Add")
        addButton.addActionListener { addPose(poses.lastOrNull()?.immutable() ?: Pose2d(0.0, 0.0, 0.0)) }
        add(addButton)

        addPose(Pose2d(0.0, 0.0, 0.0))
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
            add(comp)
        }

        poses.add(mutablePose)

        removeButton.addActionListener {
            for (comp in uiComponents) {
                remove(comp)
            }
            poses.remove(mutablePose)

            revalidate()

            fireUpdate()
        }

        revalidate()

        fireUpdate()
    }
}