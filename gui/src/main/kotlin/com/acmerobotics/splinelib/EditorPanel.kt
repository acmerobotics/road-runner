package com.acmerobotics.splinelib

import com.acmerobotics.splinelib.trajectory.DriveConstraints
import com.acmerobotics.splinelib.trajectory.Trajectory
import com.acmerobotics.splinelib.trajectory.TrajectoryBuilder
import java.awt.GridLayout
import java.awt.event.FocusEvent
import java.awt.event.FocusListener
import java.text.DecimalFormat
import javax.swing.*


class EditorPanel : JPanel() {
    enum class Type {
        START, POINT_TURN, LINE, SPLINE
    }

    data class Entry(var type: Type = Type.SPLINE, var x: Double = 0.0, var y: Double = 0.0, var heading: Double = 0.0, var reversed: Boolean = false)

    companion object {
        val NUM_FORMAT = DecimalFormat("##0.000")
    }

    val entries = mutableListOf<Entry>()
    var trajectoryListener: ((Trajectory) -> Unit)? = null

    init {
        layout = GridLayout(0, 5, 5, 0)

        add(JLabel("Type", SwingConstants.CENTER))
        add(JLabel("X", SwingConstants.CENTER))
        add(JLabel("Y", SwingConstants.CENTER))
        add(JLabel("Heading", SwingConstants.CENTER))
        add(JLabel("Reversed", SwingConstants.CENTER))

        addStartEntry()
        addGenericEntry()
    }

    fun makeNumField(initialVal: Double): JTextField {
        val numField = JTextField(initialVal.toString())  // JFormattedTextField(NUM_FORMAT)
        numField.horizontalAlignment = SwingConstants.CENTER
//        numField.value = initialVal
        numField.addBlurListener { trajectoryListener?.invoke(buildTrajectory()) }
        return numField
    }

    fun addStartEntry() {
        val xField = makeNumField(0.0)
        val yField = makeNumField(0.0)
        val headingField = makeNumField(0.0)

        val entry = Entry(Type.START)

        xField.addBlurListener { entry.x = xField.text.toDouble() }
        yField.addBlurListener { entry.y = yField.text.toDouble() }
        headingField.addBlurListener { entry.heading = headingField.text.toDouble() }

        entries.add(entry)

        add(JLabel("Start", SwingConstants.CENTER))
        add(xField)
        add(yField)
        add(headingField)
        add(JLabel())
    }

    fun addGenericEntry() {
        val typeCombobox = JComboBox(arrayOf("Spline", "Line", "Point Turn"))
        val xField = makeNumField(0.0)
        val yField = makeNumField(0.0)
        val headingField = makeNumField(0.0)
        val reversedCheckbox = JCheckBox()
        reversedCheckbox.horizontalAlignment = SwingConstants.CENTER

        val entry = Entry(Type.SPLINE)

        typeCombobox.addActionListener {
            when (typeCombobox.selectedIndex) {
                0 -> {
                    entry.type = Type.SPLINE
                    xField.isVisible = true
                    yField.isVisible = true
                    headingField.isVisible = true
                }
                1 -> {
                    entry.type = Type.LINE
                    xField.isVisible = true
                    yField.isVisible = true
                    headingField.isVisible = false
                }
                2 -> {
                    entry.type = Type.POINT_TURN
                    xField.isVisible = false
                    yField.isVisible = false
                    headingField.isVisible = true
                }
            }
        }

        xField.addBlurListener { entry.x = xField.text.toDouble() }
        yField.addBlurListener { entry.y = yField.text.toDouble() }
        headingField.addBlurListener { entry.heading = headingField.text.toDouble() }
        reversedCheckbox.addActionListener { entry.reversed = reversedCheckbox.isSelected }

        entries.add(entry)

        add(typeCombobox)
        add(xField)
        add(yField)
        add(headingField)
        add(reversedCheckbox)
    }

    fun buildTrajectory(): Trajectory {
        val start = System.currentTimeMillis()
        val startPose = Pose2d(entries[0].x, entries[0].y, Angle.norm(Math.toRadians(entries[0].heading)))
        val builder = TrajectoryBuilder(startPose, DriveConstraints(10.0, 10.0, 2.0, 2.0, 50.0))
        for (entry in entries.drop(1)) {
            builder.setReversed(entry.reversed)
            when (entry.type) {
                Type.SPLINE -> builder.splineTo(Pose2d(entry.x, entry.y, Angle.norm(Math.toRadians(entry.heading))))
                Type.LINE -> builder.lineTo(Vector2d(entry.x, entry.y))
                Type.POINT_TURN -> builder.turnTo(Angle.norm(Math.toRadians(entry.heading)))
                else -> RuntimeException("Hmmmmmm")
            }
        }
        val trajectory = builder.build()
        val elapsed = System.currentTimeMillis() - start
        println("built trajectory in $elapsed ms")
        println(trajectory)
        return trajectory
    }
}

fun JTextField.addBlurListener(listener: () -> Unit) {
    addFocusListener(object : FocusListener {
        override fun focusLost(e: FocusEvent?) {
            listener.invoke()
        }

        override fun focusGained(e: FocusEvent?) {
            listener.invoke()
        }
    })
}