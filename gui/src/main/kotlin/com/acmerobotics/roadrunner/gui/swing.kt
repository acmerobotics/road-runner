package com.acmerobotics.roadrunner.gui

import com.acmerobotics.roadrunner.geometry.Vector2d
import java.awt.geom.Ellipse2D
import java.awt.geom.Path2D
import java.awt.geom.Point2D
import java.text.DecimalFormat
import javax.swing.DefaultListModel
import javax.swing.JComboBox
import javax.swing.JFormattedTextField
import javax.swing.JTextField
import javax.swing.event.DocumentEvent
import javax.swing.event.DocumentListener
import javax.swing.text.NumberFormatter

fun JTextField.addChangeListener(listener: () -> Unit) {
    document.addDocumentListener(object : DocumentListener {
        override fun changedUpdate(e: DocumentEvent?) = listener()
        override fun insertUpdate(e: DocumentEvent?) = listener()
        override fun removeUpdate(e: DocumentEvent?) = listener()
    })
}

@Suppress("UNCHECKED_CAST")
val <E> JComboBox<E>.selectedItemTyped: E?
    get() = selectedItem as E?

fun makeFormattedDoubleField() = JFormattedTextField(NumberFormatter(DecimalFormat("0.00")))

@Suppress("UNCHECKED_CAST")
fun <T> JFormattedTextField.getTypedValue() = value as T?

val JFormattedTextField.doubleValue
    get() = getTypedValue<Number>()?.toDouble()

inline fun <reified T> JFormattedTextField.addValueChangeListener(
    crossinline listener: (T) -> Unit
) {
    addPropertyChangeListener {
        val newValue = it.newValue
        if (newValue is T) {
            listener(newValue)
        }
    }
}

fun Vector2d.awt() = Point2D.Double(x, y)

fun Path2D.Double.moveTo(point: Point2D.Double) {
    moveTo(point.x, point.y)
}

fun Path2D.Double.lineTo(point: Point2D.Double) {
    lineTo(point.x, point.y)
}

fun circle(center: Vector2d, radius: Double) = Ellipse2D.Double(center.x - radius / 2, center.y - radius / 2, radius, radius)

@Suppress("UNCHECKED_CAST")
fun <E> DefaultListModel<E>.asList() = this.toArray().asList() as List<E>
