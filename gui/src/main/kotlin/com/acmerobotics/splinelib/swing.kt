package com.acmerobotics.splinelib

import javax.swing.JTextField
import javax.swing.event.DocumentEvent
import javax.swing.event.DocumentListener

fun JTextField.addChangeListener(listener: () -> Unit) {
    document.addDocumentListener(object : DocumentListener {
        override fun changedUpdate(e: DocumentEvent?) = listener()
        override fun insertUpdate(e: DocumentEvent?) = listener()
        override fun removeUpdate(e: DocumentEvent?) = listener()
    })
}