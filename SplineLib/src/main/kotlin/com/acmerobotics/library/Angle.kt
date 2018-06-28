package com.acmerobotics.library

object Angle {
    const val TAU = Math.PI * 2

    fun norm(angle: Double): Double {
        var modifiedAngle = angle % TAU

        modifiedAngle = (modifiedAngle + TAU) % TAU

        if (modifiedAngle > Math.PI) {
            modifiedAngle -= TAU
        }

        return modifiedAngle
    }

}