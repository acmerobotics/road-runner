package com.acmerobotics.roadrunner.util


/**
 * Various Logging utilities.
 */
object Log {

    @JvmStatic
    fun dbgPrint(level: Int) {
        println("roadrunner")
        for (i in 1..level) {
            val trace = Exception().stackTrace[i]
            println(trace)
            }
        //printLineNumber()
        //Throwable.printStackTrace();
    }
}
