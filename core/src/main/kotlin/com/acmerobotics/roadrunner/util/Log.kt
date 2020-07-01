package com.acmerobotics.roadrunner.util


/**
 * Various Logging utilities.
 */
object Log {
    val enableLogging: Boolean = true
    @JvmStatic
    fun dbgPrint(level: Int) {
        if (enableLogging)
            println("roadrunner")
        for (i in 1..level) {
            val trace = Exception().stackTrace[i]
            if (enableLogging)
                println(trace)
        }
        //printLineNumber()
        //Throwable.printStackTrace();
    }
    fun dbgPrint(s: String) {
        //println("$s");
    }
}
