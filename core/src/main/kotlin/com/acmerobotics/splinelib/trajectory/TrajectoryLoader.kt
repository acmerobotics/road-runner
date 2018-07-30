package com.acmerobotics.splinelib.trajectory

import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.registerKotlinModule
import java.io.File

object TrajectoryLoader {
    private val MAPPER = ObjectMapper(YAMLFactory()).registerKotlinModule()

    @JvmStatic
    fun saveConfig(trajectoryConfig: TrajectoryConfig, file: File) {
        MAPPER.writerWithDefaultPrettyPrinter().writeValue(file, trajectoryConfig)
    }

    @JvmStatic
    fun loadConfig(file: File) = MAPPER.readValue(file, TrajectoryConfig::class.java)

    @JvmStatic
    fun load(file: File) = loadConfig(file).toTrajectory()
}