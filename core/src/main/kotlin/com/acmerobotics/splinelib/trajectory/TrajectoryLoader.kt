package com.acmerobotics.splinelib.trajectory

import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.registerKotlinModule
import java.io.File

object TrajectoryLoader {
    private val mapper = ObjectMapper(YAMLFactory()).registerKotlinModule()

    fun saveConfig(trajectoryConfig: TrajectoryConfig, file: File) {
        mapper.writerWithDefaultPrettyPrinter().writeValue(file, trajectoryConfig)
    }

    fun loadConfig(file: File) = mapper.readValue(file, TrajectoryConfig::class.java)
}