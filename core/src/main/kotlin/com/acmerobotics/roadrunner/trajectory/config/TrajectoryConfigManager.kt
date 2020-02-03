package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.fasterxml.jackson.annotation.JsonInclude
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.databind.module.SimpleModule
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.registerKotlinModule
import java.io.File
import java.io.InputStream

/**
 * Class containing methods for saving (loading) trajectory configurations to (from) YAML files.
 */
object TrajectoryConfigManager {
    @JvmStatic
    val GROUP_FILENAME = "_group.yaml"

    private val MAPPER = ObjectMapper(YAMLFactory())

    init {
        val module = SimpleModule()
        module.addDeserializer(TrajectoryConfig::class.java, TrajectoryConfigDeserializer())
        MAPPER.registerModule(module)
        MAPPER.registerKotlinModule()
        MAPPER.setSerializationInclusion(JsonInclude.Include.NON_NULL)
    }

    /**
     * Saves a [LegacyTrajectoryConfig] to [file].
     */
    @JvmStatic
    fun saveConfig(trajectoryConfig: LegacyTrajectoryConfig, file: File) {
        MAPPER.writerWithDefaultPrettyPrinter().writeValue(file, trajectoryConfig)
    }

    /**
     * Saves a [TrajectoryConfig] to [file].
     */
    @JvmStatic
    fun saveConfig(trajectoryConfig: TrajectoryConfig, file: File) {
        MAPPER.writerWithDefaultPrettyPrinter().writeValue(file, trajectoryConfig)
    }

    /**
     * Saves a [TrajectoryGroupConfig] to [dir].
     */
    @JvmStatic
    fun saveGroupConfig(trajectoryConfig: TrajectoryGroupConfig, dir: File) {
        MAPPER.writerWithDefaultPrettyPrinter().writeValue(File(dir, GROUP_FILENAME), trajectoryConfig)
    }

    /**
     * Loads a [TrajectoryConfig] from [file].
     */
    @JvmStatic
    fun loadConfig(file: File): TrajectoryConfig? = MAPPER.readValue(file, TrajectoryConfig::class.java)

    /**
     * Loads a [TrajectoryConfig] from [inputStream].
     */
    @JvmStatic
    fun loadConfig(inputStream: InputStream): TrajectoryConfig? =
        MAPPER.readValue(inputStream, TrajectoryConfig::class.java)

    /**
     * Loads the [TrajectoryGroupConfig] corresponding to the [TrajectoryConfig] file [file]. This method recursively
     * examines parent directories until the group config file is found.
     */
    @JvmStatic
    fun loadGroupConfig(file: File): TrajectoryGroupConfig? {
        var dir = file.parentFile
        var groupFile: File? = null
        while (dir != null) {
            val groupFileCand = File(dir, GROUP_FILENAME)
            if (groupFileCand.exists()) {
                groupFile = groupFileCand
                break
            }
            dir = dir.parentFile
        }
        return MAPPER.readValue(groupFile, TrajectoryGroupConfig::class.java)
    }

    /**
     * Loads a [Trajectory] from [file].
     */
    @JvmStatic
    fun loadBuilder(file: File): TrajectoryBuilder? {
        val config = loadConfig(file) ?: return null
        return config.toTrajectoryBuilder(loadGroupConfig(file) ?: return null)
    }

    /**
     * Convenience wrapper around [loadBuilder].
     */
    @JvmStatic
    fun load(file: File) = loadBuilder(file)?.build()
}
