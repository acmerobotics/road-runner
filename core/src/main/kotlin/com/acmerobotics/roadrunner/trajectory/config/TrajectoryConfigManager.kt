package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.databind.module.SimpleModule
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory
import com.fasterxml.jackson.module.kotlin.registerKotlinModule
import java.io.File

/**
 * Class containing methods for saving (loading) trajectory configurations to (from) YAML files.
 */
object TrajectoryConfigManager {
    @JvmStatic
    val GROUP_FILENAME = "group.yaml"

    private val MAPPER = ObjectMapper(YAMLFactory())

    init {
        val module = SimpleModule()
        module.addDeserializer(TrajectoryConfig::class.java, TrajectoryConfigDeserializer())
        MAPPER.registerModule(module)
        MAPPER.registerKotlinModule()
    }

    /**
     * Saves a [TrajectoryConfigV1] to [file].
     */
    @JvmStatic
    fun saveConfig(trajectoryConfig: TrajectoryConfigV1, file: File) {
        MAPPER.writerWithDefaultPrettyPrinter().writeValue(file, trajectoryConfig)
    }

    /**
     * Loads a [TrajectoryConfig] from [file].
     */
    @JvmStatic
    fun loadConfig(file: File) = MAPPER.readValue(file, TrajectoryConfig::class.java)

    /**
     * Loads a [Trajectory] from [file]. In newer trajectory config formats, the constraints and other important data is
     * stored separately (for the purpose of sharing with other trajectories). This method recursively examines parent
     * directories until the requisite files are found.
     */
    @JvmStatic
    fun loadBuilder(file: File): TrajectoryBuilder? {
        val config = loadConfig(file) ?: return null
        if (config is TrajectoryConfigV1) {
            return config.toTrajectoryBuilder()
        } else if (config is TrajectoryConfigV2) {
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
            val groupConfig = MAPPER.readValue(groupFile, TrajectoryGroupConfig::class.java)
            return config.toTrajectoryBuilder(groupConfig)
        }
        return null
    }

    /**
     * Convenience wrapper around [loadBuilder].
     */
    @JvmStatic
    fun load(file: File) = loadBuilder(file)?.build()
}
