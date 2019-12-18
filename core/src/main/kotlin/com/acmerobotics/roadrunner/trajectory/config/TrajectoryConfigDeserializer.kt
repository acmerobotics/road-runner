package com.acmerobotics.roadrunner.trajectory.config

import com.fasterxml.jackson.core.JsonParser
import com.fasterxml.jackson.databind.DeserializationContext
import com.fasterxml.jackson.databind.JsonNode
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.databind.deser.std.StdDeserializer
import com.fasterxml.jackson.module.kotlin.registerKotlinModule

class TrajectoryConfigDeserializer : StdDeserializer<TrajectoryConfig>(TrajectoryConfig::class.java) {
    // TODO: this isn't a great solution
    private val mapper = ObjectMapper().registerKotlinModule()

    override fun deserialize(p: JsonParser?, ctxt: DeserializationContext?): TrajectoryConfig? {
        p ?: return null
        val node = p.readValueAsTree<JsonNode>()
        return if (node.has("version")) {
            val version = node["version"].intValue()
            if (version == 2) {
                mapper.treeToValue(node, TrajectoryConfig::class.java)
            } else {
                null
            }
        } else {
            // legacy V1
            p.codec.treeToValue(node, LegacyTrajectoryConfig::class.java).toTrajectoryConfig()
        }
    }
}
