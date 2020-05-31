import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig

const val DEFAULT_ROBOT_SIZE = 18.0
const val DEFAULT_TRACK_WIDTH = 18.0
const val DEFAULT_RESOLUTION = 0.25
val DEFAULT_TRAJECTORY_CONFIG = TrajectoryConfig(
    Pose2d(),
    0.0,
    emptyList(),
    DEFAULT_RESOLUTION
)
val DEFAULT_GROUP_CONFIG = TrajectoryGroupConfig(
    30.0,
    40.0,
    Math.PI,
    Math.PI,
    DEFAULT_ROBOT_SIZE,
    DEFAULT_ROBOT_SIZE,
    TrajectoryGroupConfig.DriveType.GENERIC,
    null,
    null,
    null
)

val DEFAULT_WAYPOINT = TrajectoryConfig.Waypoint(
    Vector2d(),
    0.0,
    0.0,
    TrajectoryConfig.HeadingInterpolationType.TANGENT
)
