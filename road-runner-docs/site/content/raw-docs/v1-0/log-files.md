# Log Files

The quickstart logs data from every run to a file for later debugging. The most
recent logs that haven't been deleted are accessible at
`http://192.168.43.1:8080/logs`.

Each log consists of a sequence of messages stored in a binary format. Each
message belongs to a channel (e.g., `TARGET_POSE`), and all messages on a given
channel adhere to a fixed schema. (This should be familiar to
anyone who's worked with a typical robotics message-passing system.)

Users can add additional types by mimicking the [quickstart message
definitions](https://github.com/acmerobotics/road-runner-quickstart/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/messages).
The code to turn classes into schemas can be found
[here](https://github.com/acmerobotics/road-runner-ftc/blob/c9f0be75158276c5dfcd82ccabb639a15a200f98/RoadRunner/src/main/java/com/acmerobotics/roadrunner/ftc/LogFile.kt#L154-L179).

A basic Python parser can be found
[here](https://gist.github.com/rbrott/e243b8c4190e655d41a988232cb102d2).
