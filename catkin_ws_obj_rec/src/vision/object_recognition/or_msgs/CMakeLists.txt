cmake_minimum_required(VERSION 2.8.6)
project(or_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs)

add_message_files(FILES 
BoundingBox2D.msg
ExtractKeyPoints.msg
KeyPointMatchArray.msg
KeyPointMatch.msg
KeyPoint.msg
MatchResult.msg
OrCommand.msg
OrImage.msg
OrLearnCommand.msg
OrLearningStatus.msg
OrMatchResult.msg
OrObjectNames.msg
Point2D.msg
VectorObject2D.msg
	)


generate_messages(DEPENDENCIES sensor_msgs)
#generate_services(DEPENDENCIES)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs roscpp_serialization sensor_msgs
)

