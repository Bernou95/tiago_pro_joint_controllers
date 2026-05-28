# Generic joint command message for TiagoPro joint controllers.
#
# Used by effort_joint_controller, position_joint_controller, and
# velocity_joint_controller. The semantic meaning of each entry in `command`
# is determined by which controller's ~/commands topic this is published on:
#   effort_joint_controller  -> N*m  (torques)
#   position_joint_controller -> rad  (angles)
#   velocity_joint_controller -> rad/s (angular velocities)
#
# joint_names: explicitly names each joint so the sender can control the
#   ordering. If empty the controller assumes the standard TiagoPro joint order
#   (joint1 … joint7) with the configured arm_prefix and robot_type.
#
# command: one value per entry in joint_names (or 7 values if joint_names is
#   empty). Values outside the configured joint limits are rejected.

std_msgs/Header header
string[] joint_names
float64[] command
