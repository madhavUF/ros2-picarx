# Assistant state for UI/feedback
std_msgs/Header header
string state           # idle, listening, thinking, speaking, acting
string current_action  # Description of what assistant is doing
float32 action_progress  # 0.0 to 1.0 for progress indication
string last_utterance  # Last thing user said
string last_response   # Last thing assistant said
