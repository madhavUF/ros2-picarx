# Processed voice command with intent
std_msgs/Header header
string transcription   # Raw transcription
float32 confidence     # STT confidence
string intent          # Classified intent: move, look, describe, question, command, other
string[] entities      # Extracted entities (object names, directions, etc.)
