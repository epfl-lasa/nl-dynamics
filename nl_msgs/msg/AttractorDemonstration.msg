# Demonstration associated with a natural language input. The 'demonstration' is
# a trajectory that is the desired correction of the behavior. The 'anchor' is
# an offset, representing where the corrections should begin. The words are also
# stored here for convenience.

Header header
CartStateStamped anchor
uint32 num_points
Correction[] trajectory
uint32 num_words
string[] words
