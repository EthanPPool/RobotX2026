# Original rectified camera pixels; model adapter must undo letterboxing/resizing.
uint32 detection_id
float64 u
float64 v
float64 width
float64 height
string color
float32 confidence
# unknown, steady, flashing, inactive; unknown unless actually measured over time.
string light_state
