# This message is published from the AV
# it provides the location and description of a detected object

# class of detection
uint8 id

# UTM coordinates of detected object
float64 easting
float64 northing
uint8 zone

# Bounding box rectangle size
uint8 bb_height
uint8 bb_width

# Confidence of detection
float64 obj_confidence 