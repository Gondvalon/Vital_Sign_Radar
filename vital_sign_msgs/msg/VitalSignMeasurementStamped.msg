std_msgs/Header header
# Where the vital sign is located
geometry_msgs/Point location
# Heart rate estimate in beats per minute
float32 heart_rate
# Breathing rate estimate in breaths per minute
float32 breathing_rate
# Variance. Set to NaN if not available
float32 heart_rate_variance
float32 breathing_rate_variance
