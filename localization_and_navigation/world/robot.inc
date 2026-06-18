define kinect ranger
(
  sensor
  (
    pose [ 0.0 0.0 0.5 0.0 ]
    size [0.1 0.1 0.1 ]
    range  [0.2 6.5]
    fov 360.0
    samples 720
  )
  # generic model properties
  color "black"
  size [ 0.15 0.15 0.15 ]
)

define robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"


  odom_error [ 0.02 0.02 0.0 0.05 ]

  size [ 0.512 0.48 0.245 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "grey"
  
  kinect()
)
