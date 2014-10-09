bool[]      tracking
string[]    frameidOrigin
float64[]   distance              # mm
float64[]   angleOffset           # Angular offset to target point in given frame (radians).
float64[]   angleVelocity         # Angular velocity of target point in given frame (radians per sec), i.e. 2pi*frequency.
float64[]   angleOscMag           # Oscillatory addition to angular velocity (radians).
float64[]   angleOscFreq          # Hz of the added oscillation.
float64[]   speedMax              # Maximum allowed speed of travel (mm/sec).
float64[]   tolerance             # mm
string[]    typeAngleOffset       # 'constant' or 'random' (on range [0,2pi]), or 'current' (use the current angle of the robot in the origin frame)
string[]    typeAngleVelocity     # 'constant' or 'random' (on range [0,angleVelocity])
string[]    typeAngleOscMag       # 'constant' or 'random' (on range [0,angleOscMag])
string[]    typeAngleOscFreq      # 'constant' or 'random' (on range [0,angleOscFreq])
string[]    typeSpeed             # 'constant' or 'random' (on range [0,speedMax])

# angle =          a0 +           a1*t +          a2*sin(2*pi*f*t)
# angle = angleOffset + anglePerTime*t + angleOscMag*sin(2*pi*angleOscFreq*t)
