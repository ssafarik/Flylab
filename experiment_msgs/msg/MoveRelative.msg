bool 		tracking
string 		frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'
string 		frameidOriginAngle    # 'Plate' or 'Robot' or 'Fly'
float64 	distance              # mm
string 		angleType             # 'random' or 'constant'
float64 	angleOffset           # Radians from origin to target.
float64     angleOscMag           # Radian magnitude of the added oscillation.
float64     angleOscFreq          # Hz of the added oscillation.
string 		speedType             # 'random' or 'constant'
float64 	speed                 # mm/sec
float64 	tolerance             # mm

