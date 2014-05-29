# Contains the same fields as MsgPattern(), but as lists to choose from.
string[]                frameidPosition   # The frame to which the pattern position applies.
string[]                frameidAngle      # The frame to which the pattern angle applies.
string[]                shape             # 'constant' or 'square' or 'circle' or 'flylogo' or 'spiral' or 'ramp' or 'grid' or 'raster' or 'hilbert' or 'peano' or 'none' or 'bypoints'
float64[]               hzPattern         # Frequency of the pattern.
float64[]               hzPoint           # Frequency of points making up the pattern.
int32[]                 count             # How many times to output the pattern (-1 or N.inf means infinite).
PointList[]             points            # If shape=='bypoints', then this is the list of points to use.
geometry_msgs/Point[]   size              # (x,y) dimensions.
bool[]                  restart           # Should this pattern restart (True) an in-progress pattern, or continue (False) from the point nearest the current point.
float64[]               param             # An extra shape-dependent parameter, if needed (hilbert->level, peano->level, spiral->pitch, raster->gridpitch).
int32[]                 direction         # Step forward (+1) or reverse (-1) through the pattern points.  0 means choose at random, +1 or -1.
bool                    isDirty           # Set internally to True when the points need regenerating.


