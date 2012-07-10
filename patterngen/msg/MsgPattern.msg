string                mode      # 'byshape' or 'bypoints'
string                shape     # 'constant' or 'square' or 'circle' or 'flylogo' or 'spiral' or 'ramp' or 'grid' or 'raster' or 'hilbert' or 'peano' or 'none'
string                frame_id  # The frame to which the pattern applies.
float64               hzPattern # Frequency of the pattern.
float64               hzPoint   # Frequency of points making up the pattern.
int32                 count     # How many times to output the pattern (-1 or N.inf means infinite).
geometry_msgs/Point[] points    # If mode=='bypoints', then this is the list of points to scan.
geometry_msgs/Point   size      # (x,y) dimensions.
bool				  preempt   # Should this message restart an in-progress pattern.
float64               param     # An extra shape-dependent parameter, if needed (hilbert->level, peano->level, spiral->pitch, raster->gridpitch).
 

