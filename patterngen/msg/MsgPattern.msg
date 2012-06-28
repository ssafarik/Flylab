string                mode      # 'byshape' or 'bypoints'
string                shape     # 'square' or 'circle' or 'flylogo' or 'spiral'
string                frame     # The frame in which to output the points.
float64               hzPattern # Frequency of the pattern.
float64               hzPoint   # Frequency of points making up the pattern.
int32                 count     # How many times to output the pattern (-1 or N.inf means infinite).
geometry_msgs/Point[] points    # If mode=='bypoints', then this is the list of points to scan.
float64               radius    # Center to max distance.
bool				  preempt   # Should this message restart an in-progress pattern.

