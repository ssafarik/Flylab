string                mode     # 'byshape' or 'bypoints'
string                shape    # 'square' or 'circle' or 'flylogo' or 'spiral'
geometry_msgs/Point[] points   # If mode=='bypoints', then this is the list of points to scan.
float32               hz       # Pattern frequency.
int32                 count    # How many times to output the pattern (-1 or N.inf means infinite).
float32               radius   # Center to max distance.
bool				  preempt  # Should this message restart an in-progress pattern.

