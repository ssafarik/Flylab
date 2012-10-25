bool                    enabled
string                  command                 # 'None', 'trackposition' (panel position follows fly position), 'trackorientation' (panel position follows fly angle). 
int32                   idPattern               # The pattern number to show.
#geometry_msgs/Point     origin
string                  frame_id                # Which frame to track.
string                  statefilterHi   		
string                  statefilterLo	 		# Contains a string to be interpreted as a dict.  Operation will only be enabled for fly states between these two entries.
string                  statefilterCriteria	    # "inclusive" or "exclusive"
float64                 timeout

