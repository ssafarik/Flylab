bool 						enabled
patterngen/MsgPattern[] 	pattern_list
string[]                    stateFilterLo_list	# Contains strings to be interpreted as dicts.  Pattern will only be enabled for fly states between these two entries.
string[]                    stateFilterHi_list	# These lists should either be empty, or contain as many entries as patterns.
float64 					timeout

