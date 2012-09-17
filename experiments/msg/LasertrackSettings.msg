bool 						enabled
patterngen/MsgPattern[] 	pattern_list
string[]                    statefilterHi_list   		# These lists should either be empty, or contain as many entries as patterns.
string[]                    statefilterLo_list	 		# Contains strings to be interpreted as dicts.  Pattern will only be enabled for fly states between these two entries.
string[]                    statefilterCriteria_list	# "inclusive" or "exclusive"
#ZoneSettings                exclusionzone
float64 					timeout

