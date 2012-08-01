bool 		enabled
string 		frameidParent 	# 'Plate', 'Robot', 'Fly1', 'Fly2' etc
string 		frameidChild 	
float64 	speedParentMin  # Absolute speed of parent frame.
float64 	speedParentMax
float64 	speedChildMin   # Absolute speed of child frame.
float64 	speedChildMax
float64 	distanceMin     # Distance from parent to child.
float64 	distanceMax
float64 	angleMin		# Angle to child in parent frame.
float64 	angleMax
string  	angleTest		# 'inclusive' or 'exclusive'
bool    	angleTestBilateral
float64 	timeHold
float64 	timeout


