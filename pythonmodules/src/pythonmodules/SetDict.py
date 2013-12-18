# Set(d, defaults, bKeep)
# Takes a target dictionary, and enters values from the source dictionary, overwriting or not, as asked.
# For example,
#    dT={'a':1, 'b':2}
#    dS={'a':0, 'c':0}
#    Set(dT, dS, True)
#    dT is {'a':1, 'b':2, 'c':0}
#
#    dT={'a':1, 'b':2}
#    dS={'a':0, 'c':0}
#    Set(dT, dS, False)
#    dT is {'a':0, 'b':2, 'c':0}
#
def Set(dTarget, dSource, bPreserve):
    for k,v in dSource.iteritems():
        bKeyExists = (k in dTarget)
        if (not bKeyExists) and type(v)==type({}):
            dTarget[k] = {}
        if ((not bKeyExists) or not bPreserve) and (type(v)!=type({})):
            dTarget[k] = v
                
        if type(v)==type({}):
            Set(dTarget[k], v, bPreserve)
    



def SetWithPreserve(dTarget, dSource):
    Set(dTarget, dSource, True)

def SetWithOverwrite(dTarget, dSource):
    Set(dTarget, dSource, False)

