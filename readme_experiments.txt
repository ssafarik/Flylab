==================================
Documentation for experiment .py files.
a.k.a.
How to get Flylab to run your experiment.
==================================
Flylab can be configured to run a variety of experiments, and this is done primarily via settings in an experiment.py file.  These files are found in Flylab/experiments/nodes/*.py, and for a new experiment you should copy one of the existing files to a new name of your choice, then edit the new file.


The key to understanding the layout of the experimentparams is that it derives from the experiment flowchart (called a state machine in ROS).
Below you see the flowchart (crudely) as text.

                                +------------------------------------------------------------------------------------------------------------------------------------+
                                |                                                                                                                                    |
                                |                                                                                                                                    |
                                v                                                                                                                                    |
+---------------+    +-------------+    +----------+    +----------------+    +----------------+    +----------------+    +------------------+    +--------+    +--------+    +-------------+
|StartExperiment|--->|ResetHardware|--->|StartTrial|--->|    PreWait1    |--->|   PreTrigger   |--->|    PreWait2    |--->|    PostTrigger   |--->|PostWait|--->|EndTrial|--->|EndExperiment|
+---------------+    +-------------+    +----------+    +----------------+    +----------------+    +----------------+    +------------------+    +--------+    +--------+    +-------------+
                                                        |.pre.robot      |    |.pre.robot      |    |.pre.robot      |    |.trial.robot      |
                                                        +----------------+    +----------------+    +----------------+    +------------------+
                                                        |.pre.ledpanels  |    |.pre.ledpanels  |    |.pre.ledpanels  |    |.trial.ledpanels  |
                                                        +----------------+    +----------------+    +----------------+    +------------------+
                                                        |.pre.lasergalvos|    |.pre.lasergalvos|    |.pre.lasergalvos|    |.trial.lasergalvos|
                                                        +----------------+    +----------------+    +----------------+    +------------------+


Multi-line blocks above are items that run concurrently (and the first one to finish preempts the others), which just means that for example the robot can be running during the prewait & trigger periods, etc, and can have different behavior during the pre and trial phases.  
When you run an experiment, e.g. "roslaunch experiments passivechase.launch", it starts at the StartExperiment block, and moves through (possibly multiple) trials to EndExperiment where the program exits.
 


==================================
How to specify your experiment.
==================================
Assuming you know the specific behavior you want from the system as it progresses through each trial, you'll edit the experimentparams values to configure the system to run your experiment.

The first few sections of experimentparams are mostly administrative (.experiment/.save/.robotspec/.flyspec/etc).  Most of experimental detail is in the .pre/.trial/.post sections that describe the trial behavior.  

The flow of the experiment is controlled by these sections:
.pre.wait1              Wait the specified number of seconds before testing for the trigger event.  Often used to record the pre behavior.
.pre.trigger            Tests for a trigger event that signals entry to the trial's main phase.
.pre.wait2              Wait the specified number of seconds after the trigger, before moving to the "trial" phase.
.post.trigger           Tests for a trigger event that signals the trial is finished.
.post.wait              Wait the specified number of seconds after the post trigger.  Often used to record the post behavior. 


The behavior of the hardware during the pre and trial periods is controlled by these sections: 
.pre.robot              Behavior of the robot       during the prewait and pretrigger periods.
.pre.ledpanels          Behavior of the ledpanels   during the prewait and pretrigger periods.
.pre.lasergalvos        Behavior of the lasergalvos during the prewait and pretrigger periods.
.trial.robot            Behavior of the robot       during the trial body, while waiting for the posttrigger.
.trial.ledpanels        Behavior of the ledpanels   during the trial body, while waiting for the posttrigger.
.trial.lasergalvos      Behavior of the lasergalvos during the trial body, while waiting for the posttrigger.



Note that ".trial" refers to the main body of the trial, after ".pre" and before ".post", whereas "a trial" consists of the whole loop: StartTrial/Pre/Trial/Post/EndTrial

Each of the sections may be disabled or turned off, but they exist if your experiment needs the capability.



==================================
Randomization.
==================================
A number of the parameters below are given as a list of choices, e.g. [10, 20, 30].  For those, one of the entries in the list will be chosen at random for each trial.  If you don't want the randomness, just put one entry in the list.



==================================
# Section ".experiment"
#
#   Describes the experiment, and includes limits on the number of trials 
#   and/or the total time for all trials. 
==================================
self.experimentparams.experiment.description                            Textual description of the experiment.
self.experimentparams.experiment.maxTrials                              Max number of trials, or -1 for no limit.
self.experimentparams.experiment.timeout                                Total time for all trials, or -1 for no limit.




==================================
# Section ".save"
#
#   Specify how to record the data gathered during the experiment.
==================================
self.experimentparams.save.filenamebase                                 String to be prepended to the .csv log files.
self.experimentparams.save.csv                                          Write a .csv file (True/False)
self.experimentparams.save.bag                                          A bag file will allow you to replay the trial later. (True/False)
self.experimentparams.save.mov                                          Save the trial video to an .mov file.  (True/False)
self.experimentparams.save.fmf                                          Save the trial video to an .fmf file.  (True/False)
self.experimentparams.save.imagetopic_list                              List of camera streams to save in the .bag file, e.g. ['camera/image_rect']
self.experimentparams.save.onlyWhileTriggered                           Only save the data between pre.trigger and post.trigger




==================================
# Section ".robotspec"
#
#   Describe the robot, number, presence, size, etc.
==================================
self.experimentparams.robotspec.nRobots                                 How many robots, usually 0 or 1.
self.experimentparams.robotspec.width                                   Width in millimeters.
self.experimentparams.robotspec.height                                  Height in millimeters.
self.experimentparams.robotspec.isPresent                               Set this to False if you remove the robot, but still want the actuation. (True/False)
self.experimentparams.robotspec.description                             Textual description of the robot.




==================================
# Section ".flyspec"
#
#   Describe the fly or flies, gender, size, genetics, etc.
==================================
self.experimentparams.flyspec.nFlies                                    How many flies, 0, 1, 2, 3, ...
self.experimentparams.flyspec.description                               Textual description of the flies.




==================================
# Section ".tracking.exclusionzones"
#
#   Specify regions where the tracking system should not operate.
==================================
self.experimentparams.tracking.exclusionzones.enabled                   True=enable exclusionzones, False=track everywhere.
self.experimentparams.tracking.exclusionzones.point_list                List of Point()'s giving the centerpoint of each zone, e.g. [Point(x=20.0, y=30.0)]
self.experimentparams.tracking.exclusionzones.radius_list               List of radii corresponding the the points above, e.g. [10.0]




==================================
# Section ".home"
#
#   This section specifies a "home" location for the robot, where the 
#   robot will go when it resets between trials.
==================================
self.experimentparams.home.enabled                                      True=go home between trials; False=stay where it is between trials.
self.experimentparams.home.x                                            X position in millimeters.
self.experimentparams.home.y                                            Y position in millimeters.
self.experimentparams.home.speed                                        Speed to travel to home, in millimeters/sec.
self.experimentparams.home.tolerance                                    Distance from home (in millimeters) that counts as being there.




==================================
# Section ".pre.robot"
# Section ".trial.robot"
#
#   Describes how the robot operates during the pre and/or trial phases
#   of the experiment.
==================================
self.experimentparams.___.robot.enabled                                 Enable the robot during this section (True|False) 
self.experimentparams.___.robot.move.mode                               'relative' or 'pattern'.  Move the robot relative to the given frame, or move in a preset pattern.
self.experimentparams.___.robot.move.relative.tracking                  [list of choices]:  True=update the target point continually.  False=the target point is set at the trigger time. 
self.experimentparams.___.robot.move.relative.frameidOriginPosition     [list of choices]:  Frame of reference for position.
self.experimentparams.___.robot.move.relative.frameidOriginAngle        [list of choices]:  Frame of reference for angle
self.experimentparams.___.robot.move.relative.distance                  [list of choices]:  Distance to the target point from the origin frame's position.
self.experimentparams.___.robot.move.relative.angleType                 [list of choices]:  'constant' or 'random'.  Use given angle always, or choose random angle once per move.
self.experimentparams.___.robot.move.relative.angleOffset               [list of choices]:  Angle to the target point from the origin frame's x-axis.
self.experimentparams.___.robot.move.relative.angleOscMag               [list of choices]:  Radian angle to oscillate the angleOffset.
self.experimentparams.___.robot.move.relative.angleOscFreq              [list of choices]:  Hz to oscillate the angleOffset.
self.experimentparams.___.robot.move.relative.speed                     [list of choices]:  Speed at which to move the robot toward the target point. 
self.experimentparams.___.robot.move.relative.speedType                 [list of choices]:  'constant' or 'random'.  Use the given value, or a random frpre of it. 
self.experimentparams.___.robot.move.relative.tolerance                 [list of choices]:  When robot-to-target distance is within this tolerance, then the move is over.
self.experimentparams.___.robot.move.pattern.frameidPosition            [list of choices]:  Frame of reference for position.
self.experimentparams.___.robot.move.pattern.frameidAngle               [list of choices]:  Frame of reference for angle.
self.experimentparams.___.robot.move.pattern.shape                      [list of choices]:  'constant' or 'circle' or 'square' or 'flylogo' or 'spiral' or 'grid'
self.experimentparams.___.robot.move.pattern.hzPattern                  [list of choices]:  Patterns per second.
self.experimentparams.___.robot.move.pattern.hzPoint                    [list of choices]:  The update rate for the actuator.
self.experimentparams.___.robot.move.pattern.count                      [list of choices]:  Number of patterns (-1=infinite)
self.experimentparams.___.robot.move.pattern.size                       [list of choices]:  Point() giving the size of the pattern, e.g. [Point(x=10, y=0)]
self.experimentparams.___.robot.move.pattern.param                      [list of choices]:  Some shapes, e.g. grid, take an additional param.  For grid, it's dot density.
self.experimentparams.___.robot.move.pattern.direction                  [list of choices]:  The direction to step through the pattern points, e.g. -1 or +1.




==================================
# Section ".pre.lasergalvos"
# Section ".trial.lasergalvos"
#
#   Describes how the lasergalvos operate during the pre and/or trial phases
#   of the experiment.
#   The statefilter entries describe a region in "state space" where the 
#   laser will operate, with the state being the state of the fly.  
#   The pattern_list gives the patterns to draw when the statefilter is 
#   satisfied.
==================================
self.experimentparams.___.lasergalvos.enabled                           Enable the lasergalvos during this section (True|False) 
self.experimentparams.___.lasergalvos.statefilterHi_list                List of textual dictionary upper bounds on the fly state.
self.experimentparams.___.lasergalvos.statefilterLo_list                List of textual dictionary lower bounds on the fly state.
self.experimentparams.___.lasergalvos.statefilterCriteria_list          List of "exclusive" or "inclusive" entries for the corresponding bounds.  "exclusive"=laser operates when *not* within the bounds, "inclusive"=laser operates when within the bounds.
self.experimentparams.___.lasergalvos.pattern_list                      List of patterns to draw when statefilter is satisfied.


Each of the three statefilter lists should be the same length, with the items corresponding to one another, such that Lo[0], Hi[0], and Criteria[0] form the first range.  The entries of the statefilterHi & Lo lists are such that each element is a python dictionary in textual form that describes a bound on the MsgFrameState.  For example:
    self.experimentparams.___.lasergalvos.statefilterHi_list       = ["{'pose':{'position':{'x':20, 'y':40}}}"]
    self.experimentparams.___.lasergalvos.statefilterLo_list       = ["{'pose':{'position':{'x':10, 'y':30}}}"]
    self.experimentparams.___.lasergalvos.statefilterCriteria_list = ["exclusive"]

Describes a region bounded as 10<x<20 and 30<y<40 (millimeters), and the laser will not operate when the fly is in that region.  Setting the criteria to "inclusive" would have the laser operate only within that region. 

The allowed field values are the same as those in the MsgFrameState message, which can be shown by the command:
$ rosmsg show MsgFrameState

A summary of the allowed fields is below: 
pose
  position
    x
    y
    z
  orientation
    x
    y
    z
    w
velocity
  linear
    x
    y
    z
  angular
    x
    y
    z
speed
wings
  left
    angle
  right
    angle

Refer to the statefilter example above for how to specify those fields as a textual dictionary.
 
The pattern_list is a list of MsgPattern() fields that are drawn when the statefilter is satisfied, for example:
...pattern_list = [MsgPattern(frameidPosition = 'Fly01Forecast',  # Draw a 2x2 grid on the fly. 
                              frameidAngle    = 'Fly01Forecast',
                              shape      = 'grid', 
                              hzPattern  = 40.0,
                              hzPoint    = 1000.0,
                              count      = 1,
                              size       = Point(x=2, y=2),
                              restart    = False,
                              param      = 3, # Peano curve level.
                              direction  = 1),
                   MsgPattern(frameidPosition = 'Arena',        # Draw a circle in center of arena. 
                              frameidAngle    = 'Arena',
                              shape      = 'circle', 
                              hzPattern  = 40.0,
                              hzPoint    = 1000.0,
                              count      = 1,
                              size       = Point(x=20, y=0), # radius=20
                              restart    = False,
                              param      = 0,
                              direction  = 1),
                             ]
   



==================================
# Section ".pre.ledpanels"
# Section ".trial.ledpanels"
#
#   Describes how the ledpanels operate during the pre and/or trial phases
#   of the experiment.
==================================
self.experimentparams.___.ledpanels.enabled                             Enable the ledpanels during this section (True|False)
self.experimentparams.___.ledpanels.command                             [list of choices]:  'fixed', 'trackposition' (panel position follows fly position), or 'trackview' (panel position follows fly's viewpoint). 
self.experimentparams.___.ledpanels.idPattern                           [list of choices]:  Pattern id to show.
self.experimentparams.___.ledpanels.origin                              [list of choices]:  Point() giving the start position of the panels pattern, e.g. [Point(x=0, y=0)]
self.experimentparams.___.ledpanels.frame_id                            [list of choices]:  Frame of reference when .command is 'trackposition' or 'trackview'
self.experimentparams.___.ledpanels.statefilterHi                       See the statefilter description above in the .lasergalvos section.
self.experimentparams.___.ledpanels.statefilterLo                       See the statefilter description above in the .lasergalvos section.
self.experimentparams.___.ledpanels.statefilterCriteria                 See the statefilter description above in the .lasergalvos section.




==================================
# Section ".pre.wait1"
==================================
self.experimentparams.pre.wait1                                         Number of seconds to wait before testing for the pre.trigger




==================================
# Section ".pre.trigger"
# Section ".post.trigger"
#
#   Triggering waits until the specified relationship holds true between 
#   two objects.  The two frames are called parent and child, and their 
#   relationship may be given as any combination of absolute or relative 
#   speeds, relative distances, or relative angles.  The given conditions 
#   must hold for some period of time, given by timeHold.     
==================================
self.experimentparams.___.trigger.enabled                               Enable the trigger during this section (True|False)
self.experimentparams.___.trigger.frameidParent                         Frame of the parent.
self.experimentparams.___.trigger.frameidChild                          Frame of the child.
self.experimentparams.___.trigger.speedAbsParentMin                     Minimum absolute speed of the parent, in mm/sec.
self.experimentparams.___.trigger.speedAbsParentMax                     Maximum absolute speed of the parent, in mm/sec.
self.experimentparams.___.trigger.speedAbsChildMin                      Minimum absolute speed of the child, in mm/sec.
self.experimentparams.___.trigger.speedAbsChildMax                      Maximum absolute speed of the child, in mm/sec.
self.experimentparams.___.trigger.speedRelMin                           Minimum relative speed of the parent & child, in mm/sec.
self.experimentparams.___.trigger.speedRelMax                           Maximum relative speed of the parent & child, in mm/sec.
self.experimentparams.___.trigger.distanceMin                           Minimum distance between the parent & child, in mm.
self.experimentparams.___.trigger.distanceMax                           Maximum distance between the parent & child, in mm.
self.experimentparams.___.trigger.angleMin                              Minimum angle of the child, in the parent's view, in radians.
self.experimentparams.___.trigger.angleMax                              Maximum angle of the child, in the parent's view, in radians.
self.experimentparams.___.trigger.angleTest                             'inclusive'=trigger on angles inside the bounds, 'exclusive'=trigger on angles outside the bounds.
self.experimentparams.___.trigger.angleTestBilateral                    Mirror symmetry for the angle tests (True|False)
self.experimentparams.___.trigger.timeHold                              The trigger conditions must hold for this many seconds.
self.experimentparams.___.trigger.timeout                               Trigger passes after this many seconds (-1=infinite)




==================================
# Section ".pre.wait2"
==================================
self.experimentparams.pre.wait2                                         Number of seconds to wait after passing the pre.trigger.




==================================
# Section ".post.wait"
==================================
self.experimentparams.post.wait                                         Number of seconds to wait at end of trial, after passing the post.trigger.


# End of file.
