string   filenamebase           # The first part of the filename, before the timestamp, e.g. 'test'.
string   timestamp              # The timestamp in string form, e.g. '20130309031415'.  This field is set by the code.
bool     csv                    # Should we save the Arenastate to .csv?
bool     bag                    # Should we save system input to .bag?  (i.e. camera/image_rect, etc)
bool     png                    # Should we save image topics to .png files?
bool     onlyWhileTriggered     # Save all the trial data, or just from trial-start to trial-end?
string[] imagetopic_list        # List of image topics to save to .png files (if saving png).
