import os
def human_urdf_path():
    """
    Uses rospack to file the human urdf file, if ros is not installed
    simply assumes that the packages are installed in the same
    directory
    """
    directory = "../../human_bio_urdf"
    try:
        import rospkg
        rospack = rospkg.RosPack()
        directory = rospack.get_path('human_bio_urdf')
    except Exception as e:
        print "rospack failed with error code: {}".format(e)
    filepath = directory + "/urdf/human_bio.urdf"
    if os.path.exists(filepath):
        print "found URDF at : {}".format(filepath)
        return filepath
    else:
        print "human_bio.urdf not found !!!"
        return str("")
