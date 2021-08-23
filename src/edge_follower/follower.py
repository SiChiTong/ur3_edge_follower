from __future__ import division

saved_cloudXYZ = None

# Save the captured pointcloud
def freeze_cloud(freeze,cloudXYZ):
    global saved_cloudXYZ
    if freeze and saved_cloudXYZ is None:
        global saved_cloudXYZ
        saved_cloudXYZ = cloudXYZ
        print("Cloud is saved")
    return saved_cloudXYZ
    
