"""RGB-D point cloud in the actual Go2 IMU frame, from the saved source."""
import numpy as np

def depth_to_cloud(depth, sensor_yaw, camera_mount, imu_mount, focal_px=512*24/42, center_px=(256,256)):
    depth=np.asarray(depth);row,col=np.indices(depth.shape)
    keep=np.isfinite(depth)&(depth>.05)&(depth<20)
    points=np.column_stack([depth[keep],-(col[keep]-center_px[0])/focal_px*depth[keep],-(row[keep]-center_px[1])/focal_px*depth[keep]])@sensor_yaw.T+(camera_mount-imu_mount)
    body=points+imu_mount
    self_mask=(np.abs(body[:,0])<.5)&(np.abs(body[:,1])<.3)&(body[:,2]>-.45)&(body[:,2]<.3)
    points=points[~self_mask]
    stride=max(1,(len(points)+7999)//8000)
    return points[::stride],keep,self_mask
