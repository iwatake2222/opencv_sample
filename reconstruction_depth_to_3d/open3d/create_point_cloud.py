import open3d as o3d

image_rgb = o3d.io.read_image("image.png")
image_depth = o3d.io.read_image("image_depth.png")
width = int(image_rgb.get_max_bound()[0])
height = int(image_rgb.get_max_bound()[1])
camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width=width, height=height, fx=500, fy=500,cx=width/2, cy=height/2)
image_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image_rgb, image_depth, convert_rgb_to_intensity=False)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(image_rgbd, camera_intrinsic)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd], width=640, height=480)
o3d.io.write_point_cloud("output.ply", pcd)
