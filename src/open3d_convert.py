import open3d as o3d

# ply 불러오기
pcd = o3d.io.read_point_cloud("/home/moon/cloud3.ply")

# pcd로 저장
o3d.io.write_point_cloud("/home/moon/cloud3.pcd", pcd)
