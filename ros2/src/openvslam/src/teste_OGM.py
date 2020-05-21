import OGM as ogm

points_2d, keyframe = ogm.save_data(100, 1.5, './../../../maps/landmarks_and_timestamp.txt', './../../../maps/keyframe_trajectory.txt')
ogm.draw_2d_point_cloud(points_2d, keyframe)
