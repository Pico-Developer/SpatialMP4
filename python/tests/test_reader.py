import spatialmp4


def test_version():
    print(spatialmp4.__version__)


def test_reader():
    reader = spatialmp4.Reader("video/test.mp4")
    
    # Print basic information
    print(f"Has RGB: {reader.has_rgb()}")
    print(f"Has Depth: {reader.has_depth()}")
    print(f"Has Pose: {reader.has_pose()}")
    print(f"Duration: {reader.get_duration()} seconds")
    print(f"RGB FPS: {reader.get_rgb_fps()}")
    print(f"Depth FPS: {reader.get_depth_fps()}")
    
    # Get camera parameters
    rgb_intrinsics = reader.get_rgb_intrinsics_left()
    print(f"RGB Intrinsics: fx={rgb_intrinsics.fx}, fy={rgb_intrinsics.fy}, cx={rgb_intrinsics.cx}, cy={rgb_intrinsics.cy}")
    
    # Set read mode
    reader.set_read_mode(spatialmp4.ReadMode.RGB_ONLY)
    
    # Read frames
    while reader.has_next():
        # Load RGB frame
        rgb_frame = reader.load_rgb()
        
        # Access the frame data (returns numpy arrays)
        left_rgb = rgb_frame.left_rgb  # Shape: (height, width, 3)
        right_rgb = rgb_frame.right_rgb  # Shape: (height, width, 3)
        
        # Get pose data
        pose = rgb_frame.pose
        print(f"Frame timestamp: {rgb_frame.timestamp}")
        print(f"Pose: x={pose.x}, y={pose.y}, z={pose.z}, qw={pose.qw}, qx={pose.qx}, qy={pose.qy}, qz={pose.qz}")

    # Example of loading both RGB and depth
    reader.reset()
    reader.set_read_mode(spatialmp4.ReadMode.DEPTH_FIRST)
    
    if reader.has_next():
        # Load both RGB and depth frames
        rgb_frame, depth_frame = reader.load_both()
        
        # Access depth data (returns numpy array)
        depth = depth_frame.depth  # Shape: (height, width)
        print(f"Depth: {depth.shape}")
