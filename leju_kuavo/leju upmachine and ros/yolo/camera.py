import pyrealsense2 as rs
import numpy as np
import cv2

# ÅäÖÃÉî¶ÈºÍÑÕÉ«Á÷
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Æô¶¯Á÷
pipeline.start(config)

# »ñÈ¡Éî¶È´«¸ÐÆ÷µÄÉî¶È±ê³ß
depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

try:
    while True:
        # µÈ´ýÒ»×éÁ¬¹áµÄÖ¡£ºÉî¶ÈºÍÑÕÉ«
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # ½«Í¼Ïñ×ª»»ÎªnumpyÊý×é
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # »ñÈ¡Í¼ÏñÖÐÐÄµãµÄ×ø±ê
        height, width = depth_image.shape
        center_x, center_y = width // 2, height // 2

        # »ñÈ¡ÖÐÐÄµãµÄÉî¶ÈÖµ£¨µ¥Î»£ºÃ×£©
        depth_value = depth_frame.get_distance(center_x, center_y)
        
        # ´òÓ¡ÖÐÐÄµãµÄÉî¶ÈÐÅÏ¢
        print(f"ÖÐÐÄµã ({center_x}, {center_y}) µÄÉî¶È: {depth_value:.3f} Ã×")

        # Ó¦ÓÃÑÕÉ«Í¼ÒÔ¿ÉÊÓ»¯Éî¶ÈÍ¼Ïñ
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # ÔÚ²ÊÉ«Í¼ÏñºÍÉî¶ÈÍ¼ÏñÉÏ±ê¼ÇÖÐÐÄµã
        cv2.circle(color_image, (center_x, center_y), 4, (0, 0, 255), -1)
        cv2.circle(depth_colormap, (center_x, center_y), 4, (0, 0, 255), -1)

        # ¶ÑµþÍ¼ÏñË®Æ½ÏÔÊ¾
        images = np.hstack((color_image, depth_colormap))

        # ÏÔÊ¾Í¼Ïñ
        cv2.imshow('RealSense', images)

        # °´'q'¼üÍË³öÑ­»·
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Í£Ö¹Á÷
    pipeline.stop()
    cv2.destroyAllWindows()