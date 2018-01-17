import numpy as np
import cv2
from math import floor

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def navigable_area_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])

    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def goal_thresh(img, rgb_thresh=(120, 120, 60)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def obstacle_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
# def to_polar_coords(x_pixel, y_pixel, discoveredmap=0, global_x = 0, global_y = 0):
#     # Convert (x_pixel, y_pixel) to (distance, angle) 
#     # in polar coordinates in rover space
#     # Calculate distance to each pixel
#     dist = np.sqrt(x_pixel**2 + y_pixel**2)
#     angles = 0
#     #We'll track the locations of prior explored pixels (terrain)
#     # and determine pixels (terrain) already explored to be 
#     #half value of unexplored pixels
#     half_y = np.zeros_like(y_pixel)
#     half_x = np.zeros_like(x_pixel)

#     if not type(discoveredmap) is int:
#         for x in x_pixel:
#             for y in y_pixel:
#                 if (y,x) in discoveredmap:
#                     half_y[y] = y/2
#                     half_x[x] = x/2
#                 else:
#                     half_y[y] = y
#                     half_x[x] = x

#         angles = np.arctan2(half_y, half_x)
          
#     else:
#         angles = np.arctan2(y_pixel, x_pixel)
#     # Calculate angle away from vertical for each pixel
#     # print("X_pix: ", x_pixel)
#     # print("Y_pix: ", y_pixel)
#     # print("Angle: ", angles)
#     return dist, angles    

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    # print("X_pix: ", x_pixel)
    # print("Y_pix: ", y_pixel)
    angles = np.arctan2(y_pixel, x_pixel)
    # print("Angle: ", angles)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    img = Rover.img
    scale = 10
    offset = 6
    src = np.float32([[15, 140], [118, 95], [200, 95], [301, 140]])
    dst = np.float32([
                [img.shape[1]/2, img.shape[0]-offset], 
                [img.shape[1]/2, img.shape[0]-scale-offset],
                [(img.shape[1]/2)+scale, img.shape[0]-scale-offset],
                [(img.shape[1]/2)+scale, img.shape[0]-offset]
                ])
    # 2) Apply perspective transform
    warped = perspect_transform(img, src, dst)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable_area = navigable_area_thresh(warped)
    goal = goal_thresh(warped)
    obstacles = obstacle_thresh(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles*255
    Rover.vision_image[:,:,1] = goal*255
    Rover.vision_image[:,:,2] = navigable_area*255
    # 5) Convert map image pixel values to rover-centric coords
    x_coord_obstacle, y_coord_obstacle = rover_coords(obstacles)
    x_coord_path, y_coord_path = rover_coords(navigable_area)
    x_coord_goal, y_coord_goal = rover_coords(goal)
    # 6) Convert rover-centric pixel values to world coordinates
    glob_x_coord_obstacle, glob_y_coord_obstacle = pix_to_world(x_coord_obstacle, y_coord_obstacle, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    glob_x_coord_path, glob_y_coord_path = pix_to_world(x_coord_path, y_coord_path, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    glob_x_coord_goal, glob_y_coord_goal = pix_to_world(x_coord_goal, y_coord_goal, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    
    if (Rover.roll < 0.5 or Rover.roll > 359.5) and (Rover.pitch < 0.5 or Rover.pitch > 359.5):
        Rover.worldmap[glob_y_coord_obstacle, glob_x_coord_obstacle, 0] += 255
        Rover.worldmap[glob_y_coord_goal, glob_x_coord_goal, 1] += 255
        Rover.worldmap[glob_y_coord_path, glob_x_coord_path, 2] += 255

    # Update Rover discovered worldmap
    #Rover.discoveredmap[glob_y_coord_path, glob_x_coord_path] += 255
    x_pos = floor(Rover.pos[0])
    y_pos = floor(Rover.pos[1])
    position = (x_pos, y_pos)
    if not(position in Rover.discovered_locs):
        Rover.discovered_locs.append(position)
    # 8) Convert rover-centric pixel positions to polar coordinates
    # path_distance, path_angle = to_polar_coords(x_coord_path, y_coord_path, Rover.discoveredmap, glob_x_coord_path, glob_y_coord_path)
    path_distance, path_angle = to_polar_coords(x_coord_path, y_coord_path)
    goal_distance , goal_angles = to_polar_coords(x_coord_goal, y_coord_goal)

    #If pixel has already been observed, 
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists = path_distance
    Rover.nav_angles = path_angle
    Rover.goal_dists = goal_distance
    Rover.goal_angles = goal_angles
 
    return Rover