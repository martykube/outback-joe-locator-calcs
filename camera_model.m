#########################
# Camera calibration
#########################

image_size = [320, 240];
image_center = image_size / 2;
camera_matrix = ...
[277.2, 0.0, image_center(1);
 0.0, 277.2, image_center(2);
 0.0, 0.0, 1.0];

 # plumb bob model
distortion_coefficients = [0.0, 0.0, 0.0, 0.0, 0.0]

function distorted = distort(point, kc)
  x = point(1);
  y = point(2);
  r = sqrt(x^2 + y^2);
  radial = 1 + kc(1) * r^2 + kc(2) * r^4 + kc(5) * r^6;
  tangential = ...
  [
   2 * kc(3) * x * y + kc(4) * (r^2 + 2 * x^2);
   kc(3) * (r^2 + 2 * y^2) + 2 * kc(4) * x * y
  ];
  distorted = radial * [x, y]' + tangential;
  distorted = [distorted(1), distorted(2), 1]';
endfunction

#########################
# 3D to 2D
# For known 3D point calculate image 2D point
#########################

# Target on ground, 3m to the front and left of the local origin
target_world = [3, 3, 0, 1]'

# FCU frame in local coordinates
fcu_rotation = [0.9987, -0.001072, -0.0006237, 0.05031]
fcu_translation = [0.02420, 0.02760, 15.18]'

# use python script to get rotation matrix from quaternion:
# import transformations
# transformations.quaternion_matrix([0.9987, -0.001072, -0.0006237, 0.05031])
fcu_frame = ...
[ 
  0.9949367 , -0.10049446, -0.00135373,  fcu_translation(1);
  0.10049713,  0.99493518,  0.00207859,  fcu_translation(2);
  0.00113799, -0.00220411,  0.99999692,  fcu_translation(3);
  0.        ,  0.        ,  0.        ,  1.        
]

# Camera frame in FCU coordinates
# Camera is pitched down 90 degrees, +pi/2 rotation around y axis
camera_rotation = [ 0.70710678,  0.        ,  0.70710678,  0.        ]
camera_translation = [0.11, 0, 0.001]'

camera_frame = ...
[ 
   0.,  0.,  1.,  camera_translation(1);
   0.,  1.,  0.,  camera_translation(2);
  -1.,  0.,  0.,  camera_translation(3);
   0.,  0.,  0.,  1.
]

# rotate point in world coordinates into FCU frame
target_fcu_frame =  inv(fcu_frame) * target_world
# rotate from FCU frame into camera_frame
target_camera_frame =  inv(camera_frame) * target_fcu_frame

# Change from vehicle coordinates to image coordinates
# Vehicle coordinates are: (forward, left, up)
# Image coordinates are:   (right,   down, forward)
target_image = [ -target_camera_frame(2); -target_camera_frame(3); target_camera_frame(1)]

# Normalize image coordinates
target_image_normalized = target_image / target_image(3)

# distort
target_distored = distort(target_image_normalized, distortion_coefficients)

# apply the camera matrix to find pixel coordinates
target = camera_matrix * target_distored

# calculate error
forward_error_in_pixels = norm([105.5; 67] - target(1:2))


#########################
# 2D to 3D
# For a 2D target on an image and known vehicle pose, calculate the 3D coordinates.
#########################

target_pixels = [105.5, 67.]

# use inverse camera matrix to go from pixels to camera coordinates
target_2D = [target_pixels(1); target_pixels(2); 1]
target_3D = inv(camera_matrix) * target_2D

# undistort
# There is no closed form solution to the inverse distortion.  Skip for now. No distorition in SITL camera...
# http://peterabeles.com/blog/?p=73

# Change from image coordinate system to vehicle coordinate system
target_vehicle = [target_3D(3); -target_3D(1); -target_3D(2); 1]

# We have a direction vector from vehicle to target.  The direction vector is given by 2 points:
# 1) origin of camera frame
# 2) target point  
# Rotate both points into the local frame and then project along that direction vector to an 
# intersection with the ground:  joe = scale * (direction_local - camera_origin_local)
direction_local = fcu_frame * camera_frame * target_vehicle
camera_origin_local = fcu_frame * camera_frame * [0; 0; 0; 1]
# At the ground plane the z component is 0.  Find the vector scaling factor from that dimension.
scale = - camera_origin_local(3) / (direction_local(3) - camera_origin_local(3))
# Apply scale to all dimensions
joe = scale * direction_local + (1 - scale) * camera_origin_local

# should match target_world
error_in_meters = norm(joe - target_world(1:3))
