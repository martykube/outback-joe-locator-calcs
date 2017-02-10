########################################
# Input from vehicle...
# Target in image and quad attitude
########################################

target_pixels = [2540, 1983];
fcu_translation = [0.119, 2.18, 11.0];
fcu_rotation = [0.701, -0.00725, -0.0253, 0.712]

#######################################
# Fixed parameters
# Camera frame and camera calibration
#######################################

image_size = [4928, 3264];
camera_matrix = ...
[
 3714.,    0.0, 2535.7;
 0.0,   3784.0, 1637.4;
 0.0,      0.0,    1.0
];
camera_translation = [0., -0.18 0.04];
camera_rotation = [1., 0., 0., 0.];

function q_matrix = quaternion_matrix(quaternion)
	 q = [quaternion(1); quaternion(2); quaternion(3); quaternion(4)];
	 n = dot(q, q)
	 if (n < 1e-7)
	   q_matrix = eye(4);
	 else
	   q = q * power(2.0/n, 1.0/2.0);
	   q = q * q';
	   q_matrix = ...
	   [
            1.0-q(3, 3)-q(4, 4),     q(2, 3)-q(4, 1),     q(2, 4)+q(3, 1), 0.0;
                q(2, 3)+q(4, 1), 1.0-q(2, 2)-q(4, 4),     q(3, 4)-q(2, 1), 0.0;
                q(2, 4)-q(3, 1),     q(3, 4)+q(2, 1), 1.0-q(2, 2)-q(3, 3), 0.0;
                            0.0,                 0.0,                 0.0, 1.0
	    ];
	 endif
endfunction


#########################################
# calcs
#########################################

# we have image location in pixels relative to the upper left corner of the image.
# Change to 3D coordinates, centered om the camera lens, in meters.  
# Image coordinates (right, down, forward)
target_2D = [target_pixels(1); target_pixels(2); 1]
target_3D = inv(camera_matrix) * target_2D

# Rotate target into vehicle coordinates (forward, left, up)
target_vehicle = [target_3D(3); -target_3D(1); -target_3D(2); 1]
# Also rotate in camers lens center into vehicle coordinates 
# The origin of image coordinates
camera_vehicle = [0; 0; 0; 1]


# nikon frame
camera_frame = quaternion_matrix(camera_rotation)
camera_frame(1, 4) = camera_translation(1)
camera_frame(2, 4) = camera_translation(2)
camera_frame(3, 4) = camera_translation(3)

# fcu frame
fcu_frame = quaternion_matrix(fcu_rotation)
fcu_frame(1, 4) = fcu_translation(1)
fcu_frame(2, 4) = fcu_translation(2)
fcu_frame(3, 4) = fcu_translation(3)

# rotate
direction_local = fcu_frame * camera_frame * target_vehicle
camera_origin_local = fcu_frame * camera_frame * camera_vehicle

# find ground plane intersection
scale = - camera_origin_local(3) / (direction_local(3) - camera_origin_local(3))
joe = scale * direction_local + (1 - scale) * camera_origin_local




