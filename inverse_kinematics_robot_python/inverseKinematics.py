class inverseKinematics:

    def __init__(self, val):
        self.test = val

    def set_ft_dist(self, height):

        angles = np.zeros((5,np.size(height)))

        angles[2,:] = np.arccos((height * height - 59.8 * 59.8 - 59.3 * 59.3) / 2 / 59.8 / 59.3)

        angles[1,:] = np.arctan(59.8 * np.sin(angles[2,:]) / (59.8 * np.cos(angles[2,:]) + 59.3))

        angles[3,:] = angles[1,:] - angles[2,:]

        return angles

    def points2angles(self, points):
        # takes in x y z position of foot and returns angles of motors
        x = points[0,:]
        y = points[1,:]
        z = points[2,:]

        # net rotation around the x-axis is 0.
        angle_h = np.arctan(y/z);

        # calculate position of ankle. Foot rotation does not change x position
        # of ankle
        y_a = y - 45.8 * np.sin(angle_h);
        z_a = z + 45.8 * np.cos(angle_h);

        # similarly, calculate bottom of hip position. hip rotation also does not
        # change x position of hip bottom
        y_h = 45.8 * np.sin(angle_h);
        z_h = - 45.8 * np.cos(angle_h);

        ft_dist = np.sqrt(x*x + (y_a - y_h)*(y_a - y_h) + (z_a - z_h)*(z_a - z_h));

        angles = self.set_ft_dist(ft_dist);

        angles[0,:] = angle_h;
        angles[4,:] = -angle_h;

    angle_ft = asin(x/ft_dist);

    angles(2) = angles(2) + angle_ft;
    angles(4) = angles(4) + angle_ft;

    def angles2values(self, angles):
        angles[1,:] = angles[1,:] + 0.3788
        angles[2,:] = angles[2,:] - 0.7382

        r_values = np.zeros((10,np.size(angles,1)))

        angles = angles / np.pi * 180 / 0.237

        # left leg values
        r_values[0,:] = 500;
        r_values[1,:] = 500 + angles[1];
        r_values[2,:] = 500 + angles[2];
        r_values[3,:] = 500 + angles[3];
        r_values[4,:] = 500;
    
        # right leg values
        r_values[5,:] = 500;
        r_values[6,:] = 500 - angles[1];
        r_values[7,:] = 500 - angles[2];
        r_values[8,:] = 500 - angles[3];
        r_values[9,:] = 500;

        return r_values

