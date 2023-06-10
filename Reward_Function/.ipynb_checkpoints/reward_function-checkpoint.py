import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[2.59861, 0.44981, 3.50098, 0.06643],
                        [2.36821, 0.42107, 3.77231, 0.06155],
                        [2.1351, 0.39855, 4.0, 0.05855],
                        [1.89659, 0.38121, 4.0, 0.05978],
                        [1.64935, 0.36815, 4.0, 0.0619],
                        [1.3894, 0.35854, 4.0, 0.06503],
                        [1.11281, 0.35159, 4.0, 0.06917],
                        [0.81885, 0.34638, 4.0, 0.0735],
                        [0.51808, 0.34172, 4.0, 0.0752],
                        [0.21732, 0.33706, 4.0, 0.0752],
                        [-0.08345, 0.3324, 3.42748, 0.08776],
                        [-0.38421, 0.32774, 2.79836, 0.10749],
                        [-0.68498, 0.32308, 2.42026, 0.12428],
                        [-0.98571, 0.31836, 2.16126, 0.13916],
                        [-1.2832, 0.30725, 1.95806, 0.15204],
                        [-1.57436, 0.28392, 1.79078, 0.16311],
                        [-1.85636, 0.24346, 1.65928, 0.17169],
                        [-2.12663, 0.18188, 1.55172, 0.17864],
                        [-2.38281, 0.09597, 1.46045, 0.18502],
                        [-2.62231, -0.0174, 1.38256, 0.19166],
                        [-2.84181, -0.16183, 1.31492, 0.19983],
                        [-3.03659, -0.34144, 1.2547, 0.21116],
                        [-3.19761, -0.55919, 1.20141, 0.22542],
                        [-3.30778, -0.80509, 1.15, 0.23431],
                        [-3.35497, -1.05032, 1.15, 0.21715],
                        [-3.34448, -1.27954, 1.15, 0.19954],
                        [-3.28343, -1.48669, 1.15, 0.18779],
                        [-3.17551, -1.6675, 1.15, 0.1831],
                        [-3.01991, -1.81509, 1.33267, 0.16092],
                        [-2.828, -1.93236, 1.47027, 0.15297],
                        [-2.60344, -2.0183, 1.66096, 0.14476],
                        [-2.35023, -2.0727, 1.94465, 0.13318],
                        [-2.07385, -2.09795, 2.43245, 0.11409],
                        [-1.78174, -2.10068, 3.57251, 0.08177],
                        [-1.48108, -2.09163, 4.0, 0.0752],
                        [-1.1809, -2.08791, 4.0, 0.07505],
                        [-0.88147, -2.08975, 4.0, 0.07486],
                        [-0.58294, -2.09738, 4.0, 0.07466],
                        [-0.28542, -2.11103, 4.0, 0.07446],
[0.01095, -2.13095, 4.0, 0.07426],
[0.30603, -2.15738, 3.25206, 0.0911],
[0.59966, -2.1906, 2.75598, 0.10722],
[0.89164, -2.23088, 2.43415, 0.12109],
[1.18168, -2.27849, 2.19966, 0.13362],
[1.46896, -2.33416, 2.0208, 0.1448],
[1.74629, -2.40138, 1.87586, 0.15212],
[2.00905, -2.48279, 1.74341, 0.15778],
[2.25663, -2.58112, 1.70354, 0.15638],
[2.48729, -2.69836, 1.66497, 0.1554],
[2.69868, -2.8358, 1.62651, 0.15502],
[2.88787, -2.99417, 1.59061, 0.15511],
[3.05086, -3.17373, 1.55601, 0.15585],
[3.1854, -3.37213, 1.52156, 0.15754],
[3.28882, -3.58656, 1.48686, 0.16011],
[3.3582, -3.8135, 1.44911, 0.16376],
[3.39066, -4.04842, 1.41273, 0.16787],
[3.38372, -4.28562, 1.41273, 0.16798],
[3.33569, -4.51826, 1.41273, 0.16815],
[3.24603, -4.73859, 1.41273, 0.16838],
[3.11525, -4.93842, 1.41273, 0.16905],
[2.94443, -5.1096, 1.6843, 0.14358],
[2.74651, -5.25699, 1.79664, 0.13735],
[2.52435, -5.38074, 1.93361, 0.13152],
[2.28035, -5.48117, 2.10385, 0.12542],
[2.01699, -5.55894, 2.325, 0.11811],
[1.73739, -5.61539, 2.63148, 0.1084],
[1.44572, -5.65296, 3.09233, 0.0951],
[1.14726, -5.67552, 3.91146, 0.07652],
[0.84678, -5.68819, 4.0, 0.07519],
[0.54721, -5.7051, 4.0, 0.07501],
[0.24822, -5.72629, 4.0, 0.07493],
[-0.05017, -5.75179, 4.0, 0.07487],
[-0.34796, -5.78164, 4.0, 0.07482],
[-0.64513, -5.81589, 3.47455, 0.08609],
[-0.94166, -5.85463, 2.7632, 0.10823],
[-1.23753, -5.89794, 2.3624, 0.12658],
[-1.5327, -5.94595, 2.09433, 0.14279],
[-1.81241, -5.98647, 1.89763, 0.14894],
[-2.08774, -6.01534, 1.74531, 0.15862],
[-2.35724, -6.02713, 1.74531, 0.15456],
[-2.62005, -6.0173, 1.74531, 0.15068],
[-2.87543, -5.98178, 1.74531, 0.14773],
[-3.12231, -5.91628, 1.74531, 0.14635],
[-3.3588, -5.81566, 1.99435, 0.12887],
[-3.58661, -5.68734, 2.17921, 0.11998],
[-3.80642, -5.53448, 2.41948, 0.11066],
[-4.01916, -5.36019, 2.75761, 0.09973],
[-4.22615, -5.16788, 3.29236, 0.08582],
[-4.4293, -4.96142, 3.76333, 0.07697],
[-4.63068, -4.745, 3.66553, 0.08065],
[-4.82679, -4.52113, 3.57246, 0.08331],
[-5.01392, -4.29334, 3.48349, 0.08463],
[-5.19171, -4.06159, 3.40072, 0.08589],
[-5.35992, -3.82574, 3.30844, 0.08756],
[-5.51825, -3.58567, 3.1624, 0.09094],
[-5.66638, -3.34127, 3.1624, 0.09037],
[-5.80394, -3.09242, 3.1624, 0.08991],
[-5.93055, -2.83901, 3.1624, 0.08958],
[-6.04564, -2.58089, 3.1624, 0.08937],
[-6.14815, -2.31772, 3.18368, 0.08871],
[-6.23819, -2.04971, 3.20032, 0.08835],
[-6.31577, -1.77701, 3.2167, 0.08814],
[-6.38082, -1.49982, 3.22856, 0.08819],
[-6.43321, -1.21835, 3.24007, 0.08836],
[-6.47272, -0.93289, 3.24969, 0.08868],
[-6.49911, -0.6438, 3.25793, 0.0891],
[-6.5121, -0.35163, 3.24718, 0.09007],
[-6.5114, -0.0571, 3.22925, 0.09121],
[-6.49681, 0.23882, 3.20037, 0.09258],
[-6.46815, 0.53498, 3.1677, 0.09393],
[-6.42534, 0.83014, 3.12871, 0.09533],
[-6.36834, 1.12321, 2.88967, 0.10332],
[-6.29711, 1.41337, 2.61423, 0.11429],
[-6.21151, 1.69992, 2.40395, 0.12441],
[-6.11136, 1.98217, 2.23553, 0.13397],
[-5.9965, 2.25929, 2.09813, 0.14297],
[-5.86483, 2.52911, 1.9822, 0.15146],
[-5.71425, 2.78705, 1.88235, 0.15867],
[-5.54482, 3.02668, 1.79029, 0.16393],
[-5.35828, 3.24207, 1.70772, 0.16685],
[-5.15716, 3.42909, 1.63363, 0.16811],
[-4.94415, 3.58524, 1.56433, 0.16884],
[-4.72183, 3.70909, 1.56433, 0.16268],
[-4.49255, 3.79946, 1.56433, 0.15754],
[-4.25855, 3.85513, 1.56433, 0.15376],
[-4.02209, 3.87442, 1.56433, 0.15166],
[-3.78575, 3.85467, 1.63725, 0.14485],
[-3.55195, 3.79875, 1.70967, 0.14061],
[-3.32258, 3.7081, 1.79261, 0.13759],
[-3.09923, 3.5834, 1.83339, 0.13953],
[-2.88251, 3.42373, 1.81936, 0.14796],
[-2.65653, 3.29824, 1.79565, 0.14395],
[-2.42473, 3.20515, 1.79565, 0.13911],
[-2.18942, 3.14282, 1.79565, 0.13557],
[-1.95225, 3.11015, 1.79565, 0.13333],
[-1.71451, 3.10672, 1.79565, 0.13241],
[-1.47736, 3.13318, 2.11295, 0.11293],
[-1.24122, 3.18163, 2.43467, 0.09901],
[-1.00607, 3.24718, 2.95881, 0.0825],
[-0.77171, 3.32471, 2.91197, 0.08477],
[-0.53779, 3.40875, 2.65735, 0.09354],
[-0.28705, 3.49294, 2.45831, 0.10759],
[-0.03561, 3.56855, 2.28782, 0.11477],
[0.21642, 3.63298, 2.28782, 0.11371],
[0.46878, 3.68372, 2.25727, 0.11403],
[0.72103, 3.71845, 2.19368, 0.11607],
[0.97262, 3.73499, 2.11115, 0.11943],
[1.22293, 3.73101, 1.872, 0.13373],
[1.47162, 3.70741, 1.69192, 0.14765],
[1.71836, 3.663, 1.55516, 0.16121],
[1.96146, 3.59656, 1.4409, 0.17491],
[2.20303, 3.55458, 1.4409, 0.17016],
[2.44158, 3.54152, 1.4409, 0.16581],
[2.67499, 3.56159, 1.4409, 0.16259],
[2.90017, 3.61903, 1.4409, 0.16128],
[3.1121, 3.71905, 1.5364, 0.15253],
[3.30837, 3.85672, 1.64991, 0.14531],
[3.48729, 4.02809, 1.79002, 0.13841],
[3.64782, 4.22949, 1.94631, 0.13233],
[3.78981, 4.45702, 1.76185, 0.15223],
[3.91435, 4.70618, 1.61949, 0.172],
[4.02411, 4.972, 1.50502, 0.19108],
[4.15606, 5.21865, 1.41089, 0.19826],
                        [4.30925, 5.43616, 1.3271, 0.20047],
                        [4.4819, 5.6195, 1.3271, 0.18976],
                        [4.67092, 5.76513, 1.3271, 0.1798],
                        [4.87232, 5.8704, 1.3094, 0.17355],
                        [5.08141, 5.93309, 1.27872, 0.1707],
                        [5.29261, 5.95077, 1.27872, 0.16574],
                        [5.50039, 5.92753, 1.27872, 0.1635],
                        [5.69959, 5.86349, 1.27872, 0.16364],
                        [5.88404, 5.75782, 1.27872, 0.16624],
                        [6.04521, 5.60917, 1.39373, 0.15731],
                        [6.17939, 5.42519, 1.48113, 0.15374],
                        [6.28288, 5.21073, 1.58404, 0.15033],
                        [6.353, 4.9707, 1.71085, 0.14616],
                        [6.38845, 4.71032, 1.87008, 0.14052],
                        [6.38972, 4.43502, 2.08394, 0.13211],
                        [6.35962, 4.15021, 2.39189, 0.11974],
                        [6.30349, 3.86077, 2.82451, 0.10438],
                        [6.22858, 3.57059, 2.79153, 0.10736],
                        [6.13647, 3.28533, 2.75568, 0.10878],
                        [6.02821, 3.00869, 2.71928, 0.10925],
                        [5.90463, 2.74232, 2.67798, 0.10965],
                        [5.7665, 2.48738, 2.63704, 0.10995],
                        [5.61461, 2.24473, 2.59393, 0.11036],
                        [5.44967, 2.01512, 2.54614, 0.11104],
                        [5.27237, 1.7992, 2.49541, 0.11196],
                        [5.08341, 1.59769, 2.44239, 0.11311],
                        [4.88351, 1.41131, 2.44239, 0.1119],
                        [4.67352, 1.24088, 2.44239, 0.11073],
                        [4.45447, 1.08736, 2.44239, 0.10952],
                        [4.22777, 0.95183, 2.44239, 0.10814],
                        [3.99546, 0.83551, 2.54655, 0.10202],
                        [3.7605, 0.73705, 2.6615, 0.09572],
                        [3.52523, 0.65477, 2.79237, 0.08926],
                        [3.29116, 0.58677, 2.93509, 0.08305],
                        [3.05893, 0.53112, 3.09581, 0.07714],
                        [2.82836, 0.48602, 3.28018, 0.07162]]
        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 32  # seconds (time that is easily done by model)
        FASTEST_TIME = 19  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
