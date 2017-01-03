from matrix import *
import sys


def kalman_filter(x, P,measurements):
    F = matrix([[1.0]])
    H = matrix([[1.0]])
    R = matrix([[1.0]])
    I = matrix([[1.0]])
    u = matrix([[0.]])
    # measurement update
    Z = matrix([[measurements]])
    y = Z - (H*x)
    S = H*P*H.transpose() + R
    K = P*H.transpose()*S.inverse()
    x = x + (K*y)
    P = (I - (K*H))*P
    # prediction
    x = (F*x) + u
    P = F*P*F.transpose()
    return x,P


def angle_trunc(a):
    """ This maps all angles to a domain of [-pi, pi] """
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


def get_heading(start, end):
    """ Angle between the starting point and ending point """
    start_x, start_y = start
    end_x, end_y = end
    heading = atan2(end_y - start_y, end_x - start_x)
    heading = angle_trunc(heading)
    return heading


def adjust_heading(pre_heading,heading):
    """ Adjust the heading angle so that it belongs to the same quadrant with its previous one """
    if heading < 0 and pre_heading > 0:
        heading += 2*pi
    elif heading > 0 and pre_heading < 0:
        heading -= 2*pi
    return heading


def distance_between(point1, point2):
    """ Computes distance between point1 and point2. Points are (x, y) pairs. """
    x1, y1 = point1
    x2, y2 = point2
    if x1 == x2 and y1 == y2:
        return 0
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def is_collision(x,y,minX,minY,maxX,maxY):
    """ Check the coordinates to see if they exceed the coordinates of the walls, 
    if so set the coordinates equal to that wall  """
    flag = False
    if x < minX:
        x = minX
        flag = True
    elif x > maxX:
        x = maxX
        flag = True
    if y < minY:
        y = minY
        flag = True
    elif y > maxY:
        y = maxY
        flag = True
    return x,y,flag

def check_straight(measurements,index):
    """ Check the lastest point form a straight line with its two previous points """
    A = measurements[index]
    if index - 1 >= 0 and index - 2 >= 0:
        B = measurements[index-1]
        C = measurements[index-2]
        area = 0.5*(A[0]*B[1] + B[0]*C[1] + C[0]*A[1] - A[0]*C[1] - C[0]*B[1] - B[0]*A[1]) # Shoelace formula
        if area > 0.05:
            return False
    return True
    
def check_near_collision(measurements,index,min_x,max_x,min_y,max_y):
    """ Check if a position is going to hit a wall """
    curr_location = measurements[index]
    distance_to_wall = min(max_x - curr_location[0], curr_location[0] - min_x,
                           max_y - curr_location[1], curr_location[1] - min_y)
    if distance_to_wall < 35:
        if check_straight( measurements, index) == False:
            return True
    return False

def find_walls(measurements):
    """ Find x and y values of the walls """
    m = []
    n = []
    for i in measurements:
        m.append(i[0])
        n.append(i[1])
    min_x = min(m)
    max_x = max(m)
    min_y = min(n)
    max_y = max(n)
    return min_x, max_x, min_y, max_y



def predict(measurements):
    """ Main logic. Predict 60 last positions from a file's measurements """
    minX, maxX, minY, maxY = find_walls(measurements)

    # Prepare matrices for Kalman filter
    turning = matrix([[0.]])
    turning_p = matrix([[1.]])
    bouncing = matrix([[0.]])
    bouncing_p = matrix([[1.]])
    dist = matrix([[0.]])
    dist_p = matrix([[1.]])

    # Use 1d Kalman filter to estimate average distance of every 2 seconds
    for i in range(1, len(measurements)):
        dist_measurement = distance_between(measurements[i-1],measurements[i])
        dist, dist_p = kalman_filter(dist, dist_p, dist_measurement)
    est_avg_dist = dist.value[0][0]

    # Iterate again through the measurements. This time to estimate bouncing and turning values
    collide_before = False
    prev_head = 0
    for i in range(1, len(measurements)):
        cur_head = get_heading(measurements[i-1], measurements[i])
        prev_head = adjust_heading(cur_head, prev_head)
        cur_turning = cur_head - prev_head

        collide_wall = check_near_collision(measurements, i, minX, maxX, minY, maxY)

        # When hexbug moves back from wall, calculate the bouncing angle.
        # If bouncing angle is significant, put it through bouncing Kalman filter
        if not collide_wall and collide_before:
            bounce_angle = cur_head - prev_head
            if bounce_angle > 0.2:
                bouncing, bouncing_p = kalman_filter(bouncing, bouncing_p, bounce_angle)

        # Filter turning
        turning, turning_p = kalman_filter(turning, turning_p, cur_turning)

        # End iteration
        prev_head = cur_head
        collide_before = collide_wall

    # Predict 60 last positions
    prev_point = measurements[-1]
    is_collided = False
    predictions = []
    for i in range(60):
        if is_collided:
            # Bounce if colliding to wall
            predictions.append(predictions[-1])
            is_collided = False
            prev_head += bouncing.value[0][0]

        else:
            # If not colliding move using values from Kalman filters
            heading = prev_head + turning.value[0][0]

            x = prev_point[0]
            y = prev_point[1]

            new_x = int(round(x + est_avg_dist * cos(heading)))
            new_y = int(round(y + est_avg_dist * sin(heading)))

            new_x, new_y, is_collided = is_collision(new_x, new_y, minX, minY, maxX, maxY)

            prev_head = heading
            prev_point = (new_x,new_y)

            predictions.append((new_x, new_y))
    return predictions



########################## Main logic ends. Support methods below ##############################
def read_file(filename):
    file = open(filename, 'r')
    measurement = []
    for line in file:
        a,b = map(int,line.split(','))
        measurement.append([a,b])
    return measurement

def write_to_file(file_name,data):
    file = open(file_name,'w')
    for i in data:
        line = str(i[0]) + ',' + str(i[1]) + '\n'
        file.write(line)
    file.close()


######## Main function ################
if __name__ == "__main__":
    total = len(sys.argv)
    if len(sys.argv) != 2:
        print ("Invalid Argument")
        sys.exit(1)

    measurements = read_file(str(sys.argv[1]))
    prediction = predict(measurements)
    write_to_file("prediction.txt", prediction)
