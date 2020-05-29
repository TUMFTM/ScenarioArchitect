import os
import math
import numpy as np
from shapely.geometry import Polygon, Point, LineString
import trajectory_planning_helpers as tph
import pandas as pd


"""
    Python version: 3.7
    Created by: Tim Stahl (based on work by George Willis & Leonhard Hermansdorfer)
    Created on: 18.02.2020

    This package takes track bounds (coordinate arrays) and generates a reference line, including matching
    normal-vectors and track-widths.
"""


def generate_refline(bound_l: np.ndarray,
                     bound_r: np.ndarray,
                     resolution: float = 1.0):
    """
    This function takes track bounds and generates a reference line, including corresponding normal-vectors and
    track-widths.

    :param bound_l:         bound coords left given as numpy array (columns holding x and y coords)
    :param bound_r:         bound coords right given as numpy array (columns holding x and y coords)
    :param resolution:      desired resolution in meters
    :returns (output_rl,    coordinates of reference-line (columns holding x and y coords)
              v_normal,     normal-vectors based on each of the reference-line coordinates
              tw_left,      track-width to the left of reference point (along normal-vector)
              tw_right)     track-width to the right of reference point (along normal-vector)
    """

    if bound_l.shape[0] != bound_r.shape[0]:
        raise ValueError("Bound coordinate arrays must hold same length!")

    # check if provided data is closed --> remove closure
    if np.hypot(bound_l[0, 0] - bound_l[-1, 0], bound_l[0, 1] - bound_l[-1, 1]) < 0.1:
        bound_l = bound_l[:-1, :]
        bound_r = bound_r[:-1, :]

    # -- create initial center line (based on both bounds) -------------------------------------------------------------
    center_line = 0.5 * (bound_r + bound_l)

    # get current distances and interpolate to desired resolution (distance of points)
    distances, distances_sum = get_distance(input_coords=center_line)
    iter_track = interpol_new_points(input_coord=center_line,
                                     distances=distances,
                                     track_res=resolution)

    # -- create track polygon ------------------------------------------------------------------------------------------
    # --> used for smoothing - check if smoothed line is still within bounds
    # --> used for track width calculation at normal-vectors (intersection of normal-vectors with the polygon)
    # To create a valid polygon with definite enclosure of normal vectors at start and end, the polygon is extended
    # at start and end (first/last vector along bound is added again)
    both_tb = np.row_stack((
        bound_l[0, :] - np.diff(bound_l[:2, :], axis=0),
        bound_l,
        bound_l[-1, :] + np.diff(bound_l[-2:, :], axis=0),
        bound_r[-1, :] + np.diff(bound_r[-2:, :], axis=0),
        np.flipud(bound_r),
        bound_r[0, :] - np.diff(bound_r[:2, :], axis=0)
    ))
    track_polygon = Polygon(both_tb)

    # if track polygon not valid, assume closure (overlapping start and end)
    if not track_polygon.is_valid:
        point = Point(bound_l[0, :])
        polygon = Polygon(bound_r)

        # if bound_l is inner one
        if polygon.contains(point):
            track_polygon = Polygon(shell=bound_r,
                                    holes=[bound_l])
        else:
            track_polygon = Polygon(shell=bound_l,
                                    holes=[bound_r])

    # -- iteratively smooth until track bound is left ------------------------------------------------------------------
    bool_check = inpolygon(track_polygon=track_polygon,
                           input_coord=iter_track)
    n = 0
    temp_iter_track = np.copy(iter_track)
    while bool_check and not n > 500:
        temp_iter_track = movmean_smoothing(input_coord=temp_iter_track,
                                            window_size=9)
        n += 1
        if n % 5 == 0:
            bool_check = inpolygon(track_polygon=track_polygon,
                                   input_coord=temp_iter_track)

            if bool_check:
                iter_track = np.copy(temp_iter_track)

    # - split into sections (based on heading changes) -----------------------------------------------------------------
    heading_angle = get_heading_angle(input_coord=iter_track)
    section_row_indices = find_sections(heading_angle=heading_angle)[0]

    # -- smooth all sections individually until no overlapping normal-vectors exist ------------------------------------
    output_rl = smooth_sections(input_coord=iter_track,
                                section_row_indices=section_row_indices,
                                track_polygon=track_polygon)
    heading_angle = get_heading_angle(input_coord=output_rl)
    output_rl = final_smooth(input_coord=output_rl,
                             heading_angle=heading_angle)

    # -- generate outputs ----------------------------------------------------------------------------------------------
    # calculate normal vectors
    v_normal = create_normal_vectors(input_coord=output_rl)

    # get intersection points with bounds (in order to determine track-width)
    intersects_left, intersects_right = get_intersect_points(track_polygon=track_polygon,
                                                             input_coord=output_rl,
                                                             v_normal=v_normal)

    # calculate track-width based on intersection points
    tw_left, tw_right = get_track_width(input_coord=output_rl,
                                        intersects_left=intersects_left,
                                        intersects_right=intersects_right)

    return output_rl, v_normal, tw_left, tw_right


def get_distance(input_coords: np.ndarray) -> tuple:
    """
    Calculates distances between earch point in 'input_coords' and returns a array of these distances as well as an
    cumulative row, starting at 0.0.

    :param input_coords:        input coordinates (columns holding x and y coordinates)
    :returns (distances,        distances between the 'input_coords' in m
              distances_sum)    cumulative sequence of the 'distances' starting at 0.0
    """

    # Get euclidean distances
    distances = np.sqrt(np.sum(np.power(np.diff(input_coords, axis=0), 2), axis=1))

    # Cumulative sum of the distance vector
    distances_sum = np.append([[0.0]], np.cumsum(distances))

    return distances, distances_sum


def interpol_new_points(input_coord: np.ndarray,
                        distances: np.ndarray,
                        track_res: float) -> np.ndarray:
    """
    Interpolate new points to the given track resolution.

    :param input_coord:     input coordinates (columns holding x and y coordinates)
    :param distances:       distances between the coordinates 'input_coords'
    :param track_res:       desired track resolution in m
    :returns coords_out:    returned (interpolated) coordinates (columns holding x and y coordinates)
    """

    # -- step 1 - interpolate points with very tight spacing -----------------------------------------------------------
    coords_out = np.zeros((1, 2))
    dist_limit = 0.005  # maximum distances between two coordinates
    v_length = len(input_coord[:, 0])
    for row in range(v_length - 1):
        p1 = input_coord[row, :]  # Get both points from the input
        p2 = input_coord[row + 1, :]
        if distances[row] < dist_limit:
            # if the dist is smaller than the limit --> no interpol needed
            interpl_points = np.array([p1])
            coords_out = np.append(coords_out, interpl_points, axis=0)
        else:
            # if distance larger insert points to reduce the distance
            num_new_points = math.ceil(distances[row] / dist_limit)
            interpl_points = np.linspace(p1, p2, num=num_new_points, endpoint=False)
            coords_out = np.append(coords_out, interpl_points, axis=0)

    # add last end-point and remove first point (initialized to zero)
    coords_out = np.append(coords_out, np.array([input_coord[-1, :]]), axis=0)
    coords_out = np.delete(coords_out, 0, axis=0)

    # -- step 2 - remove points in order to obtain desired step width --------------------------------------------------
    indices = np.zeros((len(coords_out), 1))
    row1 = 0
    row2 = 1
    while row1 + row2 < len(coords_out):
        delta_x = coords_out[row1, 0] - coords_out[row1 + row2, 0]
        delta_y = coords_out[row1, 1] - coords_out[row1 + row2, 1]
        distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
        if distance < track_res:
            indices[row1 + row2] = 1
            row2 += 1
        else:
            row1 = row1 + row2
            row2 = 1
    indices[-1] = 0
    idx = np.where(indices[:, 0] == 1)
    coords_out = np.delete(coords_out, idx, axis=0)

    return coords_out


def get_heading_angle(input_coord: np.ndarray):
    """
    Calculates heading vector (i.e. the angle of a single-track wheel following the trajectory).
    Value at certain index represents the heading angle at that given point (i.e. angle betw. 2 vectors in that point).

    :param input_coord:     input coordinates (columns holding x and y coordinates)
    :returns heading_angle: returned heading angle along the path
    """

    heading_angle = np.zeros((len(input_coord[:, 0]), 1))
    for row in range(1, len(input_coord) - 1):
        heading_angle[row, 0] = np.degrees(
            tph.normalize_psi.normalize_psi(tph.angle3pt.angle3pt(a=input_coord[row - 1, :],
                                                                  b=input_coord[row, :],
                                                                  c=input_coord[row + 1, :]) - math.pi)
        )

    return heading_angle


def get_angle_diff(heading_angle: np.ndarray):
    return np.append([[0]], np.diff(heading_angle))


def inpolygon(track_polygon: Polygon,
              input_coord: np.ndarray) -> bool:
    """
    Checks if all provided coordinates lie within the track bounds (polygon).

    :param track_polygon:   track bounds represented by a closed polygon
    :param input_coord:     input coordinates (columns holding x and y coordinates)
    :returns in_polygon:    'True' if all points lie within polygon, 'False' if any coordinate is outside
    """

    bool_check_list = np.zeros((len(input_coord[:, 0]), 1), dtype=bool)
    for row in range(len(input_coord)):
        p = Point(input_coord[row, :])

        # check if point is within polygon
        bool_check_list[row, :] = p.within(track_polygon)

    return np.all(bool_check_list)


def movmean_smoothing(input_coord: np.ndarray,
                      window_size: int) -> np.ndarray:
    """
    Smooth coordinate line based on moving average function. The first and last coordinates always stay the same.

    :param input_coord:     input coordinates (columns holding x and y coordinates)
    :param window_size:     number of points to be included for the moving average calculation
    :returns output_coord:  smoothed output coordinates (columns holding x and y coordinates)
    """

    input_save = input_coord
    # Get the distances of the input
    _, distances_sum = get_distance(input_coord)
    # Convert the numpy array into two DataFrames
    x = pd.DataFrame(input_coord[:, 0])
    y = pd.DataFrame(input_coord[:, 1])

    # Perform a moving average on the x and y column independent from each other with the given window size
    sx = x.rolling(window=window_size)
    sx = sx.mean()
    sy = y.rolling(window=window_size)
    sy = sy.mean()

    # Convert the columns back to numpy arrays
    sx = sx.to_numpy()
    sy = sy.to_numpy()

    # Create the the original 2D Array
    output_coord = np.hstack((sx, sy))
    # Replace the NaNs and last array with a smaller smoothing window to keep
    if window_size == 3:
        output_coord = np.delete(output_coord, [0, 1], axis=0)
        first_point = input_save[0, :]
        last_point = input_save[-1, :]
        output_coord = np.vstack((first_point, output_coord))
        output_coord = np.vstack((output_coord, last_point))
    if window_size == 9:
        output_coord = np.delete(output_coord, [0, 1, 2, 3, 4, 5, 6, 7], axis=0)
        append_top = input_save[0, :]
        append_bottom = input_save[-1, :]
        for i in range(3, 8, 2):
            p = np.mean(input_save[0:i, :], axis=0)
            append_top = np.vstack((append_top, p))
        for i in range(-3, -8, -2):
            p = np.mean(input_save[i:, :], axis=0)
            append_bottom = np.vstack((p, append_bottom))
        output_coord = np.vstack((append_top, output_coord))
        output_coord = np.vstack((output_coord, append_bottom))

    return output_coord


def find_sections(heading_angle: np.ndarray) -> tuple:
    """
    Find all sections where the angle changes its sign (i.e. left turn goes to right turn)

    :param heading_angle:           array of heading course
    :returns (section_row_indices,  indices indicating a sign change within the 'heading_angle' array
              section_angles,       angles at the positions of 'section_row_indices'
              delete_row_indices):  indices where no sign change within the 'heading_angle' array occurs
    """

    # if the sign changes change the array element to 1
    sign_change = np.zeros((len(heading_angle[:, 0]), 1))
    sign_change[0] = 1
    for row in range(len(heading_angle) - 1):
        if np.sign(heading_angle[row]) != np.sign(heading_angle[row + 1]) \
                and not abs(heading_angle[row]) < 1.0 and not abs(heading_angle[row + 1]) < 1.0:
            sign_change[row + 1] = 1
    sign_change[1:6] = 0
    sign_change[-6:] = 0

    # Get the indices of all zeros and delete them
    section_row_indices = np.where(sign_change == 1)[0]
    delete_row_indices = np.where(sign_change == 0)[0]
    section_angles = np.delete(heading_angle, delete_row_indices, axis=0)

    return section_row_indices, section_angles, delete_row_indices


def final_smooth(input_coord: np.ndarray,
                 heading_angle: np.ndarray) -> np.ndarray:
    """
    Smooth path coordinates until maximal angle alternation does not decline any further.

    :param input_coord:       input coordinates (columns holding x and y coordinates)
    :param heading_angle:     array of heading course
    :returns output_coord:    smoothed output coordinates (columns holding x and y coordinates)
    """

    section_angles_diff_in = max(abs(get_angle_diff(heading_angle)))
    section_angles_diff_out = 0.0
    while section_angles_diff_in > section_angles_diff_out:
        heading_angle = get_heading_angle(input_coord)
        section_angles_diff_in = max(abs(get_angle_diff(heading_angle)))
        input_coord = movmean_smoothing(input_coord, 9)
        heading_angle_out = get_heading_angle(input_coord)
        section_angles_diff_out = max(abs(get_angle_diff(heading_angle_out)))
        if section_angles_diff_in - section_angles_diff_out < 0.01:
            break

    return input_coord


def create_normal_vectors(input_coord: np.ndarray) -> np.ndarray:
    """
    Get the normal vectors, which extend orthogonal to each coordinate pair.

    :param input_coord:     input coordinates (columns holding x and y coordinates)
    :returns v_normal:      normal vectors (columns holding x and y components)
    """

    # extrapolate the direction last vector
    # Get the x and y direction of the last vector
    d_x = input_coord[-1, 0] - input_coord[-2, 0]
    d_y = input_coord[-1, 1] - input_coord[-2, 1]
    # The new coordinate is the last coordinate plus d_x and d_y
    last_vector = np.array((input_coord[-1, 0] + d_x, input_coord[-1, 1] + d_y))
    # Add the new coordinate to the array end
    output_rl = np.vstack((input_coord, last_vector))

    # create the orthogonal vectors left and right for each vector
    v_length = len(output_rl[:, 0])
    # create the output array with zeros
    v_normal = np.zeros((v_length - 1, 2))
    for row in range(v_length - 1):
        # Find both directional deltas
        d_x = output_rl[row + 1, 0] - output_rl[row, 0]
        d_y = output_rl[row + 1, 1] - output_rl[row, 1]
        # Norm of d_x and d_y
        norm = np.linalg.norm([d_x, d_y])
        # Normalise the new vector
        d_x_n = d_x / norm
        d_y_n = d_y / norm
        # Create normal vector left and right
        # Left: [-dy, +dx] / Right: [+dy, -dx]
        v_normal[row, :] = [d_y_n, -d_x_n]

    return v_normal


def get_intersect_points(track_polygon: Polygon,
                         input_coord: np.ndarray,
                         v_normal: np.ndarray) -> tuple:
    """
    Find the coordinates where the extended normal vectors cross the track boundaries.

    :param track_polygon:       track bounds represented by a closed polygon
    :param input_coord:         input coordinates (columns holding x and y coordinates)
    :param v_normal:            normal vectors (columns holding x and y components)
    :returns (intersects_left,  intersection coordinates to the left of input_coord (columns holding x and y coords)
              intersects_right) intersection coordinates to the right of input_coord (columns holding x and y coords)
    """

    # Extend normal vector to 50.0 meters (definitely hit borders) and add it to race-line
    v_normal_left = input_coord - v_normal * 50.0
    v_normal_right = input_coord + v_normal * 50.0

    # create the output array with zeros
    v_length = len(input_coord[:, 0])
    intersects_left = np.zeros((v_length, 2))
    intersects_right = np.zeros((v_length, 2))

    # Get the coordinates of the intersect of the polygon and each normal vector
    for row in range(v_length):
        # Left Vectors:
        # Create a Shapely Line with the end point of the normal vector and the reference line coordinates
        line_left = LineString([input_coord[row, :], v_normal_left[row, :]])
        # Get the coordinate where the new line crosses the track boundaries
        intersect_left_iter = track_polygon.intersection(line_left)
        # Get the coordinates out the Multipoint data type
        left_coords_dict = intersect_left_iter.__geo_interface__
        left_coords_values = left_coords_dict.get('coordinates')
        # The position of correct coordinate changes depending on if the normal vector crosses one or more boundaries
        # Check if resulting coordinates is a list of list or just a single coordinate
        if any(isinstance(el, tuple) for el in left_coords_values):
            # If multiple lines were returned
            if any(isinstance(el, tuple) for el in left_coords_values[0]):
                left_coords_values = left_coords_values[0]
            intersects_left[row, :] = np.array(left_coords_values[1])
        else:  # More than one intersection
            intersects_left[row, :] = np.array(left_coords_values)

        # Same procedure, but for the right side
        line_right = LineString([input_coord[row, :], v_normal_right[row, :]])
        intersect_right_iter = track_polygon.intersection(line_right)
        right_coords_dict = intersect_right_iter.__geo_interface__
        right_coords_values = right_coords_dict.get('coordinates')
        if any(isinstance(el, tuple) for el in right_coords_values):
            # If multiple lines were returned
            if any(isinstance(el, tuple) for el in right_coords_values[0]):
                right_coords_values = right_coords_values[0]
            intersects_right[row, :] = np.array(right_coords_values[1])
        else:
            intersects_right[row, :] = np.array(right_coords_values)

    return intersects_left, intersects_right


def get_track_width(input_coord: np.ndarray,
                    intersects_left: np.ndarray,
                    intersects_right: np.ndarray) -> tuple:
    """
    Calculate the distances from input coordinates and the intersects with the track bounds left and right (track-width
    at corresponding point).

    :param input_coord:          input coordinates (columns holding x and y coordinates)
    :param intersects_left       intersection coordinates to the left of input_coord (columns holding x and y coords)
    :param intersects_right      intersection coordinates to the right of input_coord (columns holding x and y coords)
    :returns (track_width_left,  distances between input coordinates and left intersection coordinates
              track_width_right) distances between input coordinates and right intersection coordinates
    """

    track_width_left = np.sqrt(np.sum(np.power(input_coord - intersects_left, 2), axis=1))[np.newaxis].T
    track_width_right = np.sqrt(np.sum(np.power(input_coord - intersects_right, 2), axis=1))[np.newaxis].T

    return track_width_left, track_width_right


def normal_vectors_intersections(intersects_left: np.ndarray,
                                 intersects_right: np.ndarray) -> tuple:
    """
    Check if neighboring normal-vectors cross each other within the track-area.

    :param intersects_left          intersection coordinates to the left of ref. line (columns holding x and y coords)
    :param intersects_right         intersection coordinates to the right of ref. line (columns holding x and y coords)
    :returns (rows_with_intersect,  (row) indices of valid normal-vectors
              delete_row_indices)   (row) indices of invalid normal-vectors
    """

    # Array with zeros to improve speed
    v_length = len(intersects_left[:, 0])
    bool_intersect = np.zeros((v_length, 1))

    for row in range(v_length - 1):
        # Create both Shapely Lines from the input given (Both normal vector intersections, left and right)
        line1 = LineString([intersects_left[row, :], intersects_right[row, :]])
        line2 = LineString([intersects_left[row + 1, :], intersects_right[row + 1, :]])
        # Where is the intersect --> if no intersect output is empty
        coord_intersect = line1.intersection(line2)
        # if returning value is not empty change the 0 to a 1
        if not coord_intersect.is_empty:
            bool_intersect[row] = 1
            bool_intersect[row + 1] = 1

    # Get the indies of both valid and invalid lines
    delete_row_indices = np.where(bool_intersect == 0)
    delete_row_indices = delete_row_indices[0]
    if np.any(bool_intersect):
        rows_with_intersect = np.where(bool_intersect == 1)
        rows_with_intersect = rows_with_intersect[0]
    else:
        rows_with_intersect = np.zeros((1, 1), dtype=bool)

    return rows_with_intersect, delete_row_indices


def smooth_sections(input_coord: np.ndarray,
                    section_row_indices: list,
                    track_polygon: Polygon) -> np.ndarray:
    """
    Smooth coordinate line within sections.

    :param input_coord:         input coordinates (columns holding x and y coordinates)
    :param section_row_indices  indices indicating a sign change within the 'heading_angle' array
    :param track_polygon:       track bounds represented by a closed polygon
    :returns coords_out:        returned (smoothed) coordinates (columns holding x and y coordinates)
    """

    num_sections = len(section_row_indices)  # Get the number of sections within the track
    length_input = len(input_coord)  # Get the number of input reference line coordinates
    output_coord = np.zeros((length_input, 2))  # Make all output arrays the same size with zeros
    section_row_indices = np.hstack((section_row_indices, [length_input]))

    # print('sections started')

    # For each section, smooth until no intersects exist
    for row in range(num_sections):
        # Extract the section and its previous and next section too
        if row == 0:
            sec_start = section_row_indices[0]
            sec_end = section_row_indices[1]
        elif row == num_sections - 1:
            sec_start = section_row_indices[-2]
            sec_end = section_row_indices[-1]
        else:
            sec_start = section_row_indices[row]
            sec_end = section_row_indices[row + 1]

        section = input_coord[sec_start:sec_end, :]

        # Are there any intersects in the section
        v_normal = create_normal_vectors(section)
        intersects_left, intersects_right = get_intersect_points(track_polygon, section, v_normal)
        rows_with_intersect = normal_vectors_intersections(intersects_left, intersects_right)[0]
        section_angles = get_heading_angle(section)
        sign_of_section = np.sign(section_angles[1:-1])
        angle_bool = np.all(sign_of_section >= 0) or np.all(sign_of_section <= 0)

        # If no intersects exist from the beginning, continue to the next section
        if rows_with_intersect.dtype == bool and angle_bool is True:
            output_coord[sec_start:sec_end, :] = section
            # print('section ' + str(row + 1) + ' of ' + str(num_sections) + ' finished')
            continue

        # Tier 1: remove all intersects from the section
        while rows_with_intersect.dtype != bool:
            section = movmean_smoothing(section, 9)
            v_normal = create_normal_vectors(section)
            intersects_left, intersects_right = get_intersect_points(track_polygon, section, v_normal)
            rows_with_intersect, _ = normal_vectors_intersections(intersects_left, intersects_right)
        output_coord[sec_start:sec_end, :] = section

    return output_coord


# -- main --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    scenario_path = (os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
                     + "/sample_files/scenario_n_vehicle/modena_overtake_tight.scn")

    import scenario_testing_tools
    b_l, b_r = scenario_testing_tools.get_scene_track.get_scene_track(file_path=scenario_path)

    refline, normvec, tw_l, tw_r = generate_refline(bound_l=b_l, bound_r=b_r)

    bound_l = refline - normvec * tw_l
    bound_r = refline + normvec * tw_r
