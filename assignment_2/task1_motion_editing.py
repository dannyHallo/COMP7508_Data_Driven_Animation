import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from file_io import BVHMotion
from Viewer.controller import SimpleViewer
from Viewer.viewer import ShowBVHUpdate

'''
Find the betweening data between left_data and right_data
Parameters:
    - left_data: the left data
    - right_data: the right data
    - t: the number of betweening frames
    - method: the interpolation method, 'linear' or 'slerp'
Return: An array for the betweening data, 
        if return_first_key=True, then including the known left one, 
        for example, if t=9, then the result should be 9 frames including the left one
'''


def interpolation(left_data, right_data, t, method='linear', return_first_key=True):
    res = [left_data] if return_first_key else []
    '''
    TODO: Implement following functions
    Hints:
        1. The shape of data is (num_joints, 3) for position and (num_joints, 4) for rotation
        2. The rotation data is in quaternion format
        3. We use linear interpolation for position and slerp for rotation
            * The linear interpolation can be obtained by the following formula:
                data_between = left_data + (right_data - left_data) * (i / t)
        4. We use scipy.spatial.transform.Slerp to do slerp interpolation (https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html)
            * The Slerp can be obtained by the following code:
                key_rots = R.from_quat([q1, q2])
                key_frames= [0, 1]
                slerp = Slerp(key_times, key_rots)
                new_key_frames = np.linspace(0, 1, t+1))
                interp_rots = slerp(new_key_frames)
            * The slerp in scipy doesn't support quaternion with shape (num_joints, 4), 
              so you need to take the quaternion for each joint in a loop with joint_num,
              like left_data[joint_idx], then combine them after the loop.
            * Don't take the last frame of slerp, because it is the same as the right_data
    '''
    ########## Code Start ############
    if method == 'linear':
        for i in range(1, t + 1):
            # Ensures the last interpolated frame is before the right_data
            alpha = i / (t + 1)
            interpolated_frame = left_data + (right_data - left_data) * alpha
            res.append(interpolated_frame)
        return res
    elif method == 'slerp':
        num_joints = left_data.shape[0]
        interpolated_rotations = [np.zeros_like(left_data) for _ in range(t)]

        for j in range(num_joints):
            # Create Rotation objects for the current joint
            key_rots = R.from_quat([left_data[j], right_data[j]])
            key_times = [0, 1]

            # Initialize Slerp with key rotations
            slerp = Slerp(key_times, key_rots)

            # Generate new times for interpolation
            # Exclude the last frame to avoid duplication
            new_times = np.linspace(0, 1, t + 1)[:-1]

            # Perform Slerp interpolation
            interp_rots = slerp(new_times).as_quat()

            # Assign interpolated quaternions to each frame
            for i in range(t):
                interpolated_rotations[i][j] = interp_rots[i]
        res.extend(interpolated_rotations)

        return res
    ########## Code End ############


def part1_key_framing(viewer, time_step, target_step):
    motion = BVHMotion('data/motion_walking.bvh')

    motio_length = motion.local_joint_positions.shape[0]
    keyframes = np.arange(0, motio_length, time_step)

    new_motion_local_positions, new_motion_local_rotations = [], []

    previous_frame_idx = 0
    for current_frame_idx in keyframes[1:]:
        between_local_pos = interpolation(motion.local_joint_positions[previous_frame_idx],
                                          motion.local_joint_positions[current_frame_idx],
                                          target_step - 1, 'linear')
        between_local_rot = interpolation(motion.local_joint_rotations[previous_frame_idx],
                                          motion.local_joint_rotations[current_frame_idx],
                                          target_step - 1, 'slerp')
        new_motion_local_positions.append(between_local_pos)
        new_motion_local_rotations.append(between_local_rot)
        previous_frame_idx = current_frame_idx

    res_motion = motion.raw_copy()
    res_motion.local_joint_positions = np.concatenate(
        new_motion_local_positions)
    res_motion.local_joint_rotations = np.concatenate(
        new_motion_local_rotations)

    translation, orientation = res_motion.batch_forward_kinematics()
    task = ShowBVHUpdate(viewer, res_motion.joint_name,
                         translation, orientation)
    viewer.addTask(task.update)


'''
Combine two different motions into one motion
Parameters:
    - motion1: the first motion
    - motion2: the second motion
    - last_frame_index: the last frame index of the first motion
    - start_frame_index: the start frame index of the second motion
    - between_frames: the number of frames between the two motions
    - searching_frames: the number of frames for searching the closest frame
    - method: the interpolation method, 'interpolation' or 'inertialization'
'''


def concatenate_two_motions(motion1, motion2, last_frame_index, start_frame_index, between_frames, searching_frames=20, method='interpolation'):
    '''
    TODO: Implement following functions
    Hints:
        1. We should operation on the local joint positions and rotations
            motion.local_joint_positions: (num_frames, num_joints, 3)
            motion.local_joint_rotations: (num_frames, num_joints, 4)
        2. There are five steps in the concatenation:
            i) get the searching windows for motion 1 and motion 2 
                win_1 = motion1.local_joint_rotations[last_frame_index - searching_frames:last_frame_index + searching_frames]
                win_2 = motion2.local_joint_rotations[max(0, start_frame_index - searching_frames):start_frame_index + searching_frames]
            ii) find the closest frame in motion 1 searching window and motion 2 searching window
                * You can use similarity matrix in DTW (Dynamic Time Warping) to find the closest frame (motion editing slides)
                * sim_matrix is a matrix with shape (win_1.shape[0], win_2.shape[0])
                * sim_matrix[i, j] = np.linalg.norm(search_source[i] - search_target[j])
                * Find the minimum value in sim_matrix and get the corresponding index i and j
            iii) the i and j is the index for the searching window, so convert it to the index in the original motion sequence
            iv) we have the pose in motion_1(real_i) and motion_2(real_j), then we can do the interpolation
                * The interpolation should be done for the *positions* and *rotations* both
                * (Shifting) You must align the root positions of motion_2(real_j) to the root positions of motion_1(real_i)
                # The shifting is done by updating variable: motion2.local_joint_positions = motion2.local_joint_positions - ?
            v) combine the motion1, betweening, motion2 into one motion (have been provided)
        3. You can get all marks if you done above steps correctly, but bonus will be given if you can do any one of them:
            (bonus) There are N between_frames, but the root position in these frames are not considered
            (bonus) The velocity of two motions are not the same, it can give different weight to the two motions interpolation
            (bonus) The inertialization method provides the best results ref:https://theorangeduck.com/page/spring-roll-call#inertialization
            (bonus) Any way to produce more smooth and natural transitions

    Useful functions:
        1. The differece between two vectors: np.linalg.norm(a - b)
        2. The index of minimal value in a matrix: np.argmin(dtw)
        3. local_joint_rotations = motion.local_joint_rotations
        4. local_joint_positions = motion.local_joint_positions
        5. Visualize the dtw matrix:
            import matplotlib.pyplot as plt
            plt.imshow(dtw)
            plt.show() # You can use plt.savefig('dtw.png') to save the image
    '''

    ########## Code Start ############

    # helper functions
    def fast_negexp(x):
        # Fast approximation of exp(-x)
        return 1.0 / (1.0 + x + 0.48 * x**2 + 0.235 * x**3)

    def decay_spring_damper_exact(offset, velocity, halflife, dt, frequency, eps=1e-5):
        '''
        Applies an exact spring-damper decay to the offset and velocity.
        '''
        y = (4.0 * np.log(2)) / (halflife + eps)
        w = 2.0 * np.pi * frequency
        # Update equations based on the spring-damper system
        new_offset = fast_negexp(
            # Decaying to zero
            y * dt) * offset + (1 - fast_negexp(y * dt)) * 0.0
        new_velocity = fast_negexp(y * dt) * velocity - y * new_offset
        return new_offset, new_velocity

    # Get the searching windows for motion 1 and motion 2
    num_frames_1 = motion1.local_joint_rotations.shape[0]
    num_frames_2 = motion2.local_joint_rotations.shape[0]

    start_idx_1 = max(0, last_frame_index - searching_frames)
    end_idx_1 = min(num_frames_1, last_frame_index + searching_frames)

    start_idx_2 = max(0, start_frame_index - searching_frames)
    end_idx_2 = min(num_frames_2, start_frame_index + searching_frames)

    # Shape: (window_size_1, num_joints, 4)
    win_1 = motion1.local_joint_rotations[start_idx_1:end_idx_1]
    # Shape: (window_size_2, num_joints, 4)
    win_2 = motion2.local_joint_rotations[start_idx_2:end_idx_2]

    # Compute the similarity matrix between the two windows
    # Flatten the rotations for easier computation
    # Shape: (window_size_1, num_joints * 4)
    win_1_flat = win_1.reshape(win_1.shape[0], -1)
    # Shape: (window_size_2, num_joints * 4)
    win_2_flat = win_2.reshape(win_2.shape[0], -1)

    # Compute the Euclidean distances between each pair of frames
    # Shape: (window_size_1, window_size_2, num_joints * 4)
    diff = win_1_flat[:, np.newaxis, :] - win_2_flat[np.newaxis, :, :]
    # Shape: (window_size_1, window_size_2)
    sim_matrix = np.linalg.norm(diff, axis=2)

    # Find the closest frames in the searching windows
    min_idx = np.unravel_index(np.argmin(sim_matrix), sim_matrix.shape)
    i, j = min_idx  # Indices in the searching windows

    # Convert to indices in the original motion sequences
    real_i = start_idx_1 + i
    real_j = start_idx_2 + j

    print(f'Closest frames: {real_i}, {real_j}')

    starting_positions_motion1 = motion1.local_joint_positions[real_i]
    starting_positions_motion2 = motion2.local_joint_positions[real_j]

    # apply delta to motion2
    motion2.local_joint_positions += starting_positions_motion1 - \
        starting_positions_motion2

    # Now, we can interpolate between motion1's frame at real_i and motion2's adjusted position at index 0

    if method == 'interpolation':
        # Perform interpolation between the matching frames
        between_local_pos = interpolation(
            motion1.local_joint_positions[real_i],
            motion2.local_joint_positions[real_j],
            between_frames - 1,
            method='linear'
        )
        between_local_rot = interpolation(
            motion1.local_joint_rotations[real_i],
            motion2.local_joint_rotations[real_j],
            between_frames - 1,
            method='slerp'
        )
    else:
        # Perform inertialization
        # Initialize offset and velocity for inertialization (per joint)
        num_joints = motion1.local_joint_positions.shape[1]
        offset_positions = motion1.local_joint_positions[real_i].copy()
        velocity_positions = np.zeros_like(offset_positions)

        offset_rotations = motion1.local_joint_rotations[real_i].copy()
        velocity_rotations = np.zeros_like(offset_rotations)

        between_local_pos = []
        # between_local_rot = []

        halflife = 0.1
        frequency = 5.0

        for f in range(between_frames):
            # Step iv.1: Calculate the current offset between motion1 and motion2
            current_pose_motion1 = motion1.local_joint_positions[real_i]
            current_pose_motion2 = motion2.local_joint_positions[real_j]

            # Calculate the difference
            current_offset = current_pose_motion1 - current_pose_motion2

            # Apply spring decay to the offset
            decayed_offset, decayed_velocity = decay_spring_damper_exact(
                current_offset, velocity_positions, halflife, dt=1.0/between_frames, frequency=frequency
            )

            # Update the velocity
            velocity_positions = decayed_velocity

            # Apply the decayed offset to motion2's current frame
            interpolated_pos = current_pose_motion2 + decayed_offset

            between_local_pos.append(interpolated_pos)

            # Similarly handle rotations using SLERP (simplified)
            # For rotations, we'll use SLERP between motion1 and motion2 with a decayed alpha
            # This is a simplification; for true inertialization, you'd apply a spring to quaternion space
            # interp_rot = Slerp(
            #     motion1.local_joint_rotations[real_i], motion2.local_joint_positions[real_j], (f+1)/between_frames)

            # between_local_rot.append(interp_rot)

            between_local_rot = interpolation(
                motion1.local_joint_rotations[real_i],
                motion2.local_joint_rotations[real_j],
                between_frames - 1,
                method='slerp'
            )

    # Convert the lists to numpy arrays
    # Shape: (between_frames - 1, num_joints, 3)
    between_local_pos = np.stack(between_local_pos)
    # Shape: (between_frames - 1, num_joints, 4)
    between_local_rot = np.stack(between_local_rot)

    ########## Code End ############

    res = motion1.raw_copy()
    res.local_joint_positions = np.concatenate([motion1.local_joint_positions[:real_i],
                                                between_local_pos,
                                                motion2.local_joint_positions[real_j:]],
                                               axis=0)
    res.local_joint_rotations = np.concatenate([motion1.local_joint_rotations[:real_i],
                                                between_local_rot,
                                                motion2.local_joint_rotations[real_j:]],
                                               axis=0)
    return res


def part2_concatenate(viewer, between_frames, do_interp=True):
    walk_forward = BVHMotion('data/motion_walking.bvh')
    run_forward = BVHMotion('data/motion_running.bvh')
    run_forward.adjust_joint_name(walk_forward.joint_name)

    last_frame_index = 40
    start_frame_index = 0

    if do_interp:
        method = 'interpolation'
        # method = 'inertialization'
        motion = concatenate_two_motions(
            walk_forward, run_forward, last_frame_index, start_frame_index, between_frames, method=method)
    else:
        motion = walk_forward.raw_copy()
        motion.local_joint_positions = np.concatenate([walk_forward.local_joint_positions[:last_frame_index],
                                                       run_forward.local_joint_positions[start_frame_index:]],
                                                      axis=0)
        motion.local_joint_rotations = np.concatenate([walk_forward.local_joint_rotations[:last_frame_index],
                                                       run_forward.local_joint_rotations[start_frame_index:]],
                                                      axis=0)

    translation, orientation = motion.batch_forward_kinematics()
    task = ShowBVHUpdate(viewer, motion.joint_name, translation, orientation)
    viewer.addTask(task.update)
    pass


def main():
    viewer = SimpleViewer()

    # part1_key_framing(viewer, 10, 10)
    # part1_key_framing(viewer, 10, 5)
    # part1_key_framing(viewer, 10, 20)
    # part1_key_framing(viewer, 10, 30)
    # part2_concatenate(viewer, between_frames=8, do_interp=False)
    part2_concatenate(viewer, between_frames=16)
    viewer.run()


if __name__ == '__main__':
    main()
