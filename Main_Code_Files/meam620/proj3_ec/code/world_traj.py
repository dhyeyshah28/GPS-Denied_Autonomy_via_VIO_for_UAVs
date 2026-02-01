import numpy as np
from scipy.interpolate import CubicSpline
from proj3.code.graph_search import graph_search
from proj3_ec.util.occupancy_map import OccupancyMap

class WorldTraj(object):
    """
    Generates a smooth trajectory for local planning with replanning capabilities.
    """
    def __init__(self, world, start, goal, local=False):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        Parameters:
            world,       World object representing the environment obstacles
            start,       xyz position in meters, shape=(3,)
            goal,        xyz position in meters, shape=(3,)
        """
        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.2, 0.2, 0.2])  # Increased precision
        self.margin = 0.5
        self.min_margin = 0.2  # Safer minimum margin
        self.margin_step = 0.05

        # Don't change this part
        self.local = local
        self.local_occ_map = OccupancyMap(world, self.resolution, self.margin)

        # The local map range is 5 m, usually the local planning range could be 1.5 * map range
        self.planning_horizon = 7.5 # m
        self.stopping_distance = 0.5 # m
        self.step_t = 0.02 # s
        self.no_replan_thresh = 2.0 # m

        # Switch between replanning mode or execute trajectory mode.
        self.exec_traj = True
        # The collision checking frequency is usually higher than replanning frequency.
        self.replan_num = 0
        self.t_check_traj = 0.1  # More frequent checks

        # Generate init path
        self.global_goal = goal
        self.start = start
        self.traj_start_time = 0.0

        # Initialize trajectory attributes to handle failed planning
        self.traj_duration = 0.0
        self.points = np.array([start])  # Default to a single point (start)
        self.splines = None

        self.local_occ_map.update(self.start)
        self.plan_traj(self.start, self.crop_local_goal(start))

    def find_nearest_free_center(self, position):
        """
        Find the nearest collision-free voxel center to the given position.
        """
        index = self.local_occ_map.metric_to_index(position)
        if not self.local_occ_map.is_occupied_index(index):
            return self.local_occ_map.index_to_metric_center(index)
        margin_voxels = 3  # Search within 3 voxels
        for dx in range(-margin_voxels, margin_voxels + 1):
            for dy in range(-margin_voxels, margin_voxels + 1):
                for dz in range(-margin_voxels, margin_voxels + 1):
                    test_index = (index[0] + dx, index[1] + dy, index[2] + dz)
                    if (self.local_occ_map.is_valid_index(test_index) and
                        not self.local_occ_map.is_occupied_index(test_index)):
                        return self.local_occ_map.index_to_metric_center(test_index)
        return None

    def check_traj_collision(self, cur_t):
        """
        Given current time, return the collision time

        Input:
          cur_t, absolute time or relative time, s 
        """
        check_t = cur_t
      
        # If no valid trajectory exists, return -1 (no collision possible)
        if not hasattr(self, 'splines') or self.splines is None or self.traj_duration <= 0:
            return -1

        while check_t < self.traj_start_time + self.traj_duration:
            check_t += self.step_t
            check_pt = self.get_traj_pos(check_t)
            if self.local_occ_map.is_occupied_metric(check_pt):
                print(f"Collision detected at t={check_t:.3f}, pos={check_pt}")
                return check_t

        check_t = -1
        return check_t

    def get_traj_pos(self, t):
        """
        Given the present time, return the desired position.

        Inputs
            t,   absolute time or relative time, s 
        Outputs
            x,   position, m
        """
        # Adjust time to relative trajectory time
        t_rel = t - self.traj_start_time

        if not hasattr(self, 'splines') or self.splines is None or self.traj_duration <= 0:
            print("No valid trajectory, returning start position")
            return self.start

        if t_rel <= 0:
            return self.points[0]
        elif t_rel >= self.traj_duration:
            return self.points[-1]
        else:
            pos = np.array([self.splines[axis](t_rel) for axis in ['x', 'y', 'z']])
            # Check if interpolated point is occupied
            if self.local_occ_map.is_occupied_metric(pos):
                print(f"Warning: Trajectory point {pos} at t={t_rel} is occupied!")
            return pos

    def replan(self, cur_state, t):
        """
        Example framework for local planner. It can switch between replanning mode 
        or execute trajectory mode. You can use or improve it.
        
        Inputs
            cur_state,      a dict describing the state history with keys
                            x, position, m, shape=(N,3)
                            v, linear velocity, m/s, shape=(N,3)
                            q, quaternion [i,j,k,w], shape=(N,4)
                            w, angular velocity, rad/s, shape=(N,3)
            t,              absolute time, s 
        """
        cur_state = cur_state['x']
        # Update map with new center point
        self.local_occ_map.update(cur_state)

        self.replan_num += 1
        # Check if replanning because of potential collision
        check_t = self.check_traj_collision(t)
        if self.replan_num < 20:
            if check_t < 0 or check_t > t + 0.2 * self.traj_duration:  # Earlier replanning
                print("No need for replanning, the check_t is : ", check_t)
                return
        self.replan_num = 0

        if self.exec_traj:  # exec mode
            # 1. Reaching end, no plan threshold
            if np.linalg.norm(cur_state - self.global_goal) < self.stopping_distance: 
                print("Reaching end ...")
                return
    
            # 2. Check no planning thresh
            if np.linalg.norm(self.start - cur_state) < self.no_replan_thresh:
                return

            self.exec_traj = False  # Transfer to replanning mode

        else:  # replanning mode
            if t < self.traj_start_time + self.traj_duration:  # Redo planning
                cur_state = self.get_traj_pos(t)

            self.start = cur_state
            self.traj_start_time = t
            if self.plan_traj(self.start, self.crop_local_goal(cur_state)):
                self.exec_traj = True
            else:
                print("Replanning failed, stopping at current position")
                self.points = np.array([cur_state])
                self.traj_duration = 0
                self.splines = None
            
        return

    def crop_local_goal(self, start):
        """
        Given local start, get a straight line position as the cropped local goal 
        and end up with the global goal
        
        Inputs
            start, xyz position in meters, shape=(3,)        
        
        Outputs
            goal,  xyz position in meters, shape=(3,)  
        """
        dist = np.linalg.norm(self.global_goal - start)
        if dist <= self.planning_horizon:
            print("reaching global goal!")
            goal = self.global_goal
        else:
            goal = start + (self.planning_horizon / dist) * (self.global_goal - start)
        
        return goal

    def plan_traj(self, start, goal):
        """
        Given local start and goal, update the trajectory

        Inputs
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)
        """
        print(f"Planning trajectory from {start} to {goal}")

        # Snap start and goal to nearest collision-free voxel centers
        start_center = self.find_nearest_free_center(start)
        if start_center is None:
            print("No collision-free start point found")
            return False
        goal_center = self.find_nearest_free_center(goal)
        if goal_center is None:
            print("No collision-free goal point found")
            return False
        print(f"Using start_center: {start_center}, goal_center: {goal_center}")

        # Compute dense path using A* with adjustable margin
        margin = self.margin
        self.local_occ_map = OccupancyMap(self.local_occ_map.world, self.resolution, margin)
        self.local_occ_map.update(start_center)
        self.path, nodes_expanded = graph_search(
            self.local_occ_map.world,  # Pass the World object
            self.resolution,
            margin,
            start_center,
            goal_center,
            astar=True
        )

        # If no path is found, decrease margin and retry
        if self.path is None or len(self.path) < 2:
            print(f"No valid path found with margin {margin:.2f}, decreasing margin...")
            while margin > self.min_margin:
                margin -= self.margin_step
                print(f"Trying margin: {margin:.2f}")
                self.local_occ_map = OccupancyMap(self.local_occ_map.world, self.resolution, margin)
                self.local_occ_map.update(start_center)
                self.path, nodes_expanded = graph_search(
                    self.local_occ_map.world,
                    self.resolution,
                    margin,
                    start_center,
                    goal_center,
                    astar=True
                )
                if self.path is not None and len(self.path) >= 2:
                    print(f"Path found with margin {margin:.2f}")
                    break
            if self.path is None or len(self.path) < 2:
                print("No valid path found even after adjustments!")
                return False

        print(f"Graph search returned path with {len(self.path)} points: {[tuple(p) for p in self.path]}")

        # Verify path points are collision-free
        for i, pt in enumerate(self.path):
            if self.local_occ_map.is_occupied_metric(pt):
                print(f"Graph search path contains occupied point at index {i}: {pt}")
                return False

        # Extract sparse waypoints using RDP and collinearity check
        sparse_path = self._rdp_sparsify(self.path, epsilon=0.26)
        self.points = self._remove_collinear_points(sparse_path)

        # Verify and adjust waypoints after RDP
        adjusted_points = []
        for i, pt in enumerate(self.points):
            if self.local_occ_map.is_occupied_metric(pt):
                print(f"Waypoint at index {i} occupied: {pt}, adjusting")
                new_pt = self.find_nearest_free_center(pt)
                if new_pt is None:
                    print(f"Failed to adjust waypoint at index {i}: {pt}")
                    return False
                print(f"Adjusted waypoint to: {new_pt}")
                adjusted_points.append(new_pt)
            else:
                adjusted_points.append(pt)
        self.points = np.array(adjusted_points)
        print(f"After adjustment: {len(self.points)} waypoints: {[tuple(p) for p in self.points]}")

        # Final verification
        for i, pt in enumerate(self.points):
            if self.local_occ_map.is_occupied_metric(pt):
                print(f"Final waypoint still occupied at index {i}: {pt}")
                return False

        if len(self.points) < 2:
            print("Not enough waypoints to generate trajectory")
            return False

        # Compute segment times using adaptive velocity
        self.segment_times = self._compute_segment_times()

        # Generate smooth cubic spline trajectory
        self._compute_smooth_trajectory()

        # Update trajectory duration
        self.traj_duration = self.segment_times[-1]

        print("Trajectory verified as collision-free")
        return True

    def _rdp_sparsify(self, path, epsilon=0.26):
        """
        Implements the Ramer-Douglas-Peucker algorithm to remove redundant points while preserving curvature
        """
        def perpendicular_distance(pt, start, end):
            if np.allclose(start, end):
                return np.linalg.norm(pt - start)
            return np.linalg.norm(np.cross(end - start, start - pt)) / np.linalg.norm(end - start)

        def rdp_recursive(points, epsilon):
            if len(points) < 3:
                return points
            start, end = points[0], points[-1]
            dmax, index = max((perpendicular_distance(points[i], start, end), i) for i in range(1, len(points) - 1))
            if dmax > epsilon:
                left = rdp_recursive(points[:index + 1], epsilon)
                right = rdp_recursive(points[index:], epsilon)
                return np.vstack((left[:-1], right))
            else:
                return np.array([start, end])

        return rdp_recursive(path, epsilon)

    def _remove_collinear_points(self, path):
        """
        Remove points that lie on a straight line using a collinearity check
        """
        waypoints = [path[0]]
        for i in range(1, len(path) - 1):
            prev, curr, next_ = waypoints[-1], path[i], path[i + 1]
            if not np.allclose(np.cross(curr - prev, next_ - curr), 0, atol=1e-3):
                waypoints.append(curr)
        waypoints.append(path[-1])
        return np.array(waypoints)

    def _compute_segment_times(self):
        """
        Compute time allocation for each segment based on adaptive velocity selection
        """
        fast_vel = 4.2  # max speed
        moderate_vel = 3.2  # moderate speed
        slow_vel = 2.2  # slowest speed

        times = [0]
        for i in range(1, len(self.points)):
            dist = np.linalg.norm(self.points[i] - self.points[i - 1])
            if dist > 3.2:
                velocity = fast_vel
            elif dist > 1.2:
                velocity = moderate_vel
            else:
                velocity = slow_vel
            times.append(times[-1] + dist / velocity)
        return np.array(times)

    def _compute_smooth_trajectory(self):
        """
        Compute a smooth trajectory through waypoints using cubic spline
        """
        self.splines = {axis: CubicSpline(self.segment_times, self.points[:, i], bc_type='clamped')
                        for i, axis in enumerate(['x', 'y', 'z'])}

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, absolute time, s 
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        t_rel = t - self.traj_start_time

        if not hasattr(self, 'splines') or self.splines is None or self.traj_duration <= 0:
            pos = self.start
            vel = acc = jerk = snap = np.zeros(3)
        elif t_rel <= 0:
            pos = self.points[0]
            vel = acc = jerk = snap = np.zeros(3)
        elif t_rel >= self.traj_duration:
            pos = self.points[-1]
            vel = acc = jerk = snap = np.zeros(3)
        else:
            pos = np.array([self.splines[axis](t_rel) for axis in ['x', 'y', 'z']])
            vel = np.array([self.splines[axis].derivative()(t_rel) for axis in ['x', 'y', 'z']])
            acc = np.array([self.splines[axis].derivative(nu=2)(t_rel) for axis in ['x', 'y', 'z']])
            jerk = np.array([self.splines[axis].derivative(nu=3)(t_rel) for axis in ['x', 'y', 'z']])
            snap = np.array([self.splines[axis].derivative(nu=4)(t_rel) for axis in ['x', 'y', 'z']])

        # Keep yaw aligned with velocity direction
        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 1e-3 else 0
        yaw_dot = 0

        flat_output = {
            'x': pos,
            'x_dot': vel,
            'x_ddot': acc,
            'x_dddot': jerk,
            'x_ddddot': snap,
            'yaw': yaw,
            'yaw_dot': yaw_dot
        }
        return flat_output