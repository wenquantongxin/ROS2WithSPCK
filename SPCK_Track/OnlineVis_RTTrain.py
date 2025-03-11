import socket
import struct
import time
import numpy as np
import os
import math
import threading
import queue
import ctypes
import sys
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

# Global variables: shared data queue
data_queue = queue.Queue(maxsize=10)  # Limit queue size to avoid memory overflow
latest_data = None  # Latest data frame
previous_data = None  # Previous data frame for interpolation
data_ready = threading.Event()  # Data ready signal
running = True  # Thread control flag
frame_time = 0.0  # Current interpolated frame time
last_update_time = 0.0  # Time of last frame update

# Animation control
TARGET_FPS = 60  # Target frames per second
FRAME_TIME = 1.0 / TARGET_FPS  # Time per frame in seconds
interpolation_enabled = True  # Enable frame interpolation
animation_paused = False  # Animation pause control
use_received_time = True  # Use simulation time from UDP packets

# =========================== 1) UDP Receiver Thread ===========================
class UDPReceiverThread(threading.Thread):
    def __init__(self, ip="0.0.0.0", port=10088, port_retry=True, max_retries=10):
        threading.Thread.__init__(self)
        self.daemon = True  # Set as daemon thread, will terminate when main thread exits
        
        # UDP configuration
        self.UDP_IP = ip
        self.UDP_PORT = port
        self.port_retry = port_retry
        self.max_retries = max_retries
        
        # Data format configuration
        self.EXPECTED_LEN = 77
        self.EXPECTED_BYTES = self.EXPECTED_LEN * 8
        self.fmt = "<" + "d" * self.EXPECTED_LEN  # Little endian + 77 doubles
        
        # Field names (column names) corresponding to the sender's order
        self.col_names = [
            "Time", "y_spcktime", "y_cb_vx", "y_cb_x", "y_cb_y", "y_cb_z",
            "y_cb_roll", "y_cb_yaw", "y_cb_pitch", "y_w01_rotw", "y_w02_rotw",
            "y_w03_rotw", "y_w04_rotw", "y_w05_rotw", "y_w06_rotw", "y_w07_rotw",
            "y_w08_rotw", "y_f01_x", "y_f01_y", "y_f01_z", "y_f01_roll", 
            "y_f01_yaw", "y_f01_pitch", "y_f02_x", "y_f02_y", "y_f02_z", 
            "y_f02_roll", "y_f02_yaw", "y_f02_pitch", "y_ws01_x", "y_ws01_y", 
            "y_ws01_z", "y_ws01_roll", "y_ws01_yaw", "y_ws01_pitch", "y_ws02_x", 
            "y_ws02_y", "y_ws02_z", "y_ws02_roll", "y_ws02_yaw", "y_ws02_pitch", 
            "y_ws03_x", "y_ws03_y", "y_ws03_z", "y_ws03_roll", "y_ws03_yaw", 
            "y_ws03_pitch", "y_ws04_x", "y_ws04_y", "y_ws04_z", "y_ws04_roll", 
            "y_ws04_yaw", "y_ws04_pitch", "y_w01_rota", "y_w02_rota", "y_w03_rota",
            "y_w04_rota", "y_w05_rota", "y_w06_rota", "y_w07_rota", "y_w08_rota",
            "y_bar01_pitch", "y_bar02_pitch", "y_bar03_pitch", "y_bar04_pitch",
            "y_bar05_pitch", "y_bar06_pitch", "y_bar07_pitch", "y_bar08_pitch",
            "y_ws01_vy", "y_ws02_vy", "y_ws03_vy", "y_ws04_vy", 
            "y_ws01_vyaw", "y_ws02_vyaw", "y_ws03_vyaw", "y_ws04_vyaw"
        ]
        
        # Statistics
        self.data_count = 0
        self.filtered_count = 0
        self.last_spcktime = None
        
        # Create UDP socket with retry capability
        self.create_socket()
        
        print(f"UDP receiver thread initialized, listening on {self.UDP_IP}:{self.UDP_PORT}")

    def create_socket(self):
        """Create UDP socket with retry mechanism for port binding"""
        retry_count = 0
        current_port = self.UDP_PORT
        
        while retry_count < self.max_retries:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                
                # Add socket option to allow port reuse
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                
                # Try to bind to the port
                self.sock.bind((self.UDP_IP, current_port))
                self.sock.settimeout(0.01)  # 10ms timeout for responsiveness
                
                # If successful, store the port and exit
                self.UDP_PORT = current_port
                print(f"Successfully bound to port {self.UDP_PORT}")
                return
                
            except OSError as e:
                if not self.port_retry:
                    raise e
                
                # If port is in use, try the next port
                retry_count += 1
                current_port = self.UDP_PORT + retry_count
                print(f"Port {self.UDP_PORT + retry_count - 1} is in use, trying {current_port}...")
        
        # If we've exhausted all retries, raise an exception
        raise RuntimeError(f"Failed to bind UDP socket after {self.max_retries} retries")

    def run(self):
        """Main loop for receiving UDP data"""
        global latest_data, previous_data, running, data_ready
        
        print("UDP receiver thread started...")
        
        while running:
            try:
                # Try to receive data
                data, addr = self.sock.recvfrom(65535)
                
                # Check data length
                if len(data) != self.EXPECTED_BYTES:
                    print(f"[WARNING] Received data length is {len(data)} bytes, expected {self.EXPECTED_BYTES} -> skipping")
                    continue

                # Parse 77 doubles
                values = struct.unpack(self.fmt, data)
                
                # Increment counter
                self.data_count += 1
                
                # Check for duplicate data
                current_spcktime = values[1]
                if self.last_spcktime is not None and current_spcktime == self.last_spcktime:
                    self.filtered_count += 1
                    continue
                
                # Update timestamp
                self.last_spcktime = current_spcktime
                
                # Print reception status (every 10000 packets)
                if self.data_count % 10000 == 0:
                    print(f"[INFO] Received {self.data_count} packets, current sim_time: {values[0]:.6f}")
                
                # Update data for rendering
                previous_data = latest_data  # Store previous frame for interpolation
                latest_data = values  # Update latest data
                data_ready.set()  # Notify rendering thread that data is ready
                
            except socket.timeout:
                # Socket timeout, continue loop
                pass
            except Exception as e:
                print(f"[ERROR] UDP receiver thread exception: {e}")
                if not running:
                    break
        
        print("UDP receiver thread terminated")
        self.sock.close()

# =========================== 2) Track Data Loading ===========================
class TrackData:
    def __init__(self, npz_path='trajectory_data.npz'):
        """Load track centerline and rail data"""
        try:
            trajectory_data = np.load(npz_path)
            self.s_vals = trajectory_data['s']        # Track mileage array
            self.xvals = trajectory_data['x']         # Corresponding global X
            self.yvals = trajectory_data['y']         # Global Y
            self.zvals = trajectory_data['z']         # Global Z
            self.psi_vals = trajectory_data['psi']    # Track yaw
            self.phi_vals = trajectory_data['phi']    # Track roll (if superelevation)
            self.left_rail = trajectory_data['left_rail']    # (N,3) Left rail
            self.right_rail = trajectory_data['right_rail']   # (N,3) Right rail
            print(f"Track data loaded successfully: {len(self.s_vals)} points")
        except Exception as e:
            print(f"[ERROR] Failed to load track data: {e}")
            # Create some virtual track data for testing
            self.s_vals = np.linspace(0, 1000, 1000)
            self.xvals = self.s_vals.copy()
            self.yvals = np.zeros_like(self.s_vals)
            self.zvals = np.zeros_like(self.s_vals)
            self.psi_vals = np.zeros_like(self.s_vals)
            self.phi_vals = np.zeros_like(self.s_vals)
            self.left_rail = np.column_stack([self.xvals, self.yvals + 0.75, self.zvals])
            self.right_rail = np.column_stack([self.xvals, self.yvals - 0.75, self.zvals])
            print("[WARNING] Using virtual track data for testing")
    
    def get_track_pose(self, s):
        """
        Input track mileage s, find the nearest point index idx in s_vals array,
        return the track position and orientation in global frame: (X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T).
        pitch_T is temporarily set to 0.0 (no gradient), roll_T=phi_vals[idx].
        """
        idx = np.argmin(np.abs(self.s_vals - s))
        X_T = self.xvals[idx]
        Y_T = self.yvals[idx]
        Z_T = self.zvals[idx]
        yaw_T = self.psi_vals[idx]
        pitch_T = 0.0       # Set to 0 if no gradient
        roll_T = self.phi_vals[idx]
        return (X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T)

# =========================== 3) Coordinate Transformation Functions ===========================
def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [ c, -s, 0],
        [ s,  c, 0],
        [ 0,  0, 1]
    ], dtype=float)

def rot_y(pitch):
    c, s = np.cos(pitch), np.sin(pitch)
    return np.array([
        [ c, 0,  s],
        [ 0, 1,  0],
        [-s, 0,  c]
    ], dtype=float)

def rot_x(roll):
    c, s = np.cos(roll), np.sin(roll)
    return np.array([
        [1,  0,  0],
        [0,  c, -s],
        [0,  s,  c]
    ], dtype=float)

def euler_zyx_to_matrix(yaw, pitch, roll):
    """
    Matrix = Rz(yaw) * Ry(pitch) * Rx(roll).
    """
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

def make_transform(yaw, pitch, roll, px, py, pz):
    """
    Generate a 4x4 homogeneous transformation matrix:
      R(zyx) + translation(px, py, pz).
    """
    T = np.eye(4)
    R = euler_zyx_to_matrix(yaw, pitch, roll)
    T[:3,:3] = R
    T[:3, 3] = [px, py, pz]
    return T

def transform_points_3d(T, points):
    """
    Transform point cloud points(N,3) using 4x4 homogeneous matrix T, returns (N,3).
    """
    ones = np.ones((points.shape[0], 1))
    homogeneous_points = np.hstack([points, ones])
    transformed_points = (T @ homogeneous_points.T).T
    return transformed_points[:, :3]

# Function to linearly interpolate between two values
def lerp(a, b, t):
    """Linear interpolation between a and b with factor t (0.0-1.0)"""
    return a + (b - a) * t

# Function to linearly interpolate between two angles (in radians)
def angle_lerp(a, b, t):
    """Interpolate between two angles, handling wrap-around properly"""
    # Ensure angles are within [0, 2π]
    a = a % (2.0 * np.pi)
    b = b % (2.0 * np.pi)
    
    # Find the shortest path
    diff = b - a
    if diff > np.pi:
        b -= 2.0 * np.pi
    elif diff < -np.pi:
        b += 2.0 * np.pi
    
    # Linear interpolation
    return a + (b - a) * t

# Function to interpolate two data frames
def interpolate_frames(frame1, frame2, t):
    """
    Interpolate between two data frames with factor t (0.0-1.0)
    Returns an array of interpolated values
    """
    if frame1 is None or frame2 is None:
        return frame2 if frame2 is not None else frame1
    
    # Create array for interpolated values
    interpolated = np.zeros(len(frame1), dtype=float)
    
    # Special handling for simulation time (index 0)
    interpolated[0] = lerp(frame1[0], frame2[0], t)
    
    # Interpolate positions (indices vary based on data format)
    position_indices = [3, 4, 5, 17, 18, 19, 23, 24, 25, 29, 30, 31, 35, 36, 37, 41, 42, 43, 47, 48, 49]
    for idx in position_indices:
        interpolated[idx] = lerp(frame1[idx], frame2[idx], t)
    
    # Interpolate orientations (angles)
    angle_indices = [6, 7, 8, 20, 21, 22, 26, 27, 28, 32, 33, 34, 38, 39, 40, 44, 45, 46, 50, 51, 52]
    for idx in angle_indices:
        interpolated[idx] = angle_lerp(frame1[idx], frame2[idx], t)
    
    # Copy other values directly
    for i in range(len(frame1)):
        if i not in position_indices and i not in angle_indices and i != 0:
            interpolated[i] = frame2[i]  # Use latest values for non-interpolated fields
    
    return interpolated

# =========================== 4) Geometric Model Creation ===========================
class TrainModels:
    def __init__(self):
        # Axle and wheel dimensions
        self.axle_length = 2.0
        self.axle_radius = 0.065
        self.wheel_radius = 0.43
        self.wheel_thickness = 0.04
        self.wheel_offset = 0.7175  # Wheel center offset from axle center
        
        # Car body and bogie dimensions
        self.car_body_length = 25.0
        self.car_body_width = 3.0
        self.car_body_height = 3.0
        
        self.bogie_length = 3.0
        self.bogie_width = 2.5
        self.bogie_height = 0.5
        
        # Create geometric models
        self.create_models()
        
        # Display lists will be created later during init_opengl
        self.axle_list = 0
        self.wheel_list = 0
        self.car_body_list = 0
        self.bogie_list = 0
    
    def create_cylinder(self, length, radius, sides=20):
        """Create cylinder vertices"""
        vertices = []
        
        # Side vertices
        for i in range(sides+1):
            theta = i * (2.0 * np.pi / sides)
            x = radius * np.cos(theta)
            z = radius * np.sin(theta)
            
            # Two end points
            vertices.append([-length/2, 0, 0])
            vertices.append([length/2, 0, 0])
            
            # Two side points
            vertices.append([length/2, x, z])
            vertices.append([-length/2, x, z])
        
        return np.array(vertices)
    
    def create_wheel(self, thickness, radius, sides=20):
        """Create wheel vertices"""
        vertices = []
        
        # Wheel is a cylinder along X-axis
        for i in range(sides+1):
            theta = i * (2.0 * np.pi / sides)
            y = radius * np.cos(theta)
            z = radius * np.sin(theta)
            
            # Two end points
            vertices.append([-thickness/2, 0, 0])
            vertices.append([thickness/2, 0, 0])
            
            # Two side points
            vertices.append([thickness/2, y, z])
            vertices.append([-thickness/2, y, z])
        
        return np.array(vertices)
    
    def create_box(self, length, width, height):
        """Create box vertices"""
        vertices = [
            # Front face (x=length/2)
            [length/2, -width/2, 0], [length/2, width/2, 0], [length/2, width/2, height], [length/2, -width/2, height],
            # Back face (x=-length/2)
            [-length/2, -width/2, 0], [-length/2, width/2, 0], [-length/2, width/2, height], [-length/2, -width/2, height],
            # Right face (y=width/2)
            [-length/2, width/2, 0], [length/2, width/2, 0], [length/2, width/2, height], [-length/2, width/2, height],
            # Left face (y=-width/2)
            [-length/2, -width/2, 0], [length/2, -width/2, 0], [length/2, -width/2, height], [-length/2, -width/2, height],
            # Top face (z=height)
            [-length/2, -width/2, height], [-length/2, width/2, height], [length/2, width/2, height], [length/2, -width/2, height],
            # Bottom face (z=0)
            [-length/2, -width/2, 0], [-length/2, width/2, 0], [length/2, width/2, 0], [length/2, -width/2, 0]
        ]
        return np.array(vertices)
    
    def create_models(self):
        """Create all geometric models"""
        # Axle model (cylinder along Y-axis)
        self.axle_vertices = self.create_cylinder(self.axle_length, self.axle_radius)
        
        # Wheel model (cylinder along X-axis)
        self.wheel_vertices = self.create_wheel(self.wheel_thickness, self.wheel_radius)
        
        # Car body model (box)
        self.car_body_vertices = self.create_box(
            self.car_body_length, self.car_body_width, self.car_body_height)
        
        # Bogie model (box)
        self.bogie_vertices = self.create_box(
            self.bogie_length, self.bogie_width, self.bogie_height)
    
    def create_display_lists(self):
        """Create OpenGL display lists to improve rendering performance"""
        # Note: This method should be called AFTER OpenGL initialization
        try:
            # Axle display list
            self.axle_list = glGenLists(1)
            glNewList(self.axle_list, GL_COMPILE)
            self.render_axle()
            glEndList()
            
            # Wheel display list
            self.wheel_list = glGenLists(1)
            glNewList(self.wheel_list, GL_COMPILE)
            self.render_wheel()
            glEndList()
            
            # Car body display list
            self.car_body_list = glGenLists(1)
            glNewList(self.car_body_list, GL_COMPILE)
            self.render_car_body()
            glEndList()
            
            # Bogie display list
            self.bogie_list = glGenLists(1)
            glNewList(self.bogie_list, GL_COMPILE)
            self.render_bogie()
            glEndList()
            
            print("Train model display lists created successfully")
        except Exception as e:
            print(f"Error creating display lists: {e}")
            # Fallback: Use immediate mode rendering instead of display lists
            self.axle_list = 0
            self.wheel_list = 0
            self.car_body_list = 0
            self.bogie_list = 0
    
    def render_axle(self):
        """Render axle"""
        glColor3f(1.0, 0.0, 1.0)  # Magenta
        glBegin(GL_QUADS)
        for i in range(0, len(self.axle_vertices), 4):
            for j in range(4):
                glVertex3fv(self.axle_vertices[i+j])
        glEnd()
    
    def render_wheel(self):
        """Render wheel"""
        glColor3f(0.0, 1.0, 1.0)  # Cyan
        glBegin(GL_QUADS)
        for i in range(0, len(self.wheel_vertices), 4):
            for j in range(4):
                glVertex3fv(self.wheel_vertices[i+j])
        glEnd()
    
    def render_car_body(self):
        """Render car body"""
        glColor4f(1.0, 0.5, 0.0, 0.2)  # Semi-transparent orange
        glBegin(GL_QUADS)
        for i in range(0, len(self.car_body_vertices), 4):
            for j in range(4):
                glVertex3fv(self.car_body_vertices[i+j])
        glEnd()
    
    def render_bogie(self):
        """Render bogie"""
        glColor4f(0.0, 0.8, 0.0, 0.3)  # Semi-transparent green
        glBegin(GL_QUADS)
        for i in range(0, len(self.bogie_vertices), 4):
            for j in range(4):
                glVertex3fv(self.bogie_vertices[i+j])
        glEnd()
    
    def draw_wheelset_immediate(self, T_W2G):
        """
        Draw one axle + two wheels using immediate mode rendering
        (fallback if display lists fail)
        """
        # Rotate axle by 90 degrees around Z-axis so it's perpendicular to track
        # Create a rotation matrix for the axle
        axle_rotation = np.array([
            [0, -1, 0, 0],  # Rotate 90 degrees around Z-axis
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Apply the rotation to the axle's transformation
        T_axle = T_W2G @ axle_rotation
        
        # Draw axle (now properly oriented)
        glPushMatrix()
        glMultMatrixf(T_axle.T.flatten())
        self.render_axle()
        glPopMatrix()
        
        # Draw left wheel
        glPushMatrix()
        T_wheel_left = np.eye(4)
        T_wheel_left[1, 3] = self.wheel_offset
        rotation = np.array([
            [0, 0, 1, 0],
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
        T_final = T_W2G @ T_wheel_left @ rotation
        glMultMatrixf(T_final.T.flatten())
        self.render_wheel()
        glPopMatrix()
        
        # Draw right wheel
        glPushMatrix()
        T_wheel_right = np.eye(4)
        T_wheel_right[1, 3] = -self.wheel_offset
        T_final = T_W2G @ T_wheel_right @ rotation
        glMultMatrixf(T_final.T.flatten())
        self.render_wheel()
        glPopMatrix()
    
    def draw_wheelset(self, T_W2G):
        """
        Draw one axle + two wheels
        T_W2G: Transformation matrix from wheelset local frame to global frame
        """
        if self.axle_list > 0 and self.wheel_list > 0:
            # Use display lists if available
            
            # Rotate axle by 90 degrees around Z-axis so it's perpendicular to track
            # Create a rotation matrix for the axle
            axle_rotation = np.array([
                [0, -1, 0, 0],  # Rotate 90 degrees around Z-axis
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            
            # Apply the rotation to the axle's transformation
            T_axle = T_W2G @ axle_rotation
            
            # Draw axle (now properly oriented)
            glPushMatrix()
            glMultMatrixf(T_axle.T.flatten())
            glCallList(self.axle_list)
            glPopMatrix()
            
            # Draw left wheel
            glPushMatrix()
            T_wheel_left = np.eye(4)
            T_wheel_left[1, 3] = self.wheel_offset
            rotation = np.array([
                [0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1]
            ])
            T_final = T_W2G @ T_wheel_left @ rotation
            glMultMatrixf(T_final.T.flatten())
            glCallList(self.wheel_list)
            glPopMatrix()
            
            # Draw right wheel
            glPushMatrix()
            T_wheel_right = np.eye(4)
            T_wheel_right[1, 3] = -self.wheel_offset
            T_final = T_W2G @ T_wheel_right @ rotation
            glMultMatrixf(T_final.T.flatten())
            glCallList(self.wheel_list)
            glPopMatrix()
        else:
            # Fallback to immediate mode rendering
            self.draw_wheelset_immediate(T_W2G)
    
    def draw_box_immediate(self, T_B2G, box_vertices):
        """
        Draw a box (car body or bogie) using immediate mode rendering
        (fallback if display lists fail)
        """
        glPushMatrix()
        glMultMatrixf(T_B2G.T.flatten())
        
        # Enable blending for transparency
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        if box_vertices is self.car_body_vertices:
            self.render_car_body()
        else:
            self.render_bogie()
        
        # Disable blending
        glDisable(GL_BLEND)
        glPopMatrix()
    
    def draw_box(self, T_B2G, display_list, box_type="bogie"):
        """
        Draw a box (car body or bogie)
        T_B2G: Transformation from box local frame to global frame
        display_list: Display list to call
        box_type: Type of box ("bogie" or "car_body")
        """
        if display_list > 0:
            # Use display lists if available
            glPushMatrix()
            glMultMatrixf(T_B2G.T.flatten())
            
            # Enable blending for transparency
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            
            glCallList(display_list)
            
            # Disable blending
            glDisable(GL_BLEND)
            glPopMatrix()
        else:
            # Fallback to immediate mode rendering
            if box_type == "bogie":
                self.draw_box_immediate(T_B2G, self.bogie_vertices)
            else:
                self.draw_box_immediate(T_B2G, self.car_body_vertices)

# =========================== 5) OpenGL Rendering Class ===========================
class TrainVisualization:
    def __init__(self, window_width=1200, window_height=800):
        self.window_width = window_width
        self.window_height = window_height
        
        # Initialize OpenGL first
        self.init_opengl()
        
        # Load track data
        self.track = TrackData()
        
        # Create train models
        self.train_models = TrainModels()
        
        # Camera parameters - MODIFIED: Initial view rotated 180° around X-axis
        self.camera_distance = 30.0
        self.camera_elevation = -30.0  # Negative elevation to flip view
        self.camera_azimuth = 120.0    # Adjusted azimuth (180° rotated)
        self.camera_target = [0.0, 0.0, 0.0]  # Target point
        self.auto_camera = True       # Auto-follow train
        self.camera_up_vector = [0, 0, -1]  # Inverted Z-axis as up vector
        
        # Display control
        self.show_track = True
        self.show_rails = True
        self.show_fps = True
        
        # Animation stats
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        
        # Now that OpenGL is initialized, create display lists
        self.compile_track_display_list()
        self.train_models.create_display_lists()
        
        # Start the animation timer
        self.start_time = time.time()
        self.last_time = self.start_time
        
        # Register display timer callback for animation
        glutTimerFunc(int(FRAME_TIME * 1000), self.animation_timer, 0)
    
    def init_opengl(self):
        """Initialize OpenGL environment"""
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(self.window_width, self.window_height)
        glutCreateWindow(b"Real-time Train Visualization System")
        
        # Set callback functions
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutKeyboardFunc(self.keyboard)
        glutSpecialFunc(self.special_keys)
        glutIdleFunc(self.idle)
        
        # Set background color
        glClearColor(0.9, 0.9, 1.0, 1.0)  # Light blue background
        
        # Enable depth testing
        glEnable(GL_DEPTH_TEST)
        
        # Enable lighting
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        
        # Set light source
        light_position = [100.0, 100.0, -100.0, 0.0]  # Directional light (z-inverted)
        glLightfv(GL_LIGHT0, GL_POSITION, light_position)
        
        # Set material properties
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    
    def animation_timer(self, value):
        """Timer callback for animation control"""
        global frame_time, last_update_time
        
        if not animation_paused:
            # Get current time
            current_time = time.time()
            elapsed = current_time - last_update_time
            
            # When using simulation time from UDP
            if use_received_time and latest_data is not None and previous_data is not None:
                # Calculate sim time progress
                sim_time_start = previous_data[0]
                sim_time_end = latest_data[0]
                sim_time_range = sim_time_end - sim_time_start
                
                if sim_time_range > 0:
                    # Calculate interpolation factor based on elapsed real time
                    # Map elapsed real time to a fraction of sim time range
                    t = min(1.0, elapsed / (FRAME_TIME * 10))  # Adjust speed factor as needed
                    frame_time = sim_time_start + t * sim_time_range
                else:
                    frame_time = sim_time_end
            else:
                # Just increment frame time for smooth animation 
                # when not using simulation time
                frame_time += FRAME_TIME
            
            # Request redisplay
            glutPostRedisplay()
            
            # Calculate FPS
            self.frame_count += 1
            fps_elapsed = current_time - self.last_fps_time
            if fps_elapsed >= 1.0:  # Update FPS once per second
                self.fps = self.frame_count / fps_elapsed
                self.frame_count = 0
                self.last_fps_time = current_time
        
        # Schedule next frame
        glutTimerFunc(int(FRAME_TIME * 1000), self.animation_timer, 0)
    
    def compile_track_display_list(self):
        """Precompile track display lists to improve performance"""
        try:
            # Track centerline display list
            self.track_center_list = glGenLists(1)
            glNewList(self.track_center_list, GL_COMPILE)
            glDisable(GL_LIGHTING)
            glColor3f(0.0, 0.0, 0.0)  # Black
            glBegin(GL_LINE_STRIP)
            for i in range(len(self.track.xvals)):
                glVertex3f(self.track.xvals[i], self.track.yvals[i], self.track.zvals[i])
            glEnd()
            glEnable(GL_LIGHTING)
            glEndList()
            
            # Left and right rails display list
            self.rails_list = glGenLists(1)
            glNewList(self.rails_list, GL_COMPILE)
            glDisable(GL_LIGHTING)
            
            # Left rail (red)
            glColor3f(0.8, 0.0, 0.0)
            glBegin(GL_LINE_STRIP)
            for i in range(len(self.track.left_rail)):
                glVertex3f(self.track.left_rail[i, 0], 
                           self.track.left_rail[i, 1], 
                           self.track.left_rail[i, 2])
            glEnd()
            
            # Right rail (blue)
            glColor3f(0.0, 0.0, 0.8)
            glBegin(GL_LINE_STRIP)
            for i in range(len(self.track.right_rail)):
                glVertex3f(self.track.right_rail[i, 0], 
                           self.track.right_rail[i, 1], 
                           self.track.right_rail[i, 2])
            glEnd()
            
            glEnable(GL_LIGHTING)
            glEndList()
            
            print("Track display lists created successfully")
        except Exception as e:
            print(f"Error creating track display lists: {e}")
            # Fallback: set these to 0 to use immediate mode rendering instead
            self.track_center_list = 0
            self.rails_list = 0
    
    def set_camera(self):
        """Set camera position and orientation"""
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # Calculate camera position
        x = self.camera_target[0] + self.camera_distance * np.cos(np.radians(self.camera_elevation)) * np.cos(np.radians(self.camera_azimuth))
        y = self.camera_target[1] + self.camera_distance * np.cos(np.radians(self.camera_elevation)) * np.sin(np.radians(self.camera_azimuth))
        z = self.camera_target[2] + self.camera_distance * np.sin(np.radians(self.camera_elevation))
        
        # Apply 180-degree rotation for Z-down coordinate system
        gluLookAt(x, y, z,  # Camera position
                  self.camera_target[0], self.camera_target[1], self.camera_target[2],  # Target point
                  self.camera_up_vector[0], self.camera_up_vector[1], self.camera_up_vector[2])  # Up vector (Z-down)
    
    def reshape(self, width, height):
        """Window resize callback function"""
        self.window_width = width
        self.window_height = height
        
        # Update viewport
        glViewport(0, 0, width, height)
        
        # Set projection matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(width)/float(height), 0.1, 1000.0)
    
    def keyboard(self, key, x, y):
        """Keyboard callback function"""
        global running, animation_paused, interpolation_enabled, use_received_time
        
        if key == b'\x1b':  # ESC key
            running = False
            glutLeaveMainLoop()
        elif key == b't':  # Toggle track display
            self.show_track = not self.show_track
        elif key == b'r':  # Toggle rails display
            self.show_rails = not self.show_rails
        elif key == b'f':  # Toggle FPS display
            self.show_fps = not self.show_fps
        elif key == b'p':  # Pause/Resume animation
            animation_paused = not animation_paused
        elif key == b'i':  # Toggle interpolation
            interpolation_enabled = not interpolation_enabled
        elif key == b's':  # Toggle between simulation time and real time
            use_received_time = not use_received_time
        elif key == b'c':  # Toggle camera auto-follow
            self.auto_camera = not self.auto_camera
        elif key == b'v':  # Toggle Z-axis orientation (up/down)
            self.camera_up_vector = [0, 0, -self.camera_up_vector[2]]  # Flip Z direction
            self.camera_elevation = -self.camera_elevation  # Invert elevation
    
    def special_keys(self, key, x, y):
        """Special keys callback function"""
        # Camera control
        if key == GLUT_KEY_UP:
            self.camera_elevation += 5.0
        elif key == GLUT_KEY_DOWN:
            self.camera_elevation -= 5.0
        elif key == GLUT_KEY_LEFT:
            self.camera_azimuth += 5.0
        elif key == GLUT_KEY_RIGHT:
            self.camera_azimuth -= 5.0
        elif key == GLUT_KEY_PAGE_UP:
            self.camera_distance -= 2.0
        elif key == GLUT_KEY_PAGE_DOWN:
            self.camera_distance += 2.0
        
        # Limit elevation range
        self.camera_elevation = max(-85.0, min(85.0, self.camera_elevation))
        
        # Limit camera distance
        self.camera_distance = max(5.0, self.camera_distance)
        
        glutPostRedisplay()
    
    def idle(self):
        """Idle callback function, used for data updates"""
        global data_ready, last_update_time
        
        # Check if new data is available
        if data_ready.is_set():
            data_ready.clear()
            last_update_time = time.time()  # Reset interpolation time
    
    def render_track_immediate(self):
        """Render track using immediate mode (fallback if display lists fail)"""
        # Track centerline
        glDisable(GL_LIGHTING)
        glColor3f(0.0, 0.0, 0.0)  # Black
        glBegin(GL_LINE_STRIP)
        for i in range(len(self.track.xvals)):
            glVertex3f(self.track.xvals[i], self.track.yvals[i], self.track.zvals[i])
        glEnd()
        
        # Left rail (red)
        glColor3f(0.8, 0.0, 0.0)
        glBegin(GL_LINE_STRIP)
        for i in range(len(self.track.left_rail)):
            glVertex3f(self.track.left_rail[i, 0], 
                       self.track.left_rail[i, 1], 
                       self.track.left_rail[i, 2])
        glEnd()
        
        # Right rail (blue)
        glColor3f(0.0, 0.0, 0.8)
        glBegin(GL_LINE_STRIP)
        for i in range(len(self.track.right_rail)):
            glVertex3f(self.track.right_rail[i, 0], 
                       self.track.right_rail[i, 1], 
                       self.track.right_rail[i, 2])
        glEnd()
        
        glEnable(GL_LIGHTING)
    
    def display(self):
        """Display callback function"""
        global latest_data, previous_data, frame_time
        
        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # Get interpolated data frame
        current_data = None
        
        if latest_data is not None:
            if interpolation_enabled and previous_data is not None:
                # Calculate interpolation factor
                sim_time_start = previous_data[0]
                sim_time_end = latest_data[0]
                
                if sim_time_end > sim_time_start:
                    # Calculate t factor (0.0-1.0) for interpolation
                    t = (frame_time - sim_time_start) / (sim_time_end - sim_time_start)
                    t = max(0.0, min(1.0, t))  # Clamp between 0 and 1
                    
                    # Get interpolated frame
                    current_data = interpolate_frames(previous_data, latest_data, t)
                else:
                    current_data = latest_data
            else:
                current_data = latest_data
        
        # Set camera
        self.set_camera()
        
        # Draw track
        if self.show_track:
            if self.track_center_list > 0:
                glCallList(self.track_center_list)
            else:
                self.render_track_immediate()
        
        # Draw rails
        if self.show_rails:
            if self.rails_list > 0:
                glCallList(self.rails_list)
            else:
                self.render_track_immediate()
        
        # If data is available, draw train
        if current_data is not None:
            # Update camera target if auto-follow is enabled
            if self.auto_camera:
                s_cb = current_data[3]  # y_cb_x
                y_cb = current_data[4]  # y_cb_y
                z_cb = current_data[5]  # y_cb_z
                
                # Get global position on track
                X_T, Y_T, Z_T, _, _, _ = self.track.get_track_pose(s_cb)
                
                # Update camera target
                self.camera_target = [X_T, Y_T, Z_T]
            
            # Render train with current data
            self.render_train(current_data)
        
        # Display help text on screen
        self.display_help_text()
        
        # Swap buffers
        glutSwapBuffers()
    
    def render_train(self, data):
        """Render train based on data"""
        # Draw 4 wheelsets
        for ws_id in range(1, 5):
            # Wheelset position indices
            s_idx = 29 + (ws_id-1) * 6    # y_ws0X_x
            y_idx = 30 + (ws_id-1) * 6    # y_ws0X_y
            z_idx = 31 + (ws_id-1) * 6    # y_ws0X_z
            r_idx = 32 + (ws_id-1) * 6    # y_ws0X_roll
            yw_idx = 33 + (ws_id-1) * 6   # y_ws0X_yaw
            p_idx = 34 + (ws_id-1) * 6    # y_ws0X_pitch
            
            s_ = data[s_idx]
            y_ = data[y_idx]
            z_ = data[z_idx]
            r_ = data[r_idx]
            yw_ = data[yw_idx]
            p_ = data[p_idx]
            
            # Get global position and orientation on track
            X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T = self.track.get_track_pose(s_)
            
            # Calculate transformation matrices
            T_T2G = make_transform(yaw_T, pitch_T, roll_T, X_T, Y_T, Z_T)
            T_W2T = make_transform(yw_, p_, r_, 0.0, y_, z_)
            T_W2G = T_T2G @ T_W2T
            
            # Draw wheelset
            self.train_models.draw_wheelset(T_W2G)
        
        # Draw 2 bogies
        for bg_id in range(1, 3):
            # Bogie position indices
            s_idx = 17 + (bg_id-1) * 6    # y_f0X_x
            y_idx = 18 + (bg_id-1) * 6    # y_f0X_y
            z_idx = 19 + (bg_id-1) * 6    # y_f0X_z
            r_idx = 20 + (bg_id-1) * 6    # y_f0X_roll
            yw_idx = 21 + (bg_id-1) * 6   # y_f0X_yaw
            p_idx = 22 + (bg_id-1) * 6    # y_f0X_pitch
            
            s_bg = data[s_idx]
            y_bg = data[y_idx]
            z_bg = data[z_idx]
            r_bg = data[r_idx]
            yw_bg = data[yw_idx]
            p_bg = data[p_idx]
            
            # Get global position and orientation on track
            X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T = self.track.get_track_pose(s_bg)
            
            # Calculate transformation matrices
            T_T2G = make_transform(yaw_T, pitch_T, roll_T, X_T, Y_T, Z_T)
            T_bg2T = make_transform(yw_bg, p_bg, r_bg, 0.0, y_bg, z_bg)
            T_bg2G = T_T2G @ T_bg2T
            
            # If 180-degree flip is needed
            T_flipBogie = make_transform(0, 0, math.pi, 0, 0, 0)
            T_bg2G = T_bg2G @ T_flipBogie
            
            # Draw bogie
            self.train_models.draw_box(T_bg2G, self.train_models.bogie_list, "bogie")
        
        # Draw car body
        s_cb = data[3]    # y_cb_x
        y_cb = data[4]    # y_cb_y
        z_cb = data[5]    # y_cb_z
        r_cb = data[6]    # y_cb_roll
        yw_cb = data[7]   # y_cb_yaw
        p_cb = data[8]    # y_cb_pitch
        
        # Get global position and orientation on track
        X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T = self.track.get_track_pose(s_cb)
        
        # Calculate transformation matrices
        T_T2G = make_transform(yaw_T, pitch_T, roll_T, X_T, Y_T, Z_T)
        T_cb2T = make_transform(yw_cb, p_cb, r_cb, 0.0, y_cb, z_cb)
        T_cb2G = T_T2G @ T_cb2T
        
        # If 180-degree flip is needed
        T_flipCar = make_transform(0, 0, math.pi, 0, 0, 0)
        T_cb2G = T_cb2G @ T_flipCar
        
        # Draw car body
        self.train_models.draw_box(T_cb2G, self.train_models.car_body_list, "car_body")
    
    def display_help_text(self):
        """Display help text on screen"""
        # Disable lighting and depth testing for 2D text
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        
        # Switch to orthographic projection
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.window_width, 0, self.window_height, -1, 1)
        
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        
        # Draw text
        glColor3f(0.0, 0.0, 0.0)  # Black text
        
        # Basic controls
        self.render_string(10, self.window_height - 20, b"Controls: Arrow keys - Rotate view, Page Up/Down - Zoom")
        self.render_string(10, self.window_height - 40, b"         T - Toggle track, R - Toggle rails, ESC - Exit")
        self.render_string(10, self.window_height - 60, b"         P - Pause/Resume, I - Toggle interpolation")
        self.render_string(10, self.window_height - 80, b"         C - Toggle camera auto-follow, V - Flip Z-axis")
        
        # Status information
        y_pos = self.window_height - 120
        
        # Display animation status
        status_text = f"Paused: {'Yes' if animation_paused else 'No'}, " \
                     f"Interpolation: {'On' if interpolation_enabled else 'Off'}, " \
                     f"Camera mode: {'Auto-follow' if self.auto_camera else 'Manual'}"
        self.render_string(10, y_pos, status_text.encode('utf-8'))
        y_pos -= 20
        
        # Display current frame time
        if latest_data is not None:
            frame_info = f"Sim time: {frame_time:.3f}s, Using: {'Sim Time' if use_received_time else 'Real Time'}"
            self.render_string(10, y_pos, frame_info.encode('utf-8'))
            y_pos -= 20
        
        # Display FPS if enabled
        if self.show_fps:
            fps_text = f"FPS: {self.fps:.1f}"
            self.render_string(10, y_pos, fps_text.encode('utf-8'))
        
        # Restore projection matrix
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()
        
        # Re-enable lighting and depth testing
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
    
    def render_string(self, x, y, string):
        """Render string at specified position"""
        glRasterPos2f(x, y)
        for c in string:
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ctypes.c_int(c))

# =========================== 6) Main Function ===========================
def main():
    global running
    
    # Create and start UDP receiver thread
    udp_thread = UDPReceiverThread()
    udp_thread.start()
    
    # Create visualization system
    visualization = TrainVisualization()
    
    # Start OpenGL main loop
    try:
        glutMainLoop()
    except Exception as e:
        print(f"Error in GLUT main loop: {e}")
    finally:
        running = False
    
    # Wait for UDP thread to end
    udp_thread.join(timeout=1.0)
    
    print("Program terminated normally")

if __name__ == "__main__":
    main()


"""
运行：

E:
cd E:\ResearchDocuments\ROS2WithSPCK\SPCK_Track
python OnlineVis_RTTrain.py


+-----------------------+      +----------------------+
| UDP接收线程           |      | OpenGL渲染线程       |
| - 监听UDP数据         | ---> | - 3D车辆模型渲染     |
| - 解析数据包          |      | - 轨道与环境渲染     |
| - 数据预处理          |      | - 相机视角控制       |
+-----------------------+      +----------------------+
            |                            ^
            v                            |
      +------------------+               |
      | 共享数据结构     |               |
      | - 最新车辆状态   | --------------+
      | - 位置与姿态信息 |
      +------------------+


"""
