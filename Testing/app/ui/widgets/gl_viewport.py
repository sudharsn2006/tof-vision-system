"""
app/ui/widgets/gl_viewport.py
------------------------------
Author: SUDHARSAN
High-performance 3D QOpenGLWidget Viewport for LiDAR 3D Viewer Pro.
Renders GPU-accelerated VBO/VAO scenes including orbit camera, infinite ground grid, XYZ axes,
cylinder robot model, rotating laser sweep, anti-aliased points, 3D distance rings, bounding circles, and HUD overlays.
"""

import math
import time
from typing import List, Optional, Tuple
import numpy as np
import glm

from PySide6.QtCore import Qt, QTimer, QPoint, Signal
from PySide6.QtGui import QMouseEvent, QWheelEvent, QPainter, QColor, QFont, QPen
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from OpenGL.GL import *
from OpenGL.GL import shaders

from app.config import AppConfig
from app.core.data_types import ScanPoint, LidarScan, DetectedObject


# GLSL Shader Source Code (Compatibility & GLES3 / OpenGL 3.0 safe)
VERTEX_SHADER_SRC = """
#version 120
uniform mat4 u_mvp;
attribute vec3 a_pos;
attribute vec4 a_color;
attribute float a_size;
varying vec4 v_color;

void main() {
    gl_Position = u_mvp * vec4(a_pos, 1.0);
    gl_PointSize = a_size;
    v_color = a_color;
}
"""

FRAGMENT_SHADER_SRC = """
#version 120
varying vec4 v_color;

void main() {
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist_sq = dot(coord, coord);
    if (dist_sq > 0.25) {
        discard;
    }
    // Smooth anti-aliased edge alpha fade
    float alpha = 1.0 - smoothstep(0.16, 0.25, dist_sq);
    gl_FragColor = vec4(v_color.rgb, v_color.a * alpha);
}
"""


class OrbitCamera:
    """3D Orbiting Perspective Camera with smooth target panning and mouse wheel zoom."""

    def __init__(self):
        self.target = glm.vec3(0.0, 0.0, 0.0)
        self.distance = 4500.0  # mm distance from target
        self.azimuth = 45.0     # degrees (rotation around Z axis)
        self.elevation = 35.0   # degrees (pitch angle)
        self.fov = 60.0         # Field of view

        self.min_distance = 300.0
        self.max_distance = 30000.0
        self.min_elevation = -85.0
        self.max_elevation = 85.0

    def reset(self) -> None:
        """Reset camera to default isometric perspective."""
        self.target = glm.vec3(0.0, 0.0, 0.0)
        self.distance = 4500.0
        self.azimuth = 45.0
        self.elevation = 35.0
        self.fov = 60.0

    def rotate(self, delta_azimuth: float, delta_elevation: float) -> None:
        """Rotate camera angles."""
        self.azimuth = (self.azimuth + delta_azimuth) % 360.0
        self.elevation = max(self.min_elevation, min(self.max_elevation, self.elevation + delta_elevation))

    def pan(self, delta_x: float, delta_y: float) -> None:
        """Pan target position parallel to current camera orientation."""
        az_rad = math.radians(self.azimuth)
        # Right vector
        rx = -math.sin(az_rad)
        ry = math.cos(az_rad)
        # Up vector (projected)
        ux = -math.cos(az_rad)
        uy = -math.sin(az_rad)

        speed = self.distance * 0.0012
        self.target.x += (rx * delta_x + ux * delta_y) * speed
        self.target.y += (ry * delta_x + uy * delta_y) * speed

    def zoom(self, delta_factor: float) -> None:
        """Zoom camera closer or further."""
        self.distance = max(self.min_distance, min(self.max_distance, self.distance * delta_factor))

    def get_eye_position(self) -> glm.vec3:
        """Calculate eye position in world space."""
        az_rad = math.radians(self.azimuth)
        el_rad = math.radians(self.elevation)

        x = self.target.x + self.distance * math.cos(el_rad) * math.cos(az_rad)
        y = self.target.y + self.distance * math.cos(el_rad) * math.sin(az_rad)
        z = self.target.z + self.distance * math.sin(el_rad)
        return glm.vec3(x, y, z)

    def get_view_matrix(self) -> glm.mat4:
        """Return 4x4 View Matrix."""
        eye = self.get_eye_position()
        up = glm.vec3(0.0, 0.0, 1.0)
        return glm.lookAt(eye, self.target, up)

    def get_projection_matrix(self, aspect_ratio: float) -> glm.mat4:
        """Return 4x4 Perspective Projection Matrix."""
        return glm.perspective(glm.radians(self.fov), aspect_ratio, 10.0, 100000.0)


class OpenGLViewport(QOpenGLWidget):
    """Real-time 3D OpenGL Viewport Widget for RViz-like LiDAR visualization."""

    fps_updated = Signal(float)

    def __init__(self, config: AppConfig, parent=None):
        super().__init__(parent)
        self.config = config
        self.camera = OrbitCamera()

        # State data
        self.current_scan: Optional[LidarScan] = None
        self.detected_objects: List[DetectedObject] = []
        self.nearest_object: Optional[DetectedObject] = None
        self.has_collision: bool = False
        self.sweep_angle_deg: float = 0.0

        # Mouse Interaction State
        self.last_mouse_pos = QPoint()
        self.mouse_world_pos = (0.0, 0.0, 0.0)

        # Performance counters
        self.frame_count: int = 0
        self.last_fps_calc_time = time.time()
        self.current_fps: float = 60.0

        # Render loop timer
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.update)
        self.render_timer.start(1000 // max(self.config.fps_limit, 15))

        # OpenGL GLSL Program Handles
        self.shader_program = None
        self.u_mvp_loc = -1
        self.a_pos_loc = -1
        self.a_color_loc = -1
        self.a_size_loc = -1

        # VBO Handles
        self.vbo_dynamic = None

    def update_scan_data(
        self,
        scan: LidarScan,
        objects: List[DetectedObject],
        nearest: Optional[DetectedObject],
        collision: bool
    ) -> None:
        """Receive updated scan frame and cluster metrics from main thread."""
        self.current_scan = scan
        self.detected_objects = objects
        self.nearest_object = nearest
        self.has_collision = collision
        if scan and scan.points:
            self.sweep_angle_deg = scan.points[-1].angle_deg

    def initializeGL(self) -> None:
        """Initialize OpenGL contexts, shaders, and GL states."""
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # Enable Point Sprite & Point Size scaling
        glEnable(GL_POINT_SPRITE)
        glEnable(GL_VERTEX_PROGRAM_POINT_SIZE)

        # Compile GLSL Shaders
        try:
            vert_shader = shaders.compileShader(VERTEX_SHADER_SRC, GL_VERTEX_SHADER)
            frag_shader = shaders.compileShader(FRAGMENT_SHADER_SRC, GL_FRAGMENT_SHADER)
            self.shader_program = shaders.compileProgram(vert_shader, frag_shader)

            self.u_mvp_loc = glGetUniformLocation(self.shader_program, "u_mvp")
            self.a_pos_loc = glGetAttribLocation(self.shader_program, "a_pos")
            self.a_color_loc = glGetAttribLocation(self.shader_program, "a_color")
            self.a_size_loc = glGetAttribLocation(self.shader_program, "a_size")
        except Exception as e:
            print(f"[OpenGLViewport] Shader compilation error: {e}")

        self.vbo_dynamic = glGenBuffers(1)

    def resizeGL(self, w: int, h: int) -> None:
        """Handle viewport resize events."""
        glViewport(0, 0, w, h)

    def paintGL(self) -> None:
        """Main GPU 3D render loop."""
        # Calculate rendering FPS
        now = time.time()
        self.frame_count += 1
        if now - self.last_fps_calc_time >= 0.5:
            self.current_fps = self.frame_count / (now - self.last_fps_calc_time)
            self.frame_count = 0
            self.last_fps_calc_time = now
            self.fps_updated.emit(self.current_fps)

        # Clear buffers with configurable dark background color
        bg = self.config.bg_color
        glClearColor(bg[0], bg[1], bg[2], bg[3])
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        if not self.shader_program:
            return

        aspect = self.width() / max(float(self.height()), 1.0)
        proj_matrix = self.camera.get_projection_matrix(aspect)
        view_matrix = self.camera.get_view_matrix()
        mvp_matrix = proj_matrix * view_matrix
        mvp_np = np.array(mvp_matrix, dtype=np.float32)

        glUseProgram(self.shader_program)
        glUniformMatrix4fv(self.u_mvp_loc, 1, GL_FALSE, mvp_np)

        # Vertex buffer data accumulator: [x, y, z, r, g, b, a, size]
        lines_vertices: List[float] = []
        points_vertices: List[float] = []

        # 1. Ground Grid (Minor 100mm, Major 1000mm)
        extent = self.config.grid_extent_mm
        minor_step = self.config.minor_grid_mm
        major_step = self.config.major_grid_mm

        x = -extent
        while x <= extent:
            is_major = (abs(x) % major_step < 1e-3) or (abs(x - extent) < 1e-3)
            color = [0.25, 0.30, 0.40, 0.8] if is_major else [0.15, 0.18, 0.25, 0.4]

            lines_vertices.extend([x, -extent, 0.0, *color, 1.0])
            lines_vertices.extend([x,  extent, 0.0, *color, 1.0])
            x += minor_step

        y = -extent
        while y <= extent:
            is_major = (abs(y) % major_step < 1e-3) or (abs(y - extent) < 1e-3)
            color = [0.25, 0.30, 0.40, 0.8] if is_major else [0.15, 0.18, 0.25, 0.4]

            lines_vertices.extend([-extent, y, 0.0, *color, 1.0])
            lines_vertices.extend([ extent, y, 0.0, *color, 1.0])
            y += minor_step

        # 2. Distance Rings (0.5m, 1m, 2m, 3m, 4m)
        if self.config.show_distance_rings:
            ring_radii = [500.0, 1000.0, 2000.0, 3000.0, 4000.0]
            num_ring_segments = 64
            for r in ring_radii:
                ring_color = [0.0, 0.6, 0.9, 0.5] if r == self.config.collision_radius_mm else [0.2, 0.4, 0.6, 0.35]
                for i in range(num_ring_segments):
                    a1 = (2.0 * math.pi / num_ring_segments) * i
                    a2 = (2.0 * math.pi / num_ring_segments) * (i + 1)
                    lines_vertices.extend([r * math.cos(a1), r * math.sin(a1), 0.0, *ring_color, 1.0])
                    lines_vertices.extend([r * math.cos(a2), r * math.sin(a2), 0.0, *ring_color, 1.0])

        # 3. Coordinate Axes (X=Red, Y=Green, Z=Blue)
        axis_len = 800.0
        # X Axis
        lines_vertices.extend([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 2.0])
        lines_vertices.extend([axis_len, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 2.0])
        # Y Axis
        lines_vertices.extend([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0])
        lines_vertices.extend([0.0, axis_len, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0])
        # Z Axis
        lines_vertices.extend([0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 2.0])
        lines_vertices.extend([0.0, 0.0, axis_len, 0.0, 0.5, 1.0, 1.0, 2.0])

        # 4. Robot Base Cylinder & Direction Arrow
        if self.config.show_robot_model:
            r_robot = 180.0
            h_robot = 120.0
            robot_segments = 24
            robot_color = [0.3, 0.35, 0.45, 0.9]
            for i in range(robot_segments):
                a1 = (2.0 * math.pi / robot_segments) * i
                a2 = (2.0 * math.pi / robot_segments) * (i + 1)
                # Bottom circle
                lines_vertices.extend([r_robot * math.cos(a1), r_robot * math.sin(a1), 0.0, *robot_color, 1.0])
                lines_vertices.extend([r_robot * math.cos(a2), r_robot * math.sin(a2), 0.0, *robot_color, 1.0])
                # Top circle
                lines_vertices.extend([r_robot * math.cos(a1), r_robot * math.sin(a1), h_robot, *robot_color, 1.0])
                lines_vertices.extend([r_robot * math.cos(a2), r_robot * math.sin(a2), h_robot, *robot_color, 1.0])
                # Vertical pillars
                lines_vertices.extend([r_robot * math.cos(a1), r_robot * math.sin(a1), 0.0, *robot_color, 1.0])
                lines_vertices.extend([r_robot * math.cos(a1), r_robot * math.sin(a1), h_robot, *robot_color, 1.0])

            # Forward Red Direction Arrow
            arrow_len = 350.0
            lines_vertices.extend([0.0, 0.0, h_robot / 2.0, 1.0, 0.2, 0.2, 1.0, 3.0])
            lines_vertices.extend([arrow_len, 0.0, h_robot / 2.0, 1.0, 0.2, 0.2, 1.0, 3.0])
            lines_vertices.extend([arrow_len, 0.0, h_robot / 2.0, 1.0, 0.2, 0.2, 1.0, 3.0])
            lines_vertices.extend([arrow_len - 60.0, 40.0, h_robot / 2.0, 1.0, 0.2, 0.2, 1.0, 3.0])
            lines_vertices.extend([arrow_len, 0.0, h_robot / 2.0, 1.0, 0.2, 0.2, 1.0, 3.0])
            lines_vertices.extend([arrow_len - 60.0, -40.0, h_robot / 2.0, 1.0, 0.2, 0.2, 1.0, 3.0])

        # 5. Laser Sweep Beam Animation Line
        if self.config.show_laser_sweep:
            sweep_rad = math.radians(self.sweep_angle_deg)
            sw_x = 4000.0 * math.cos(sweep_rad)
            sw_y = 4000.0 * math.sin(sweep_rad)
            lines_vertices.extend([0.0, 0.0, 10.0, 0.0, 1.0, 0.8, 0.6, 1.5])
            lines_vertices.extend([sw_x, sw_y, 10.0, 0.0, 1.0, 0.8, 0.6, 1.5])

        # 6. Object Bounding Circles & Centroid Lines
        for obj in self.detected_objects:
            obj_color = [1.0, 0.1, 0.1, 0.9] if obj.is_collision else [0.0, 0.8, 1.0, 0.8]
            cx, cy, br = obj.centroid_x_mm, obj.centroid_y_mm, obj.bounding_radius_mm
            circle_segs = 32
            for i in range(circle_segs):
                a1 = (2.0 * math.pi / circle_segs) * i
                a2 = (2.0 * math.pi / circle_segs) * (i + 1)
                lines_vertices.extend([cx + br * math.cos(a1), cy + br * math.sin(a1), 5.0, *obj_color, 2.0])
                lines_vertices.extend([cx + br * math.cos(a2), cy + br * math.sin(a2), 5.0, *obj_color, 2.0])

            # Centroid cross
            lines_vertices.extend([cx - 50.0, cy, 5.0, *obj_color, 2.0])
            lines_vertices.extend([cx + 50.0, cy, 5.0, *obj_color, 2.0])
            lines_vertices.extend([cx, cy - 50.0, 5.0, *obj_color, 2.0])
            lines_vertices.extend([cx, cy + 50.0, 5.0, *obj_color, 2.0])

        # 7. Render Point Cloud
        pt_size = self.config.point_size
        if self.current_scan and self.current_scan.points:
            for pt in self.current_scan.points:
                if pt.distance_mm <= 0:
                    continue
                if pt.is_collision:
                    color = [1.0, 0.15, 0.15, 1.0]
                elif pt.is_warning:
                    color = [1.0, 0.85, 0.0, 0.9]
                else:
                    color = [0.0, 1.0, 0.4, 0.85]
                points_vertices.extend([pt.x_mm, pt.y_mm, pt.z_mm, *color, pt_size])

        # Draw Lines VBO
        if lines_vertices:
            buf_lines = np.array(lines_vertices, dtype=np.float32)
            self._render_vbo(buf_lines, GL_LINES)

        # Draw Points VBO
        if points_vertices:
            buf_points = np.array(points_vertices, dtype=np.float32)
            self._render_vbo(buf_points, GL_POINTS)

        glUseProgram(0)

        # Render 2D Qt Overlay HUD (FPS, Point Count, Camera Info)
        if self.config.show_hud:
            self._render_hud_overlay()

    def _render_vbo(self, data_np: np.ndarray, primitive_type: int) -> None:
        """Upload and render vertex attribute data."""
        stride = 8 * 4  # 8 floats per vertex (x,y,z, r,g,b,a, size)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_dynamic)
        glBufferData(GL_ARRAY_BUFFER, data_np.nbytes, data_np, GL_STREAM_DRAW)

        glEnableVertexAttribArray(self.a_pos_loc)
        glVertexAttribPointer(self.a_pos_loc, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(0))

        glEnableVertexAttribArray(self.a_color_loc)
        glVertexAttribPointer(self.a_color_loc, 4, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(3 * 4))

        glEnableVertexAttribArray(self.a_size_loc)
        glVertexAttribPointer(self.a_size_loc, 1, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(7 * 4))

        glDrawArrays(primitive_type, 0, len(data_np) // 8)

        glDisableVertexAttribArray(self.a_pos_loc)
        glDisableVertexAttribArray(self.a_color_loc)
        glDisableVertexAttribArray(self.a_size_loc)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

    def _render_hud_overlay(self) -> None:
        """Render 2D HUD text overlay onto viewport surface using QPainter."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.TextAntialiasing)

        # Top-Left HUD (Telemetry & Performance)
        painter.setFont(QFont("Consolas", 10, QFont.Bold))
        painter.setPen(QPen(QColor(15, 18, 25, 180)))
        painter.setBrush(QColor(15, 18, 25, 180))
        painter.drawRoundedRect(10, 10, 220, 95, 6, 6)

        painter.setPen(QColor(0, 255, 200))
        pts_count = len(self.current_scan.points) if self.current_scan else 0
        scan_hz = self.current_scan.scan_frequency_hz if self.current_scan else 0.0
        conn_str = "CONNECTED" if (self.current_scan and pts_count > 0) else "WAITING"

        painter.drawText(20, 30, f"Render FPS:    {self.current_fps:.1f}")
        painter.drawText(20, 50, f"Scan Rate:     {scan_hz:.1f} Hz")
        painter.drawText(20, 70, f"Point Count:   {pts_count}")
        painter.drawText(20, 90, f"Connection:    {conn_str}")

        # Bottom-Left HUD (Camera & Mouse Controls)
        h = self.height()
        painter.setPen(QPen(QColor(15, 18, 25, 180)))
        painter.drawRoundedRect(10, h - 85, 250, 75, 6, 6)

        painter.setPen(QColor(180, 210, 245))
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(20, h - 65, f"Camera Dist: {self.camera.distance / 1000.0:.2f} m")
        painter.drawText(20, h - 48, f"Azimuth:     {self.camera.azimuth:.1f}°")
        painter.drawText(20, h - 31, f"Elevation:   {self.camera.elevation:.1f}°")
        painter.drawText(20, h - 14, "Controls: L-Drag(Rot) M-Drag(Pan) Wheel(Zoom)")

        painter.end()

    # --- Mouse Event Handlers for 3D Camera Manipulation ---

    def mousePressEvent(self, event: QMouseEvent) -> None:
        """Capture mouse press position."""
        self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        """Handle 3D orbit rotation and middle-button panning."""
        dx = event.x() - self.last_mouse_pos.x()
        dy = event.y() - self.last_mouse_pos.y()

        if event.buttons() & Qt.LeftButton:
            # Orbit Rotate Camera
            self.camera.rotate(dx * 0.4 * self.config.camera_speed, -dy * 0.4 * self.config.camera_speed)
        elif event.buttons() & Qt.MiddleButton:
            # Pan Camera
            self.camera.pan(-dx, dy)

        self.last_mouse_pos = event.pos()

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        """Reset 3D camera orientation on double click."""
        if event.button() == Qt.LeftButton:
            self.camera.reset()

    def wheelEvent(self, event: QWheelEvent) -> None:
        """Handle camera mouse wheel zoom."""
        delta = event.angleDelta().y()
        factor = 0.9 if delta > 0 else 1.11
        self.camera.zoom(factor)

    def capture_frame_np(self) -> np.ndarray:
        """Capture current OpenGL framebuffer as an RGB numpy array."""
        self.makeCurrent()
        w = self.width()
        h = self.height()
        glPixelStorei(GL_PACK_ALIGNMENT, 1)
        data = glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE)
        img = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
        # Flip vertically to correct OpenGL top-down orientation
        return np.flipud(img)
