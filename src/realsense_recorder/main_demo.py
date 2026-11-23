"""RealSense RGB-D 기반 상단 힌지 레버 6-DoF 파지(Grasp) 인스펙터.

- View 1: RGB 화면상 AprilTag 3축 및 손잡이 타겟 3축 렌더링
- View 2: 도어면 기준 국소 돌출 깊이맵 (0~8cm)
- View 3: RGB-D 반투명 블렌딩(55:45) 정합 투시 뷰
- View 4: 마젠타(Pink) 가상 2핑거 로봇 그리퍼 파지 시뮬레이션 뷰
- 기능 추가: [R] 키를 이용한 4채널 대시보드 화면 실시간 녹화(MP4)
"""

import datetime
import logging
import math
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R

try:
    from pupil_apriltags import Detector  # type: ignore[import-not-found]

    USE_PUPIL = True
except ImportError:
    USE_PUPIL = False

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("LeverGraspInspector")


# ==========================================
# 1. 설정 및 데이터 모델 (Data Contract)
# ==========================================
@dataclass
class LeverKinematicsConfig:
    """센서, 기구학 및 가상 그리퍼 파라미터."""

    width: int = 640
    height: int = 480
    fps: int = 30

    tag_size_m: float = 0.06  # AprilTag 크기: 6cm

    # AprilTag 중심 기준 레버 손잡이 3D 기하학 오프셋 (단위: 미터)
    handle_offset_x_m: float = 0.092  # 마커 좌측 9.2cm 위치
    handle_offset_y_m: float = -0.110  # 상단 힌지 방향 세로 중심 정렬
    handle_width_m: float = 0.046  # 손잡이 순수 폭: 4.6cm
    handle_length_m: float = 0.320  # 손잡이 순수 길이: 32.0cm
    handle_base_z_m: float = 0.038  # 닫힘 상태 기본 돌출 두께: 3.8cm

    # 가상 평행 그리퍼 치수 모델 (단위: 미터)
    gripper_stroke_m: float = 0.080  # 핑거 간격 (80mm 스트로크)
    finger_width_m: float = 0.012  # 핑거 두께: 12mm
    finger_length_m: float = 0.045  # 핑거 길이: 45mm
    palm_depth_m: float = 0.035  # 팜 베이스 두께: 35mm

    filter_alpha: float = 0.35  # 시간축 저역통과 필터 계수


@dataclass(frozen=True)
class LeverPose6DoF:
    """로봇 제어용 손잡이 파지 6-DoF 자세 모델."""

    x: float
    y: float
    z: float
    distance_m: float
    roll: float
    pitch: float
    yaw: float
    qx: float
    qy: float
    qz: float
    qw: float
    tilt_deg: float


@dataclass(frozen=True)
class LeverGraspPacket:
    """제어 계층으로 전달되는 최종 데이터 패킷."""

    pose: LeverPose6DoF | None
    center_2d: tuple[int, int]
    is_opened: bool
    tracking_valid: bool
    debug_status: str


@dataclass(frozen=True)
class VisualInspectionFrames:
    """4채널 시각화를 위한 중간 프레임 패킷."""

    rgb_pose: np.ndarray  # View 1: RGB 3D Pose HUD
    depth_colormap: np.ndarray  # View 2: Local Protrusion Colormap
    rgbd_blend: np.ndarray  # View 3: RGB-D Overlay Blend
    gripper_sim: np.ndarray  # View 4: Virtual Gripper Simulation


# ==========================================
# 2. 하드웨어 계층 (RealSense Pipeline)
# ==========================================
class RealSenseD435Stream:
    """D435 스트림 생명주기 및 하드웨어 필터 관리."""

    def __init__(self, cfg: LeverKinematicsConfig) -> None:
        self.cfg = cfg
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, cfg.fps)
        self.config.enable_stream(rs.stream.color, cfg.width, cfg.height, rs.format.bgr8, cfg.fps)

        self.align = rs.align(rs.stream.color)
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()

        self.intrinsics: rs.intrinsics | None = None
        self.cam_mat: np.ndarray | None = None
        self.depth_scale: float = 0.001

    def start(self) -> None:
        profile = self.pipeline.start(self.config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()

        if intr is None:
            raise RuntimeError("카메라 내부 파라미터(Intrinsics) 획득 실패")

        self.intrinsics = intr
        self.cam_mat = np.array(
            [
                [intr.fx, 0.0, intr.ppx],
                [0.0, intr.fy, intr.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())
        logger.info("RealSense 스트림 시작 완료 (Scale=%.4fm)", self.depth_scale)

    def get_frames(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        filtered = self.spatial.process(depth_frame)
        filtered = self.temporal.process(filtered)
        filtered = self.hole_filling.process(filtered)
        depth_obj = filtered.as_depth_frame()

        color_img = np.asanyarray(color_frame.get_data())
        depth_m = np.asanyarray(depth_obj.get_data(), dtype=np.float32) * self.depth_scale

        return color_img, depth_m

    def stop(self) -> None:
        self.pipeline.stop()
        logger.info("RealSense 스트림 종료")


# ==========================================
# 3. 비전 인지 및 3D 모델 투영 엔진
# ==========================================
class LeverModelProjectionEngine:
    """3D 모델 투영, 다중 3축 렌더링 및 가상 그리퍼 시뮬레이션 엔진."""

    def __init__(self, cfg: LeverKinematicsConfig) -> None:
        self.cfg = cfg
        self._f_pos: np.ndarray | None = None
        self._f_quat: np.ndarray | None = None

        if USE_PUPIL:
            self.detector = Detector(families="tag36h11")
        else:
            self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            self.params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)  # type: ignore[reportArgumentType]
            half = cfg.tag_size_m / 2.0
            self.marker_pts = np.array(
                [
                    [-half, half, 0.0],
                    [half, half, 0.0],
                    [half, -half, 0.0],
                    [-half, -half, 0.0],
                ],
                dtype=np.float32,
            )

    def _get_handle_3d_corners(self) -> np.ndarray:
        """마커 로컬 좌표계 기준 레버 손잡이 4개 코너 3D 좌표를 반환합니다."""
        hw = self.cfg.handle_width_m / 2.0
        hl = self.cfg.handle_length_m / 2.0
        cx = self.cfg.handle_offset_x_m
        cy = self.cfg.handle_offset_y_m
        cz = self.cfg.handle_base_z_m

        return np.array(
            [
                [cx - hw, cy - hl, cz],  # 상단 좌측
                [cx + hw, cy - hl, cz],  # 상단 우측
                [cx + hw, cy + hl, cz],  # 하단 우측
                [cx - hw, cy + hl, cz],  # 하단 좌측
            ],
            dtype=np.float64,
        )

    def _get_virtual_gripper_boxes(self) -> list[np.ndarray]:
        """로봇 툴 로컬 좌표계 기준 평행 그리퍼(좌/우 핑거, 팜) 3D 박스들을 생성합니다."""
        hw = self.cfg.gripper_stroke_m / 2.0
        fw = self.cfg.finger_width_m
        fl = self.cfg.finger_length_m
        pd = self.cfg.palm_depth_m
        fy_half = 0.015  # 핑거 두께 (Y축 방향)

        box_left = self._create_3d_box(-hw - fw, -hw, -fy_half, fy_half, -fl * 0.4, fl * 0.6)
        box_right = self._create_3d_box(hw, hw + fw, -fy_half, fy_half, -fl * 0.4, fl * 0.6)
        box_palm = self._create_3d_box(-hw - fw * 1.2, hw + fw * 1.2, -0.025, 0.025, -fl * 0.4 - pd, -fl * 0.4)

        return [box_left, box_right, box_palm]

    @staticmethod
    def _create_3d_box(x1: float, x2: float, y1: float, y2: float, z1: float, z2: float) -> np.ndarray:
        return np.array(
            [
                [x1, y1, z1],
                [x2, y1, z1],
                [x2, y2, z1],
                [x1, y2, z1],
                [x1, y1, z2],
                [x2, y1, z2],
                [x2, y2, z2],
                [x1, y2, z2],
            ],
            dtype=np.float64,
        )

    def process(
        self,
        color_img: np.ndarray,
        depth_m: np.ndarray,
        intrinsics: rs.intrinsics,
        cam_mat: np.ndarray,
    ) -> tuple[LeverGraspPacket, VisualInspectionFrames]:
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        img_h, img_w = depth_m.shape[:2]

        view_rgb = color_img.copy()
        view_depth = np.zeros((img_h, img_w, 3), dtype=np.uint8)
        view_blend = color_img.copy()
        view_gripper = (color_img * 0.45).astype(np.uint8)

        rvec: np.ndarray | None = None
        tvec: np.ndarray | None = None
        corners: np.ndarray | None = None

        if USE_PUPIL:
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy),
                tag_size=self.cfg.tag_size_m,
            )
            if detections:
                rvec, _ = cv2.Rodrigues(detections[0].pose_R)
                tvec = detections[0].pose_t.reshape(3, 1)
                corners = detections[0].corners
        else:
            c_list, ids, _ = self.detector.detectMarkers(color_img)
            if ids is not None and len(c_list) > 0:
                corners = c_list[0].reshape((4, 2))
                success, r_tmp, t_tmp = cv2.solvePnP(
                    self.marker_pts,
                    corners,
                    cam_mat,
                    np.zeros(5),
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if success:
                    rvec, tvec = r_tmp, t_tmp

        if rvec is None or tvec is None or corners is None:
            self._f_pos, self._f_quat = None, None
            packet = self._empty_packet("Searching AprilTag...")
            return packet, VisualInspectionFrames(view_rgb, view_depth, view_blend, view_gripper)

        rot_mat, _ = cv2.Rodrigues(rvec)

        cv2.polylines(view_rgb, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.drawFrameAxes(view_rgb, cam_mat, np.zeros(5), rvec, tvec, length=0.045, thickness=2)

        corners_3d = self._get_handle_3d_corners()
        proj_pts, _ = cv2.projectPoints(corners_3d, rvec, tvec, cam_mat, np.zeros(5))
        handle_poly_2d = proj_pts.reshape(-1, 2).astype(np.int32)
        handle_poly_2d[:, 0] = np.clip(handle_poly_2d[:, 0], 0, img_w - 1)
        handle_poly_2d[:, 1] = np.clip(handle_poly_2d[:, 1], 0, img_h - 1)

        handle_mask = np.zeros((img_h, img_w), dtype=np.uint8)
        cv2.fillPoly(handle_mask, [handle_poly_2d], 255)

        tag_center = corners.mean(axis=0)
        tag_w = float(np.linalg.norm(corners[0] - corners[1]))
        sample_x = max(5, min(img_w - 6, int(tag_center[0] + tag_w * 1.2)))
        sample_y = max(5, min(img_h - 6, int(tag_center[1] + tag_w * 1.0)))

        door_samples = depth_m[sample_y - 4 : sample_y + 5, sample_x - 4 : sample_x + 5].flatten()
        valid_door = door_samples[(door_samples > 0.3) & (door_samples < 2.5)]
        z_door = float(np.median(valid_door)) if len(valid_door) >= 10 else float(tvec[2])

        rx, ry, rw, rh = cv2.boundingRect(handle_poly_2d)
        pad = int(tag_w * 0.4)
        x1, y1 = max(0, rx - pad), max(0, ry - pad)
        x2, y2 = min(img_w, rx + rw + pad), min(img_h, ry + rh + pad)

        roi_depth = depth_m[y1:y2, x1:x2]
        valid_mask = roi_depth > 0.2
        protrusion_roi = np.zeros_like(roi_depth)
        protrusion_roi[valid_mask] = z_door - roi_depth[valid_mask]

        clipped_roi = np.clip(protrusion_roi, 0.0, 0.08)
        norm_roi = (clipped_roi / 0.08 * 255.0).astype(np.uint8)
        colored_roi = cv2.applyColorMap(norm_roi, cv2.COLORMAP_JET)
        colored_roi[~valid_mask] = [0, 0, 0]

        view_depth[y1:y2, x1:x2] = colored_roi
        cv2.polylines(view_depth, [handle_poly_2d], True, (255, 255, 255), 2)
        cv2.putText(
            view_depth,
            f"Z_door: {z_door:.3f}m",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        protrusion_full = np.zeros_like(depth_m)
        valid_full = (depth_m > 0.2) & (depth_m < 2.5)
        protrusion_full[valid_full] = z_door - depth_m[valid_full]
        clipped_full = np.clip(protrusion_full, 0.0, 0.08)
        norm_full = (clipped_full / 0.08 * 255.0).astype(np.uint8)
        colored_depth_full = cv2.applyColorMap(norm_full, cv2.COLORMAP_JET)
        colored_depth_full[~valid_full] = [0, 0, 0]

        view_blend = cv2.addWeighted(color_img, 0.55, colored_depth_full, 0.45, 0)
        cv2.polylines(view_blend, [handle_poly_2d], True, (0, 255, 255), 2)
        cv2.polylines(view_blend, [corners.astype(np.int32)], True, (0, 255, 0), 2)

        handle_depths = depth_m[handle_mask > 0]
        valid_handle_depths = handle_depths[(handle_depths > 0.2) & (handle_depths < z_door)]

        if len(valid_handle_depths) >= 30:
            d_handle_median = float(np.median(valid_handle_depths))
            actual_protrusion_m = z_door - d_handle_median
        else:
            d_handle_median = z_door - self.cfg.handle_base_z_m
            actual_protrusion_m = self.cfg.handle_base_z_m

        top_pt_2d = ((handle_poly_2d[0] + handle_poly_2d[1]) // 2).astype(int)
        bot_pt_2d = ((handle_poly_2d[2] + handle_poly_2d[3]) // 2).astype(int)

        d_top = float(depth_m[top_pt_2d[1], top_pt_2d[0]])
        d_bot = float(depth_m[bot_pt_2d[1], bot_pt_2d[0]])

        dz_top = (z_door - d_top) if d_top > 0.2 else self.cfg.handle_base_z_m
        dz_bot = (z_door - d_bot) if d_bot > 0.2 else self.cfg.handle_base_z_m

        delta_dz = max(0.0, dz_bot - dz_top)
        sin_val = min(1.0, max(-1.0, delta_dz / self.cfg.handle_length_m))
        tilt_deg = math.degrees(math.asin(sin_val))
        is_opened = actual_protrusion_m > 0.055

        M = cv2.moments(handle_poly_2d)
        if M["m00"] > 0:
            cx_2d = int(M["m10"] / M["m00"])
            cy_2d = int(M["m01"] / M["m00"])
        else:
            cx_2d = int(handle_poly_2d[:, 0].mean())
            cy_2d = int(handle_poly_2d[:, 1].mean())

        raw_x = (cx_2d - intrinsics.ppx) * d_handle_median / intrinsics.fx
        raw_y = (cy_2d - intrinsics.ppy) * d_handle_median / intrinsics.fy
        raw_target_3d = np.array([raw_x, raw_y, d_handle_median], dtype=np.float64)

        z_tool = -rot_mat[:, 2]
        y_tool = -rot_mat[:, 1]
        x_tool = np.cross(y_tool, z_tool)
        x_tool /= np.linalg.norm(x_tool)
        y_tool = np.cross(z_tool, x_tool)

        r_base = R.from_matrix(np.column_stack([x_tool, y_tool, z_tool]))
        r_tilt = R.from_euler("x", -tilt_deg, degrees=True)
        target_rot = r_base * r_tilt
        raw_quat = target_rot.as_quat()

        alpha = self.cfg.filter_alpha
        if self._f_pos is None or self._f_quat is None:
            self._f_pos = raw_target_3d.copy()
            self._f_quat = raw_quat.copy()
        else:
            self._f_pos = alpha * raw_target_3d + (1.0 - alpha) * self._f_pos
            if np.dot(self._f_quat, raw_quat) < 0.0:
                raw_quat = -raw_quat
            interp_quat = (1.0 - alpha) * self._f_quat + alpha * raw_quat
            self._f_quat = interp_quat / float(np.linalg.norm(interp_quat))

        f_pos = self._f_pos
        f_quat = self._f_quat
        if f_pos is None or f_quat is None:
            packet = self._empty_packet("Filter initialization error")
            return packet, VisualInspectionFrames(view_rgb, view_depth, view_blend, view_gripper)

        rvec_target, _ = cv2.Rodrigues(R.from_quat(f_quat).as_matrix())
        tvec_target = f_pos.reshape(3, 1)

        cv2.polylines(view_rgb, [handle_poly_2d], True, (0, 255, 255), 2)
        cv2.drawFrameAxes(view_rgb, cam_mat, np.zeros(5), rvec_target, tvec_target, length=0.050, thickness=2)
        cv2.circle(view_rgb, (cx_2d, cy_2d), 10, (255, 255, 0), 2)
        cv2.drawMarker(view_rgb, (cx_2d, cy_2d), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(view_rgb, "GRASP TARGET", (cx_2d + 15, cy_2d), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # [수정] 가상 그리퍼 렌더링 색상을 핫핑크(255, 105, 180) BGR로 변경
        gripper_color = (180, 105, 255)  # BGR 형식으로 마젠타/핫핑크 표기
        cv2.polylines(view_gripper, [handle_poly_2d], True, (0, 255, 255), 2)
        gripper_boxes = self._get_virtual_gripper_boxes()

        for box in gripper_boxes:
            pts_2d, _ = cv2.projectPoints(box, rvec_target, tvec_target, cam_mat, np.zeros(5))
            pts = pts_2d.reshape(-1, 2).astype(np.int32)
            cv2.polylines(view_gripper, [pts[:4]], True, gripper_color, 2)
            cv2.polylines(view_gripper, [pts[4:]], True, gripper_color, 2)
            for i in range(4):
                cv2.line(view_gripper, tuple(pts[i]), tuple(pts[i + 4]), gripper_color, 1)

        cv2.drawFrameAxes(view_gripper, cam_mat, np.zeros(5), rvec_target, tvec_target, length=0.055, thickness=2)
        cv2.putText(
            view_gripper,
            "GRIPPER: READY (Stroke: 80mm)",
            (15, img_h - 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            gripper_color,
            2,
        )
        cv2.putText(
            view_gripper,
            f"Clearance: OK | Protrusion: {actual_protrusion_m * 1000:.1f}mm",
            (15, img_h - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 0),
            1,
        )

        euler = R.from_quat(f_quat).as_euler("xyz", degrees=True)
        pose = LeverPose6DoF(
            x=float(f_pos[0]),
            y=float(f_pos[1]),
            z=float(f_pos[2]),
            distance_m=float(np.linalg.norm(f_pos)),
            roll=float(euler[0]),
            pitch=float(euler[1]),
            yaw=float(euler[2]),
            qx=float(f_quat[0]),
            qy=float(f_quat[1]),
            qz=float(f_quat[2]),
            qw=float(f_quat[3]),
            tilt_deg=tilt_deg,
        )

        packet = LeverGraspPacket(
            pose=pose,
            center_2d=(cx_2d, cy_2d),
            is_opened=is_opened,
            tracking_valid=True,
            debug_status="TRACKING",
        )

        self._draw_hud(view_rgb, packet, actual_protrusion_m)

        return packet, VisualInspectionFrames(view_rgb, view_depth, view_blend, view_gripper)

    def _empty_packet(self, status: str) -> LeverGraspPacket:
        return LeverGraspPacket(None, (0, 0), False, False, status)

    @staticmethod
    def _draw_hud(img: np.ndarray, packet: LeverGraspPacket, dZ_m: float) -> None:
        if packet.tracking_valid and packet.pose is not None:
            p = packet.pose
            status_txt = "UNLOCKED (OPENED)" if packet.is_opened else "LOCKED (FLUSH)"
            color_status = (0, 140, 255) if packet.is_opened else (0, 255, 0)

            cv2.putText(
                img,
                f"STATUS: {status_txt} | Protrusion: {dZ_m * 1000.0:.1f}mm | Tilt: {p.tilt_deg:.1f} deg",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color_status,
                2,
            )
            cv2.putText(
                img,
                f"ROBOT TARGET (cm): X={p.x * 100:5.1f} | Y={p.y * 100:5.1f} | Z={p.z * 100:5.1f}",
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img,
                f"TOOL ROTATION (deg): R={p.roll:5.1f} | P={p.pitch:5.1f} | Y={p.yaw:5.1f}",
                (20, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (100, 255, 100),
                2,
            )
        else:
            cv2.putText(
                img,
                f"STATE: {packet.debug_status}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
            )


# ==========================================
# 4. 2x2 대시보드 뷰어, 녹화 및 메인 진입점
# ==========================================
def compose_dashboard(frames: VisualInspectionFrames) -> np.ndarray:
    """4개 채널 프레임을 2x2 그리드(1280x960)로 결합합니다."""
    v1 = frames.rgb_pose.copy()
    cv2.putText(
        v1,
        "[1] RGB 3D Pose HUD (Tag & Target Axes: X=R, Y=G, Z=B)",
        (15, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 255, 255),
        2,
    )

    v2 = frames.depth_colormap.copy()
    cv2.putText(
        v2,
        "[2] Local Protrusion Colormap (0~8cm)",
        (15, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        2,
    )

    v3 = frames.rgbd_blend.copy()
    cv2.putText(
        v3,
        "[3] RGB-D Overlay Blend (Alignment Check)",
        (15, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 255, 255),
        2,
    )

    v4 = frames.gripper_sim.copy()
    cv2.putText(
        v4,
        "[4] Virtual Gripper Clearance (Pink)",
        (15, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (180, 105, 255),
        2,
    )

    top_row = np.hstack([v1, v2])
    bot_row = np.hstack([v3, v4])
    return np.vstack([top_row, bot_row])


def main() -> None:
    cfg = LeverKinematicsConfig()
    sensor = RealSenseD435Stream(cfg)
    engine = LeverModelProjectionEngine(cfg)

    win_name = "RealSense Lever Grasp Inspector (Advanced Dashboard + REC)"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 960)

    # 비디오 녹화 상태 변수
    is_recording = False
    video_writer: cv2.VideoWriter | None = None
    dash_w, dash_h = 1280, 960

    logger.info("======================================================")
    logger.info("🎥 단축키 안내:")
    logger.info("   [R] 키 : 동영상 녹화 시작 / 종료 토글")
    logger.info("   [Q] 키 : 프로그램 종료")
    logger.info("======================================================")

    try:
        sensor.start()
        if sensor.cam_mat is None or sensor.intrinsics is None:
            return

        while True:
            if cv2.getWindowProperty(win_name, cv2.WND_PROP_VISIBLE) < 1:
                break

            color_img, depth_m = sensor.get_frames()
            if color_img is None or depth_m is None:
                continue

            packet, visual_frames = engine.process(
                color_img,
                depth_m,
                sensor.intrinsics,
                sensor.cam_mat,
            )

            dashboard = compose_dashboard(visual_frames)

            # [기능 추가] 녹화 상태 처리 및 아이콘 깜빡임
            if is_recording and video_writer is not None:
                # 우측 상단 붉은색 '● REC' 깜빡임 (0.5초 주기)
                if int(time.time() * 2) % 2 == 0:
                    cv2.circle(dashboard, (dash_w - 90, 30), 8, (0, 0, 255), -1)
                    cv2.putText(
                        dashboard,
                        "REC",
                        (dash_w - 70, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )
                video_writer.write(dashboard)

            cv2.imshow(win_name, dashboard)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                # 녹화 시작/종료 토글
                if not is_recording:
                    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"lever_record_{timestamp}.mp4"
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    video_writer = cv2.VideoWriter(filename, fourcc, float(cfg.fps), (dash_w, dash_h))
                    is_recording = True
                    logger.info("🎥 녹화 시작: %s", filename)
                else:
                    if video_writer:
                        video_writer.release()
                        video_writer = None
                    is_recording = False
                    logger.info("⏹️ 녹화 종료 및 저장 완료.")

    except Exception as e:
        logger.critical("시스템 예외 발생: %s", e, exc_info=True)
    finally:
        if video_writer:
            video_writer.release()
        sensor.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
