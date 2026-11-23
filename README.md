# RealSense RGB-D Lever 6-DoF Grasp Inspector

![RealSense](https://img.shields.io/badge/Intel-RealSense-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)

**RealSense RGB-D 기반 상단 힌지 레버 6-DoF 파지(Grasp) 인스펙터**는 로봇 매니퓰레이터가 도어 레버(손잡이)를 정확하게 파지하고 조작할 수 있도록 돕는 실시간 비전 인식 및 시각화 도구입니다. AprilTag 마커와 Depth 정보를 융합하여 레버의 3차원 자세를 추정하고, 열림/닫힘 상태를 감지하며, 가상 2핑거 평행 그리퍼의 접근 가능성을 시뮬레이션합니다.

## 🌟 주요 기능 (Key Features)

- **정밀한 6-DoF 자세 추정**: AprilTag(tag36h11)를 기준으로 도어 손잡이의 정확한 3차원 위치(X, Y, Z)와 회전(Roll, Pitch, Yaw) 추정.
- **깊이(Depth) 기반 상태 감지**: 도어 평면을 기준으로 손잡이의 돌출 깊이를 측정하여 레버의 열림/닫힘 상태 판별.
- **가상 그리퍼 시뮬레이션**: 목표 파지 위치에 로봇 그리퍼(80mm 스트로크) 3D 모델을 투영하여 간섭 여부 시각적 확인.
- **4채널 실시간 대시보드**:
  - `View 1`: RGB 화면상 3D Pose HUD 렌더링
  - `View 2`: 도어면 기준 국소 돌출 깊이맵 (Colormap, 0~8cm)
  - `View 3`: RGB-D 반투명 블렌딩 정합 투시 뷰
  - `View 4`: 마젠타(Pink) 가상 평행 그리퍼 파지 시뮬레이션 뷰
- **대시보드 실시간 녹화**: `R` 키를 이용한 2x2 대시보드 화면 실시간 MP4 녹화 기능.

## 🛠 하드웨어 요구사항 (Hardware Requirements)

- **카메라**: Intel RealSense Depth Camera (D435 등 RGB-D 지원 모델)
- **마커**: 크기 6cm의 AprilTag (Family: `tag36h11`)

## 📦 설치 및 의존성 (Installation & Dependencies)

이 프로젝트는 Python 3.8 이상에서 동작합니다. 아래 명령어를 통해 필요한 패키지를 설치하십시오.

```bash
pip install numpy opencv-python opencv-contrib-python pyrealsense2 scipy
```

> **Note**: AprilTag 인식률을 높이기 위해 `pupil_apriltags` 패키지 사용을 권장합니다. 설치되어 있지 않은 경우 OpenCV의 기본 Aruco 모듈로 자동 대체되어 동작합니다.
```bash
pip install pupil-apriltags
```

## 🚀 사용 방법 (Usage)

1. RealSense 카메라를 PC에 연결합니다.
2. 타겟 도어(레버 근처 지정된 위치)에 6cm AprilTag를 부착합니다.
3. 메인 스크립트를 실행합니다.

```bash
python main_demo.py
```

### ⌨️ 조작 키 (Key Bindings)

- **`R` 키**: 4채널 대시보드 화면의 실시간 동영상 녹화를 시작하거나 종료합니다. (현재 디렉토리에 `lever_record_YYYYMMDD_HHMMSS.mp4` 형태로 자동 저장됩니다.)
- **`Q` 키**: 프로그램을 안전하게 종료합니다.

## ⚙️ 시스템 설정 (Configuration)

기구학 파라미터 및 센서 설정은 코드 내 `LeverKinematicsConfig` 클래스에서 수정할 수 있습니다.
실제 환경에 맞게 로봇 그리퍼 치수나 마커와 손잡이 간의 오프셋 거리를 조정하여 사용하십시오.

```python
# 예시: main_demo.py 내부 파라미터
tag_size_m = 0.06           # AprilTag 크기 (6cm)
handle_offset_x_m = 0.092   # 마커 기준 손잡이 X 오프셋 (9.2cm)
gripper_stroke_m = 0.080    # 그리퍼 최대 벌림 폭 (80mm)
```

## 📝 라이선스 (License)

이 프로젝트는 MIT 라이선스에 따라 배포됩니다.
