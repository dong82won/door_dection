RealSense Lever Grasp Inspector 🤖📹
Intel RealSense RGB-D 카메라를 활용한 상단 힌지 레버 6-DoF 파지(Grasp) 인스펙터입니다.

AprilTag 마커 인식을 기반으로 레버 손잡이의 3차원 6-DoF 자세를 추정하고, 가상 로봇 그리퍼의 파지 시뮬레이션 및 깊이(Depth) 맵 분석 결과를 4채널 대시보드를 통해 실시간으로 제공합니다.
🌟 주요 기능 (Key Features)
실시간으로 제공되는 2x2 분할 대시보드 화면을 통해 다음 4가지 뷰를 확인할 수 있습니다:
1.	[View 1] RGB 3D Pose HUD: RGB 화면상에 AprilTag 3축 및 계산된 손잡이 타겟의 3축(X, Y, Z) 렌더링.
2.	[View 2] Local Protrusion Colormap: 도어 면을 기준으로 한 국소 돌출 깊이맵(Depth Map, 0~8cm) 컬러맵 시각화.
3.	[View 3] RGB-D Overlay Blend: RGB 뷰와 Depth 뷰의 반투명 블렌딩(55:45 비율)을 통한 정합 투시 뷰.
4.	[View 4] Virtual Gripper Clearance: 마젠타(Pink) 색상의 평행 2핑거 가상 로봇 그리퍼 파지 시뮬레이션 뷰.
·	실시간 녹화 기능: 대시보드 화면을 MP4 비디오 형식으로 실시간 녹화할 수 있습니다.
🛠️ 하드웨어 요구사항 (Hardware Requirements)
·	카메라: Intel RealSense D435 (또는 호환되는 RealSense RGB-D 카메라)
·	타겟 마커: AprilTag (Family: tag36h11, Size: 6cm)
📦 설치 방법 (Installation)
이 프로젝트는 Python 3.8 이상 환경에서 실행하는 것을 권장합니다.
1.	레포지토리를 클론합니다.
git clone https://github.com/yourusername/lever-grasp-inspector.git
cd lever-grasp-inspector

2.	필요한 Python 패키지를 설치합니다.
pip install opencv-python numpy pyrealsense2 scipy

3.	(권장) 성능 향상 및 더 정교한 AprilTag 인식을 위해 pupil-apriltags 패키지를 추가로 설치하는 것을 적극 권장합니다. (미설치 시 OpenCV의 기본 Aruco 모듈로 대체되어 동작합니다.)
pip install pupil-apriltags

🚀 실행 및 조작법 (Usage)
RealSense 카메라를 PC에 연결한 후, 다음 명령어를 통해 메인 스크립트를 실행합니다.
python main_demo.py

단축키 (Keyboard Controls)
Key	Description
[R]	🎥 실시간 4채널 대시보드 동영상 녹화 시작 / 종료 토글 (루트 디렉토리에 .mp4 저장)
[Q]	⏹️ 프로그램 안전 종료

⚙️ 기구학 설정 (Kinematics Configuration)
main_demo.py 내부의 LeverKinematicsConfig 데이터 클래스를 수정하여 환경에 맞게 파라미터를 조정할 수 있습니다.
·	마커 및 오프셋: tag_size_m, handle_offset_x_m, handle_offset_y_m 등을 통해 마커와 손잡이 간의 실제 물리적 거리를 설정합니다.
·	그리퍼 설정: gripper_stroke_m, finger_width_m 등 가상으로 렌더링 될 2핑거 로봇 그리퍼의 치수를 정의합니다.
·	필터 계수: filter_alpha 값을 조정하여 3D 좌표 추정 시 발생하는 떨림(Jitter)을 제어할 수 있습니다.
📝 License
This project is licensed under the MIT License - see the LICENSE file for details.
