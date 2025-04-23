import cv2
import numpy as np
import socket
import threading
import struct
import time
import zlib
from collections import defaultdict
from queue import Queue, Full, Empty
import mediapipe as mp

# UDP配置
UDP_IP = "0.0.0.0"  # 监听所有网络接口
UDP_PORT = 8888
PACKET_SIZE = 1024  # 与ESP32端保持一致
HEADER_SIZE = 20  # frame_id + packet_id + total_packets + packet_size + crc
MAX_QUEUE_SIZE = 10  # 最大显示队列大小

class Statistics:
    def __init__(self):
        self.total_packets = 0
        self.lost_packets = 0
        self.current_frame_id = 0
        self.received_packets = set()
        self.last_feedback_time = time.time()
        self.feedback_interval = 0.1  # 提高反馈频率到100ms
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.frame_stats = {}
        self.processing_time = 0  # 添加处理时间统计

    def update_fps(self):
        self.frame_count += 1
        current_time = time.time()
        time_diff = current_time - self.last_fps_time
        if time_diff >= 1.0:
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.last_fps_time = current_time
            # 打印详细统计信息
            print(f"性能统计 - FPS: {self.fps:.1f}, "
                  f"丢包率: {(self.lost_packets / max(1, self.total_packets)) * 100:.2f}%, "
                  f"平均处理时间: {self.processing_time*1000:.2f}ms")

    def reset_frame_stats(self, frame_id, total_packets):
        if frame_id not in self.frame_stats:
            self.frame_stats[frame_id] = {
                'total_packets': total_packets,
                'received_packets': set(),
                'complete': False,
                'start_time': time.time()
            }
        # 只保留最近10帧的统计
        old_frames = [fid for fid in self.frame_stats.keys() if fid < frame_id - 10]
        for fid in old_frames:
            if not self.frame_stats[fid]['complete']:
                self.lost_packets += (self.frame_stats[fid]['total_packets'] - 
                                    len(self.frame_stats[fid]['received_packets']))
            del self.frame_stats[fid]

    def add_packet(self, frame_id, packet_id, total_packets):
        self.reset_frame_stats(frame_id, total_packets)
        self.frame_stats[frame_id]['received_packets'].add(packet_id)
        self.total_packets = sum(fs['total_packets'] for fs in self.frame_stats.values())

class FaceDetector:
    def __init__(self):
        # 初始化MediaPipe人脸检测
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_drawing = mp.solutions.drawing_utils
        self.face_detection = self.mp_face_detection.FaceDetection(
            model_selection=1,  # 0表示适合近距离人脸，1表示适合远距离人脸
            min_detection_confidence=0.5
        )
        # 初始化MediaPipe人脸网格
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=3,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
    def detect_faces(self, frame):
        """检测人脸"""
        # 将BGR图像转换为RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 进行人脸检测
        detection_results = self.face_detection.process(rgb_frame)
        
        # 进行人脸网格检测
        mesh_results = self.face_mesh.process(rgb_frame)
        
        if detection_results.detections:
            for detection in detection_results.detections:
                # 获取边界框
                bbox = detection.location_data.relative_bounding_box
                ih, iw, _ = frame.shape
                x, y = int(bbox.xmin * iw), int(bbox.ymin * ih)
                w, h = int(bbox.width * iw), int(bbox.height * ih)
                
                # 绘制边界框
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # 显示置信度分数
                score = detection.score[0]
                cv2.putText(frame, f'Score: {score:.2f}', (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 绘制人脸网格
        if mesh_results.multi_face_landmarks:
            for face_landmarks in mesh_results.multi_face_landmarks:
                # 绘制所有特征点
                for idx, landmark in enumerate(face_landmarks.landmark):
                    ih, iw, _ = frame.shape
                    x, y = int(landmark.x * iw), int(landmark.y * ih)
                    cv2.circle(frame, (x, y), 1, (255, 0, 0), -1)
                
                # 绘制眼睛轮廓
                left_eye = [(face_landmarks.landmark[p].x * iw, face_landmarks.landmark[p].y * ih) 
                           for p in [33, 246, 161, 160, 159, 158, 157, 173, 133, 155, 154, 153, 145, 144, 163, 7]]
                right_eye = [(face_landmarks.landmark[p].x * iw, face_landmarks.landmark[p].y * ih) 
                            for p in [362, 398, 384, 385, 386, 387, 388, 466, 263, 249, 390, 373, 374, 380, 381, 382]]
                
                # 转换为numpy数组并绘制
                left_eye = np.array(left_eye, dtype=np.int32)
                right_eye = np.array(right_eye, dtype=np.int32)
                cv2.polylines(frame, [left_eye], True, (0, 255, 255), 1)
                cv2.polylines(frame, [right_eye], True, (0, 255, 255), 1)
        
        return frame

class FrameAssembler:
    def __init__(self):
        self.frames = defaultdict(dict)
        self.display_queue = Queue(maxsize=MAX_QUEUE_SIZE)
        self.lock = threading.Lock()
        self.stats = Statistics()
        self.feedback_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_feedback(self, addr):
        current_time = time.time()
        if current_time - self.stats.last_feedback_time >= self.stats.feedback_interval:
            feedback_data = struct.pack('II', 
                                      self.stats.lost_packets,
                                      self.stats.total_packets)
            try:
                self.feedback_sock.sendto(feedback_data, (addr[0], UDP_PORT + 1))
                self.stats.last_feedback_time = current_time
            except Exception as e:
                print(f"发送反馈数据失败: {e}")

    def verify_crc(self, data, expected_crc):
        calculated_crc = zlib.crc32(data) & 0xFFFFFFFF
        return calculated_crc == expected_crc

    def add_packet(self, frame_id, packet_id, total_packets, data, packet_size, crc, addr):
        start_time = time.time()
        
        with self.lock:
            self.stats.add_packet(frame_id, packet_id, total_packets)
            
            if not self.verify_crc(data[:packet_size], crc):
                return

            self.send_feedback(addr)

            if packet_size > 0:
                self.frames[frame_id][packet_id] = data[:packet_size]
                
                if len(self.frames[frame_id]) == total_packets:
                    try:
                        frame_data = b''.join([self.frames[frame_id][i] 
                                             for i in range(total_packets)])
                        
                        frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), 
                                           cv2.IMREAD_COLOR)
                        
                        if frame is not None:
                            try:
                                self.display_queue.put_nowait((frame, self.stats.fps))
                            except Full:
                                # 如果队列满了，移除最旧的帧
                                try:
                                    self.display_queue.get_nowait()
                                    self.display_queue.put_nowait((frame, self.stats.fps))
                                except:
                                    pass
                        
                        del self.frames[frame_id]
                        
                        if frame_id in self.stats.frame_stats:
                            self.stats.frame_stats[frame_id]['complete'] = True
                            
                        self.stats.update_fps()
                        
                    except Exception as e:
                        print(f"组装帧数据失败: {e}")
        
        # 更新处理时间统计
        self.stats.processing_time = (self.stats.processing_time * 0.9 + 
                                    (time.time() - start_time) * 0.1)

def receive_stream(assembler):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    # 增加接收缓冲区大小
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536 * 8)
    
    # 设置超时，避免阻塞
    sock.settimeout(0.1)
    
    print(f"正在监听 UDP 端口 {UDP_PORT}...")
    
    while True:
        try:
            data, addr = sock.recvfrom(PACKET_SIZE + HEADER_SIZE)
            if len(data) < HEADER_SIZE:
                continue

            frame_id, packet_id, total_packets, packet_size, crc = struct.unpack('IIIII', 
                                                                               data[:HEADER_SIZE])
            
            assembler.add_packet(frame_id, packet_id, total_packets, 
                               data[HEADER_SIZE:], packet_size, crc, addr)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"接收数据错误: {e}")

def display_stream(assembler):
    # 初始化人脸检测器
    face_detector = FaceDetector()
    
    # 确保在主线程中创建窗口
    try:
        cv2.namedWindow('Camera Stream', cv2.WINDOW_NORMAL)
        cv2.moveWindow('Camera Stream', 0, 0)  # 移动窗口到左上角
        cv2.resizeWindow('Camera Stream', 1024, 768)  # 使用更大的窗口
    except Exception as e:
        print(f"创建窗口失败: {e}")
        return
    
    last_frame_time = time.time()
    
    while True:
        try:
            frame, fps = assembler.display_queue.get(timeout=1.0)
            
            current_time = time.time()
            frame_interval = current_time - last_frame_time
            last_frame_time = current_time
            
            # 进行人脸检测
            frame = face_detector.detect_faces(frame)
            
            # 添加统计信息到画面
            cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f'Frame Interval: {frame_interval*1000:.1f}ms', (10, 70),
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f'Queue Size: {assembler.display_queue.qsize()}', (10, 110),
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            try:
                cv2.imshow('Camera Stream', frame)
            except cv2.error as e:
                print(f"显示帧失败: {e}")
                continue
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):  # 添加截图功能
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f'capture_{timestamp}.jpg', frame)
                print(f"截图已保存: capture_{timestamp}.jpg")
        except Empty:
            continue
        except Exception as e:
            print(f"显示错误: {e}")
            time.sleep(0.1)  # 出错时短暂暂停
            continue

    try:
        cv2.destroyAllWindows()
    except:
        pass

def main():
    assembler = FrameAssembler()
    
    # 创建接收线程
    receive_thread = threading.Thread(target=receive_stream, args=(assembler,))
    receive_thread.daemon = True
    receive_thread.start()
    
    # 在主线程中运行显示
    try:
        display_stream(assembler)
    except KeyboardInterrupt:
        print("\n正在退出...")
    finally:
        # 清理资源
        try:
            cv2.destroyAllWindows()
        except:
            pass

if __name__ == "__main__":
    main() 