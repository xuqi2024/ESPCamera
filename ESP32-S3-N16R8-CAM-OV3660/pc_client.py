import cv2
import numpy as np
import socket
import threading
import struct
import time
import zlib
import pyttsx3
import argparse
from translate import Translator
from collections import defaultdict
from queue import Queue, Full, Empty
from ultralytics import YOLO

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
        self.feedback_interval = 0.2  # 降低反馈频率到200ms
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.frame_stats = {}
        self.processing_time = 0
        self.window_size = 15  # 减小统计窗口大小
        self.frame_history = []

    def update_fps(self):
        self.frame_count += 1
        current_time = time.time()
        time_diff = current_time - self.last_fps_time
        if time_diff >= 1.0:
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.last_fps_time = current_time
            
            # 计算滑动窗口内的丢包率
            total_lost = 0
            total_packets = 0
            for frame in self.frame_history[-self.window_size:]:
                total_lost += frame['lost']
                total_packets += frame['total']
            
            loss_rate = (total_lost / max(1, total_packets)) * 100 if total_packets > 0 else 0
            
            # 打印详细统计信息
            print(f"性能统计 - FPS: {self.fps:.1f}, "
                  f"丢包率: {loss_rate:.2f}%, "
                  f"平均处理时间: {self.processing_time*1000:.2f}ms")

    def reset_frame_stats(self, frame_id, total_packets):
        if frame_id not in self.frame_stats:
            self.frame_stats[frame_id] = {
                'total_packets': total_packets,
                'received_packets': set(),
                'complete': False,
                'start_time': time.time()
            }
            
        # 检查前一帧是否完成
        if frame_id > 0 and (frame_id - 1) in self.frame_stats:
            prev_frame = self.frame_stats[frame_id - 1]
            if not prev_frame['complete']:
                lost_packets = prev_frame['total_packets'] - len(prev_frame['received_packets'])
                self.frame_history.append({
                    'total': prev_frame['total_packets'],
                    'lost': lost_packets
                })
                prev_frame['complete'] = True
        
        # 保持历史记录在窗口大小范围内
        if len(self.frame_history) > self.window_size:
            self.frame_history.pop(0)
            
        # 清理旧的帧统计信息
        old_frames = [fid for fid in self.frame_stats.keys() if fid < frame_id - self.window_size]
        for fid in old_frames:
            del self.frame_stats[fid]

    def add_packet(self, frame_id, packet_id, total_packets):
        self.reset_frame_stats(frame_id, total_packets)
        self.frame_stats[frame_id]['received_packets'].add(packet_id)
        self.total_packets = sum(fs['total_packets'] for fs in self.frame_stats.values())

class TTSManager:
    def __init__(self):
        self.engine = pyttsx3.init()
        # 设置中文语音
        voices = self.engine.getProperty('voices')
        for voice in voices:
            if 'chinese' in voice.name.lower():
                self.engine.setProperty('voice', voice.id)
                break
        self.engine.setProperty('rate', 200)  # 语速
        self.translator = Translator(to_lang="zh")
        self.last_spoken = {}  # 用于记录上次播报时间
        self.speak_interval = 3  # 同一物体的最小播报间隔（秒）
        self.engine.startLoop(False)  # 非阻塞模式

    def translate_to_chinese(self, text):
        try:
            return self.translator.translate(text)
        except Exception as e:
            print(f"翻译错误: {e}")
            return text

    def speak(self, text, object_id):
        current_time = time.time()
        # 检查是否需要播报
        if object_id in self.last_spoken:
            if current_time - self.last_spoken[object_id] < self.speak_interval:
                return
        
        # 更新播报时间
        self.last_spoken[object_id] = current_time
        
        try:
            # 翻译并播报
            chinese_text = self.translate_to_chinese(text)
            self.engine.say(chinese_text)
            self.engine.iterate()
        except Exception as e:
            print(f"TTS错误: {e}")

    def __del__(self):
        try:
            self.engine.endLoop()
        except:
            pass

class ObjectDetector:
    def __init__(self):
        # 使用YOLOv8模型
        self.model = YOLO('yolov8x.pt')
        self.conf_threshold = 0.5
        self.tts_manager = TTSManager()
        # 定义红绿灯的类别ID（在COCO数据集中，交通灯的类别ID为9）
        self.traffic_light_class = 9
        
    def detect_objects(self, frame):
        """只检测红绿灯"""
        # 图像预处理：提高分辨率和对比度
        frame = cv2.resize(frame, None, fx=1.2, fy=1.2, interpolation=cv2.INTER_LINEAR)
        
        # 增强对比度
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        enhanced = cv2.merge((cl,a,b))
        frame = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        
        # 进行物体检测
        results = self.model(frame, conf=self.conf_threshold)
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # 只处理交通灯
                cls = int(box.cls[0])
                if cls != self.traffic_light_class:
                    continue
                
                # 获取边界框坐标
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                # 获取置信度
                conf = float(box.conf[0])
                
                # 提取红绿灯区域并判断颜色
                light_roi = frame[y1:y2, x1:x2]
                if light_roi.size == 0:
                    continue
                
                # 转换到HSV颜色空间
                hsv_roi = cv2.cvtColor(light_roi, cv2.COLOR_BGR2HSV)
                
                # 定义红色和绿色的HSV范围
                red_lower1 = np.array([0, 100, 100])
                red_upper1 = np.array([10, 255, 255])
                red_lower2 = np.array([160, 100, 100])
                red_upper2 = np.array([180, 255, 255])
                green_lower = np.array([35, 100, 100])
                green_upper = np.array([85, 255, 255])
                
                # 创建红色和绿色的掩码
                red_mask1 = cv2.inRange(hsv_roi, red_lower1, red_upper1)
                red_mask2 = cv2.inRange(hsv_roi, red_lower2, red_upper2)
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                green_mask = cv2.inRange(hsv_roi, green_lower, green_upper)
                
                # 计算红色和绿色像素的数量
                red_pixels = cv2.countNonZero(red_mask)
                green_pixels = cv2.countNonZero(green_mask)
                
                # 根据像素数量判断红绿灯状态
                color = None
                if red_pixels > green_pixels and red_pixels > 50:
                    color = "红灯"
                    box_color = (0, 0, 255)  # 红色边框
                elif green_pixels > red_pixels and green_pixels > 50:
                    color = "绿灯"
                    box_color = (0, 255, 0)  # 绿色边框
                
                if color and conf > 0.7:
                    # 绘制边界框
                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 3)
                    
                    # 添加标签
                    label = f'{color} {conf:.2f}'
                    (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(frame, (x1, y1 - 30), (x1 + label_width, y1), box_color, -1)
                    cv2.putText(frame, label, (x1, y1 - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # 语音播报
                    object_id = f"traffic_light_{x1}_{y1}"
                    self.tts_manager.speak(color, object_id)
        
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

def display_stream(assembler, headless=False):
    # 初始化物体检测器
    object_detector = ObjectDetector()
    
    if not headless:
        try:
            cv2.namedWindow('Camera Stream', cv2.WINDOW_NORMAL)
            cv2.moveWindow('Camera Stream', 0, 0)
            cv2.resizeWindow('Camera Stream', 800, 600)  # 降低显示分辨率
        except Exception as e:
            print(f"创建窗口失败: {e}")
            return
    
    last_frame_time = time.time()
    min_frame_interval = 0.1  # 限制最大FPS为10
    skip_frames = 2  # 每3帧处理1帧
    frame_counter = 0
    
    while True:
        try:
            frame, fps = assembler.display_queue.get(timeout=1.0)
            
            current_time = time.time()
            frame_interval = current_time - last_frame_time
            
            # 控制帧率
            if frame_interval < min_frame_interval:
                continue
                
            frame_counter += 1
            if frame_counter % skip_frames != 0:
                continue
                
            last_frame_time = current_time
            
            # 降低处理分辨率
            if frame.shape[0] > 480:  # 如果高度大于480
                scale = 480 / frame.shape[0]
                frame = cv2.resize(frame, None, fx=scale, fy=scale, 
                                 interpolation=cv2.INTER_AREA)
            
            # 进行物体检测
            if headless:
                results = object_detector.model(frame, conf=object_detector.conf_threshold)
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])
                        if cls == object_detector.traffic_light_class and conf > 0.7:
                            x1, y1, x2, y2 = box.xyxy[0]
                            center_x = int((x1 + x2) / 2)
                            center_y = int((y1 + y2) / 2)
                            object_id = f"traffic_light_{center_x}_{center_y}"
                            # 简化颜色检测逻辑
                            light_roi = frame[int(y1):int(y2), int(x1):int(x2)]
                            if light_roi.size > 0:
                                hsv_roi = cv2.cvtColor(light_roi, cv2.COLOR_BGR2HSV)
                                red_mask = cv2.inRange(hsv_roi, np.array([0, 100, 100]), 
                                                     np.array([10, 255, 255]))
                                green_mask = cv2.inRange(hsv_roi, np.array([35, 100, 100]), 
                                                       np.array([85, 255, 255]))
                                if cv2.countNonZero(red_mask) > cv2.countNonZero(green_mask):
                                    object_detector.tts_manager.speak("红灯", object_id)
                                else:
                                    object_detector.tts_manager.speak("绿灯", object_id)
            else:
                frame = object_detector.detect_objects(frame)
                # 简化显示的统计信息
                cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                try:
                    cv2.imshow('Camera Stream', frame)
                except cv2.error as e:
                    print(f"显示帧失败: {e}")
                    continue
            
            if not headless:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f'capture_{timestamp}.jpg', frame)
                    print(f"截图已保存: capture_{timestamp}.jpg")
        except Empty:
            continue
        except Exception as e:
            print(f"处理错误: {e}")
            time.sleep(0.1)
            continue

    if not headless:
        try:
            cv2.destroyAllWindows()
        except:
            pass

def main():
    # 添加命令行参数
    parser = argparse.ArgumentParser(description='ESP32摄像头客户端')
    parser.add_argument('--headless', action='store_true', help='无显示模式，只进行物体检测和语音播报')
    args = parser.parse_args()
    
    assembler = FrameAssembler()
    
    # 创建接收线程
    receive_thread = threading.Thread(target=receive_stream, args=(assembler,))
    receive_thread.daemon = True
    receive_thread.start()
    
    # 在主线程中运行显示
    try:
        display_stream(assembler, args.headless)
    except KeyboardInterrupt:
        print("\n正在退出...")
    finally:
        if not args.headless:
            try:
                cv2.destroyAllWindows()
            except:
                pass

if __name__ == "__main__":
    main() 