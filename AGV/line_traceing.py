import sys
import os
import time
import json
import RPi.GPIO as GPIO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(BASE_DIR, ".."))
COMMON_DIR = os.path.join(PROJECT_DIR, "common")
TRACKING_DIR = os.path.join(PROJECT_DIR, "2.Hardware Control course", "07.Tracking")

sys.path.insert(0, COMMON_DIR)
sys.path.insert(0, TRACKING_DIR)

from agent_base import AgentBase
from topics import FORKLIFT_COMMAND, FORKLIFT_RESULT, FORKLIFT_STATUS
from YB_Pcb_Car import YB_Pcb_Car

BROKER_HOST = "192.168.0.21"
DEVICE_ID = "car_1"
REGION = "서울"

L1_PIN = 13
L2_PIN = 15
R1_PIN = 11
R2_PIN = 7

BASE_SPEED = 40
TURN_SPEED = 35
SOFT = 28
HARD = 20
CORRECT_SPEED = 20

# 지게 각도
FORK_INIT = 125
FORK_UP = 170
FORK_DOWN = 125

TARGET_ROUTE = {
    "서울_VINYL": ["STRAIGHT"],
    "서울_BOX": ["RIGHT", "LEFT"],
    "서울_BOX_FRAGILE": ["RIGHT", "LEFT"],

    "CAR_SLOT": ["LEFT", "RIGHT"],
    "STORAGE_EMPTY": ["STRAIGHT"],

    "HOME": ["FORCE_STRAIGHT"],
    "EXIT": ["FORCE_STRAIGHT"],
}
RETURN_ROUTE = {
    "서울_VINYL": ["RIGHT", "RIGHT_STRONG"],
    "서울_BOX": ["RIGHT", "RIGHT"],
    "서울_BOX_FRAGILE": ["LEFT", "RIGHT"],
}

TURN_SPEED_CROSS = 40
LEFT_TURN_TIME = 0.75
RIGHT_TURN_TIME = 0.65
STRAIGHT_PASS_TIME = 0.9
END_DETECT_TIME = 0.8


class SeoulAgvAgent(AgentBase):
    def __init__(self):
        super().__init__(
            broker_host=BROKER_HOST,
            device_id=DEVICE_ID,
            device_type="FORKLIFT",
            command_topic=FORKLIFT_COMMAND,
            result_topic=FORKLIFT_RESULT,
        )


        self.running = False

        self.last_turn = "STRAIGHT"

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(L1_PIN, GPIO.IN)
        GPIO.setup(L2_PIN, GPIO.IN)
        GPIO.setup(R1_PIN, GPIO.IN)
        GPIO.setup(R2_PIN, GPIO.IN)

        self.car = YB_Pcb_Car()

        self.car.Ctrl_Servo(1, FORK_INIT)
        time.sleep(1)

        self._stop()
        print("[car_1] 하드웨어 초기화 완료")

    def _publish_status(self, status):
        self.client.publish(
            FORKLIFT_STATUS,
            json.dumps({
                "device_id": self.device_id,
                "status": status
            }, ensure_ascii=False)
        )
        print(f"[STATUS] {status}")

    def _stop(self):
        # self.running = False
        self.car.Car_Stop()

    def _line_trace_step(self):
        L1 = GPIO.input(L1_PIN)
        L2 = GPIO.input(L2_PIN)
        R1 = GPIO.input(R1_PIN)
        R2 = GPIO.input(R2_PIN)

        # 정중앙
        if L2 == 0 and R1 == 0 and L1 == 1 and R2 == 1:
            self.last_turn = "STRAIGHT"
            self.car.Car_Run(BASE_SPEED, BASE_SPEED)

        # 왼쪽 많이 벗어남
        elif L1 == 0 and L2 == 0:
            self.last_turn = "LEFT"
            self.car.Car_Run(HARD, BASE_SPEED)

        # 왼쪽 살짝 벗어남
        elif L2 == 0:
            self.last_turn = "LEFT"
            self.car.Car_Run(SOFT, BASE_SPEED)

        # 오른쪽 많이 벗어남
        elif R1 == 0 and R2 == 0:
            self.last_turn = "RIGHT"
            self.car.Car_Run(BASE_SPEED, HARD)

        # 오른쪽 살짝 벗어남
        elif R1 == 0:
            self.last_turn = "RIGHT"
            self.car.Car_Run(BASE_SPEED, SOFT)

        # 완전 왼쪽 끝
        elif L1 == 0:
            self.last_turn = "LEFT"
            self.car.Car_Run(18, BASE_SPEED)

        # 완전 오른쪽 끝
        elif R2 == 0:
            self.last_turn = "RIGHT"
            self.car.Car_Run(BASE_SPEED, 18)

        # 라인 놓침
        else:
            if self.last_turn == "LEFT":
                self.car.Car_Run(8, BASE_SPEED)
            elif self.last_turn == "RIGHT":
                self.car.Car_Run(BASE_SPEED, 8)
            else:
                self.car.Car_Run(25, 25)


    def _read_sensors(self):
        L1 = GPIO.input(L1_PIN)
        L2 = GPIO.input(L2_PIN)
        R1 = GPIO.input(R1_PIN)
        R2 = GPIO.input(R2_PIN)
        return L1, L2, R1, R2


    def _is_cross(self, L1, L2, R1, R2):
        # 교차점: 센서 4개가 전부 검은색
        return L1 == 0 and L2 == 0 and R1 == 0 and R2 == 0


    def _turn_at_cross(self, direction):
        print(f"[car_1] 교차점 방향 선택: {direction}")

        if direction == "LEFT":
            self.car.Car_Left(TURN_SPEED_CROSS, TURN_SPEED_CROSS)
            time.sleep(LEFT_TURN_TIME)

        elif direction == "RIGHT":
            self.car.Car_Right(TURN_SPEED_CROSS, TURN_SPEED_CROSS)
            time.sleep(RIGHT_TURN_TIME)

        elif direction == "RIGHT_STRONG":
            print("[car_1] 강한 우회전")
            self.car.Car_Right(TURN_SPEED_CROSS, TURN_SPEED_CROSS)
            time.sleep(1.15)

        elif direction == "STRAIGHT":
            self.last_turn = "STRAIGHT"
            self.car.Car_Run(30,30)
            time.sleep(0.8)

        elif direction == "FORCE_STRAIGHT":
            self.last_turn = "STRAIGHT"
            self.car.Car_Run(35, 35)
            time.sleep(0.8)

        self.car.Car_Stop()
        time.sleep(0.2)

        if direction in ["LEFT", "RIGHT"]:
            self._find_line_after_turn(direction)

    def _find_line_after_turn(self, direction, timeout=2.0):
        start = time.time()

        while time.time() - start < timeout:
            L1, L2, R1, R2 = self._read_sensors()

            if L2 == 0 and R1 == 0:
                print("[car_1] 라인 감지 완료")

                self.car.Car_Run(15, 15)
                time.sleep(0.05)

                self.car.Car_Stop()
                return True

            if direction == "LEFT":
                self.car.Car_Left(18, 18)
            elif direction in ["RIGHT", "RIGHT_STRONG"]:
                self.car.Car_Right(18, 18)
            else:
                self.car.Car_Run(25, 25)

            time.sleep(0.03)

        print("[car_1] 새 라인 찾기 실패")
        self.car.Car_Stop()
        return False

    def _line_trace_to(self, target):
        route = TARGET_ROUTE.get(target, ["STRAIGHT"])

        print(f"[car_1] {target} 방향 라인트레이싱 시작 route={route}")
        self.running = True

        turn_index = 0
        cross_locked = False
        end_start_time = None

        try:
            while self.running:
                L1, L2, R1, R2 = self._read_sensors()
                # print(f"L1={L1} L2={L2} R1={R1} R2={R2}")

                is_cross = self._is_cross(L1, L2, R1, R2)

                # route를 다 처리한 뒤 검은 도착 마커를 만나면 바로 정지
                if is_cross and turn_index >= len(route):
                    print(f"[car_1] {target} 도착 마커 감지")
                    break

                if is_cross and not cross_locked:
                    cross_locked = True

                    if turn_index < len(route):
                        direction = route[turn_index]
                        turn_index += 1
                        self._turn_at_cross(direction)

                        if direction not in ["STRAIGHT", "FORCE_STRAIGHT"]:
                            self.car.Car_Run(BASE_SPEED, BASE_SPEED)
                            time.sleep(0.1)

                        continue

                    else:
                        # route를 다 처리한 뒤 다시 검은 마커를 만나면 목적지 도착으로 판단
                        if end_start_time is None:
                            end_start_time = time.time()

                        if time.time() - end_start_time >= END_DETECT_TIME:
                            print(f"[car_1] {target} 도착 마커 감지")
                            break

                elif not is_cross:
                    cross_locked = False
                    end_start_time = None
                    self._line_trace_step()

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("[car_1] 주행 중 Ctrl+C 감지")
            self._stop()
            raise

        self.running = False
        self._stop()
        print(f"[car_1] {target} 도착 처리")

    
    def _line_trace_by_route(self, target, route):
        old_route = TARGET_ROUTE.get(target)
        TARGET_ROUTE[target] = route
        self._line_trace_to(target)

        if old_route is not None:
            TARGET_ROUTE[target] = old_route


    def _pickup_pallet(self):
        print("[car_1] 팔레트 픽업 동작")
        print(f"[SERVO] 지게 올림: {FORK_UP}도")

        self.car.Car_Stop()
        time.sleep(0.5)

        self.car.Ctrl_Servo(1, FORK_UP)
        time.sleep(2.0)

    def _drop_pallet(self):
        print("[car_1] 팔레트 드롭 동작")
        print(f"[SERVO] 지게 내림: {FORK_DOWN}도")

        self.car.Car_Stop()
        time.sleep(0.5)

        # 1번 서보: 지게 내림 각도
        self.car.Ctrl_Servo(1, FORK_DOWN)
        time.sleep(2.0)


    def _back_out(self, seconds=1.4):
        print("[car_1] 후진해서 라인 복귀")
        self.car.Car_Back(30, 30)
        time.sleep(seconds)
        self.car.Car_Stop()
        time.sleep(0.2)

    def _turn_around_pickup(self):
        print("[car_1] 픽업 후 180도 회전")
        self.car.Car_Right(65, 65)
        time.sleep(1.5)
        self.car.Car_Stop()
        time.sleep(0.2)

        self.car.Car_Run(20, 20)
        time.sleep(0.35)
        self.car.Car_Stop()
        time.sleep(0.1)
 
        # self._find_line_after_turn("RIGHT", timeout =2.5 )

    def _turn_around_pickup_vinyl(self):
        print("[car_1] 픽업 후 180도 회전")
        self.car.Car_Right(65, 65)
        time.sleep(1.8)
        self.car.Car_Stop()
        time.sleep(0.2)

        self.car.Car_Run(20, 20)
        time.sleep(0.35)

        self.car.Car_Stop()
        time.sleep(0.1)

    def _turn_around_car_slot_drop(self):
        print("[car_1] 다 찬 분류함 놓고 180도 회전")
        self.car.Car_Left(65, 65)
        time.sleep(1.6)
        self.car.Car_Stop()
        time.sleep(0.2)
 
    def _turn_around_drop(self):
        print("[car_1] 드롭 후 180도 회전")
        self.car.Car_Left(65, 65)
        time.sleep(1.5)
        self.car.Car_Stop()
        time.sleep(0.2)

    def _turn_around_final(self):
        print("[car_1] 마지막 HOME 복귀 전 180도 회전")
        self.car.Car_Left(65, 65)
        time.sleep(1.7)
        self.car.Car_Stop()
        time.sleep(0.2)  

    def handle_emergency(self):
        print("[car_1] 비상정지 수신")
        self._stop()
        self._publish_status("FORKLIFT_EMERGENCY_STOP")

    def handle_command(self, data):
        cmd = data.get("command")
        print(f"[car_1] 명령 수신 payload={data}")

        if cmd == "MOVE_PALLET":
            from_pos = data.get("from", "")
            to_pos = data.get("to", "CAR_SLOT")

            if from_pos != "서울_VINYL":
                print(f"[car_1] 비닐만 처리 → {from_pos} 무시")
                self._publish_status("FORKLIFT_READY")
                return

            # if from_pos in ["서울_BOX_FRAGILE", "서울_BOX"]:
            #     print("[car_1] 파손 분류함은 시연에서 제외 → 명령 무시")
            #     self._publish_status("FORKLIFT_READY")
            #     return

            if not from_pos.startswith(REGION):
                print(f"[car_1] 무시: 서울 담당 아님 from={from_pos}")
                return

            if not from_pos.startswith(REGION):
                print(f"[car_1] 무시: 서울 담당 아님 from={from_pos}")
                return

            try:
                self._publish_status("FORKLIFT_MOVING")
                self._line_trace_to(from_pos)

                self._publish_status("FORKLIFT_ARRIVED")

                self._publish_status("FORKLIFT_LOADING")
                self._pickup_pallet()

                # 팔레트 들고 뒤로 빠져나오기
                self._back_out(1.5)
                self._turn_around_pickup_vinyl()


                # self._find_line_after_turn("STRAIGHT")
                print("[car_1] 이제 CAR_SLOT으로 이동")

                self._line_trace_to(to_pos)

                self.car.Car_Run(15, 15)
                time.sleep(0.2)
                self.car.Car_Stop()
                time.sleep(0.1)

                self._drop_pallet()
                self._back_out(1.0)
                self._turn_around_car_slot_drop()

                self._publish_status("FORKLIFT_RETURNING")
                self._line_trace_to("STORAGE_EMPTY")
                self._pickup_pallet()
                self._back_out(0.5)
                self._turn_around_pickup()

                # self._line_trace_to(from_pos)
                # self._drop_pallet()

                # 빈 분류함을 원래 일반/비닐/파손 위치로 다시 가져다 놓기
                return_route = RETURN_ROUTE.get(from_pos, TARGET_ROUTE.get(from_pos, ["STRAIGHT"]))

                self._line_trace_by_route(from_pos, return_route)

                self._drop_pallet()

                self._back_out(1.2)
                self._turn_around_final()

                self._line_trace_to("HOME")

                print("[car_1] HOME 도착 → 마지막 180도 회전 후 정지")
                self._turn_around_drop()
                self._stop()

                self.publish_result(
                    "MOVE_PALLET",
                    "DONE",
                    device_id=self.device_id,
                    **{"from": from_pos, "to": to_pos}
                )

                self._publish_status("FORKLIFT_READY")
                print(f"[car_1] 미션 완료: {from_pos} -> {to_pos}")

            except Exception as e:
                print(f"[car_1] 오류: {e}")
                self._stop()
                self.publish_result(
                    "MOVE_PALLET",
                    "FAIL",
                    device_id=self.device_id,
                    reason=str(e)
                )
                self._publish_status("FORKLIFT_ERROR")

            return

        if cmd == "DEPART":
            print("[car_1] DEPART 수신")
            self._publish_status("FORKLIFT_DEPARTED")
            self._line_trace_to("EXIT")
            return

        print(f"[car_1] 알 수 없는 명령: {cmd}")


if __name__ == "__main__":
    agent = None
    try:
        agent = SeoulAgvAgent()
        agent.connect()
    except KeyboardInterrupt:
        print("[car_1] Ctrl+C 종료")
        if agent:
            agent._stop()
        GPIO.cleanup()