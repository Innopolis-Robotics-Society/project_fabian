#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import struct
from typing import Optional

import lcm  # sudo apt-get install liblcm-dev python3-lcm
import rclpy
from rclpy.node import Node

from f_interfaces.msg import FoxCommand, PersonBodyArray


# Каналы — из include/unitree_legged_sdk/lcm.h
HIGH_CMD_CHANNEL = "LCM_High_Cmd"
HIGH_STATE_CHANNEL = "LCM_High_State"

# Формат HighCmd — строго по comm.h (#pragma pack(1))
HIGH_CMD_FMT = "<B H H I B B B B f f 2f 3f 2f f 12B 40B I I"
HIGH_CMD_SIZE = struct.calcsize(HIGH_CMD_FMT)


def pack_highcmd(
    mode: int = 2,
    gait_type: int = 1,
    speed_level: int = 0,
    vx: float = 0.3,
    vy: float = 0.0,
    yaw_speed: float = 0.0,
    foot_raise_height: float = 0.08,
    body_height: float = 0.0,
    euler_roll: float = 0.0,
    euler_pitch: float = 0.0,
    euler_yaw: float = 0.0,
):
    """
    Собираем HighCmd в байты по C-структуре из comm.h.
    Служебные поля и CRC = 0 — UDP-класс в C++ SDK сам пересчитает CRC.
    """

    levelFlag = 0x00       # HIGHLEVEL
    commVersion = 0
    robotID = 0
    SN = 0
    bandWidth = 0

    mode_u8 = mode & 0xFF
    gait_u8 = gait_type & 0xFF
    speed_u8 = speed_level & 0xFF

    footRaiseHeight = float(foot_raise_height)
    bodyHeight = float(body_height)

    # опечатка в C: postion[2]
    postion = (0.0, 0.0)

    # roll, pitch, yaw (используются для mode=1)
    euler = (float(euler_roll), float(euler_pitch), float(euler_yaw))

    velocity = (float(vx), float(vy))
    yawSpeed = float(yaw_speed)

    leds = [0] * 12           # 4 LED * RGB
    wirelessRemote = [0] * 40

    reserve = 0
    crc = 0  # будет перезаписан на стороне C++

    data = struct.pack(
        HIGH_CMD_FMT,
        levelFlag,
        commVersion,
        robotID,
        SN,
        bandWidth,
        mode_u8,
        gait_u8,
        speed_u8,
        footRaiseHeight,
        bodyHeight,
        *postion,
        *euler,
        *velocity,
        yawSpeed,
        *leds,
        *wirelessRemote,
        reserve,
        crc,
    )

    assert len(data) == HIGH_CMD_SIZE
    return data


class A1LCMClient(object):
    def __init__(self, lcm_url=None):
        """
        Если lcm_url=None — используется LCM_DEFAULT_URL или дефолт.
        Главное, чтобы совпадал с тем, что использует lcm_server_high.
        """
        if lcm_url is None:
            self.lc = lcm.LCM()
        else:
            self.lc = lcm.LCM(lcm_url)

        self._state_lock = threading.Lock()
        self._last_state_raw: Optional[bytes] = None

        self.lc.subscribe(HIGH_STATE_CHANNEL, self._state_handler)

        self._stop = False
        self._rx_thread = threading.Thread(target=self._lcm_loop)
        self._rx_thread.daemon = True
        self._rx_thread.start()

    def _state_handler(self, channel, data):
        # сохраняем сырые байты HighState (разбор можно добавить позже)
        with self._state_lock:
            self._last_state_raw = data

    def _lcm_loop(self):
        while not self._stop:
            self.lc.handle_timeout(100)  # 100 ms ожидание LCM

    def stop(self):
        self._stop = True
        self._rx_thread.join()

    def send_raw_cmd(self, cmd_bytes: bytes):
        assert len(cmd_bytes) == HIGH_CMD_SIZE
        self.lc.publish(HIGH_CMD_CHANNEL, cmd_bytes)

    def get_last_state_raw(self) -> Optional[bytes]:
        with self._state_lock:
            return self._last_state_raw


class A1LCMControlNode(Node):
    """
    Управление A1 через LCM по high-level командам FoxCommand.command (нижний регистр):

      - "salute"       : глубокий поклон (mode=1, euler_pitch)
      - "come to me"   : повернуться к человеку по bbox и пройти к нему 1 м
      - "come closer"  : шаг назад примерно на 0.5 м и остановка
      - "walking"      : просто идти вперёд
      - "jumping"      : 1 c вперёд, 1 c назад (циклично)

    Логика команд:

      - Любая команда при первом приходе запускает действие.
      - Действие живёт хотя бы min_action_duration секунд, даже если FoxCommand больше не приходит.
      - Новая команда всегда немедленно перебивает текущую (нет очереди).
      - После min_action_duration и при отсутствии свежих FoxCommand робот встаёт.
    """

    def __init__(self):
        super(A1LCMControlNode, self).__init__("a1_lcm_controller")

        # Параметры
        self.declare_parameter("timer_dt", 0.01)            # частота LCM-команд
        self.declare_parameter("walk_vx", 0.3)              # скорость вперёд
        self.declare_parameter("jump_vx", 0.3)              # скорость вперёд/назад при jumping
        self.declare_parameter("wave_yaw_speed", 0.5)       # пока не используем
        self.declare_parameter("command_topic", "/f_fox_command/command")
        self.declare_parameter("stand_body_height", 0.0)
        self.declare_parameter("action_timeout", 1.5)       # СЕКУНД без сообщений FoxCommand
        # Увеличим, чтобы влезло "повернуться + пройти 1 м"
        self.declare_parameter("min_action_duration", 6.0)  # гарантированная длительность действия, сек

        # Для "come closer" (отступ назад на ~0.5 м)
        self.declare_parameter("step_back_vx", 0.25)        # модуль скорости назад

        # PersonBody / "come to me"
        self.declare_parameter("person_topic", "/person_bodies")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 384)
        self.declare_parameter("person_timeout", 0.5)

        # Параметры поворота и прохода на "come to me"
        self.declare_parameter("come_to_me_yaw_speed", 0.5)         # рад/с
        self.declare_parameter("come_to_me_max_angle", 0.7)         # рад, макс поворот ~40°
        self.declare_parameter("come_to_me_dead_zone", 0.1)         # [-1,1] по X, зона без поворота
        self.declare_parameter("come_to_me_forward_distance", 1.0)  # м
        self.declare_parameter("come_to_me_vx", 0.25)               # м/с вперёд

        self.dt = float(self.get_parameter("timer_dt").value)
        self.walk_vx = float(self.get_parameter("walk_vx").value)
        self.jump_vx = float(self.get_parameter("jump_vx").value)
        self.wave_yaw_speed = float(self.get_parameter("wave_yaw_speed").value)
        self.command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self.stand_body_height = float(self.get_parameter("stand_body_height").value)
        self.action_timeout = float(self.get_parameter("action_timeout").value)
        self.min_action_duration = float(self.get_parameter("min_action_duration").value)

        self.step_back_vx = float(self.get_parameter("step_back_vx").value)

        self.person_topic = self.get_parameter("person_topic").get_parameter_value().string_value
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.person_timeout = float(self.get_parameter("person_timeout").value)

        self.come_to_me_yaw_speed = float(self.get_parameter("come_to_me_yaw_speed").value)
        self.come_to_me_max_angle = float(self.get_parameter("come_to_me_max_angle").value)
        self.come_to_me_dead_zone = float(self.get_parameter("come_to_me_dead_zone").value)
        self.come_to_me_forward_distance = float(self.get_parameter("come_to_me_forward_distance").value)
        self.come_to_me_vx = float(self.get_parameter("come_to_me_vx").value)

        # LCM-клиент
        self.lcm_client = A1LCMClient()

        # Текущее действие
        self.current_label: Optional[str] = None
        self.action_start_time: Optional[float] = None

        # Время последнего ПОЛУЧЕННОГО FoxCommand
        self.last_msg_time: Optional[float] = None

        # Последний человек (для "come to me")
        self.last_person_time: Optional[float] = None
        self.last_person_cx: Optional[float] = None
        self.last_person_cy: Optional[float] = None
        self.last_person_h: Optional[float] = None
        self.last_person_score: Optional[float] = None

        # Состояние для "come to me"
        self.come_to_me_turn_duration: float = 0.0
        self.come_to_me_turn_dir: int = 0   # -1, 0, +1
        self.come_to_me_forward_duration: float = 0.0
        self.come_to_me_initialized: bool = False

        # Подписка на FoxCommand
        self.sub_command = self.create_subscription(
            FoxCommand,
            self.command_topic,
            self.command_cb,
            10,
        )

        # Подписка на PersonBodyArray
        self.sub_person = self.create_subscription(
            PersonBodyArray,
            self.person_topic,
            self.person_cb,
            10,
        )

        self.get_logger().info(
            f"A1LCMControlNode started. "
            f"dt={self.dt}, walk_vx={self.walk_vx}, jump_vx={self.jump_vx}, "
            f"command_topic='{self.command_topic}', "
            f"person_topic='{self.person_topic}', "
            f"image_size=({self.image_width}x{self.image_height}), "
            f"action_timeout={self.action_timeout}, "
            f"min_action_duration={self.min_action_duration}"
        )

        # Таймер управления
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self._step = 0
        self._last_timeout_state = False  # для логов перехода в idle

    # ====== callbacks ======

    def command_cb(self, msg: FoxCommand):
        now = self.get_clock().now().nanoseconds / 1e9
        new_label = (msg.command or "").strip().lower()
        if not new_label:
            # пустая команда: игнорируем, но фиксируем факт прихода сообщения
            self.last_msg_time = now
            return

        # Нормализация алиасов:
        # "come to me" -> выравниваемся по bbox и идём 1 м
        if new_label in ("come to me", "come_to_me"):
            new_label = "come_to_me"
        # "come closer" -> шаг назад на 0.5 м
        elif new_label in ("come closer", "come_closer"):
            new_label = "step_back"

        # Переинициализируем действие при смене команды
        if (self.current_label is None) or (new_label != self.current_label):
            self.current_label = new_label
            self.action_start_time = now

            # Инициализация сценария "come to me" (фиксируем человека и времена манёвров)
            if new_label == "come_to_me":
                self._init_come_to_me(now)

            desc = (msg.description or "").strip()
            if desc:
                self.get_logger().info(
                    f"New action label: '{self.current_label}', "
                    f"description='{desc}'"
                )
            else:
                self.get_logger().info(
                    f"New action label: '{self.current_label}'"
                )

        # Обновляем "последнее время команды"
        self.last_msg_time = now

    def person_cb(self, msg: PersonBodyArray):
        """
        Выбираем PersonBody с максимальным score и валидным bbox,
        сохраняем центр и высоту bbox.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        best_person = None
        best_score = -1.0

        for p in msg.persons:
            if len(p.bbox) >= 4 and p.score > best_score:
                best_score = p.score
                best_person = p

        if best_person is None:
            return

        x, y, w, h = best_person.bbox[0:4]
        cx = x + w / 2.0
        cy = y + h / 2.0

        self.last_person_time = now
        self.last_person_cx = cx
        self.last_person_cy = cy
        self.last_person_h = h
        self.last_person_score = best_score

    # ====== init for specific actions ======

    def _init_come_to_me(self, now: float):
        """
        При первом приходе команды "come to me":
          - фиксируем, где человек в кадре (по последнему PersonBody),
          - считаем длительность поворота,
          - считаем длительность прохода на 1 м.
        """
        self.come_to_me_initialized = True
        self.come_to_me_turn_dir = 0
        self.come_to_me_turn_duration = 0.0

        # Всегда планируем пройти 1 м вперёд (параметр)
        distance = self.come_to_me_forward_distance
        speed = abs(self.come_to_me_vx) if self.come_to_me_vx != 0.0 else 0.25
        self.come_to_me_forward_duration = distance / speed

        # Если нет свежего человека — идём просто прямо
        if (
            self.last_person_time is None
            or (now - self.last_person_time) > self.person_timeout
            or self.last_person_cx is None
        ):
            self.get_logger().warn(
                "come_to_me: no fresh PersonBody, will walk straight."
            )
            return

        # Смещение по X относительно центра кадра
        img_cx = self.image_width / 2.0
        err_x = (self.last_person_cx - img_cx) / img_cx  # [-1,1]

        if abs(err_x) < self.come_to_me_dead_zone:
            # Почти по центру — поворот не нужен
            self.get_logger().info(
                f"come_to_me: person nearly centered (err_x={err_x:.3f}), no turn."
            )
            return

        # Угол поворота пропорционален err_x, но не больше come_to_me_max_angle
        k = self.come_to_me_max_angle
        # saturate err_x в [-1,1]
        if err_x > 1.0:
            err_clamped = 1.0
        elif err_x < -1.0:
            err_clamped = -1.0
        else:
            err_clamped = err_x

        angle = k * err_clamped  # рад
        self.come_to_me_turn_dir = 1 if angle > 0 else -1

        yaw_speed = abs(self.come_to_me_yaw_speed) if self.come_to_me_yaw_speed != 0.0 else 0.5
        self.come_to_me_turn_duration = abs(angle) / yaw_speed

        self.get_logger().info(
            f"come_to_me init: err_x={err_x:.3f}, angle={angle:.3f} rad, "
            f"turn_duration={self.come_to_me_turn_duration:.2f}s, "
            f"forward_duration={self.come_to_me_forward_duration:.2f}s"
        )

    # ====== main control loop ======

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Свежесть FoxCommand (по timeout)
        if self.last_msg_time is None:
            raw_active = False
        else:
            raw_active = (now - self.last_msg_time) <= self.action_timeout

        # Время с начала текущего действия
        if self.current_label is None or self.action_start_time is None:
            elapsed = 0.0
        else:
            elapsed = now - self.action_start_time

        # Гарантия минимальной длительности действия
        if self.current_label is None or self.action_start_time is None:
            min_duration_active = False
        else:
            min_duration_active = elapsed < self.min_action_duration

        # Команда активна, если:
        #   - приходят свежие FoxCommand ИЛИ
        #   - не истёк минимальный срок действия
        command_active = raw_active or min_duration_active
        label = self.current_label if command_active else None

        # Логируем переход в idle
        if command_active != self._last_timeout_state and not command_active:
            self.get_logger().warn(
                f"No FoxCommand and action duration > {self.min_action_duration}s. "
                f"Switching to stand."
            )
        self._last_timeout_state = command_active

        # Выбор команды
        if label is None:
            cmd = self._cmd_stand()
            phase = "idle"

        elif label == "walking":
            cmd = self._cmd_walking()
            phase = "walking"

        elif label == "salute":
            cmd = self._cmd_salute(elapsed)
            phase = "salute"

        elif label == "jumping":
            cmd = self._cmd_jumping(elapsed)
            phase = "jumping"

        elif label == "step_back":
            cmd = self._cmd_step_back(elapsed)
            phase = "step_back"

        elif label == "come_to_me":
            cmd = self._cmd_come_to_me(elapsed)
            phase = "come_to_me"

        else:
            cmd = self._cmd_stand()
            phase = f"stand_unknown('{label}')"

        # Отправка в LCM
        self.lcm_client.send_raw_cmd(cmd)

        # Лог раз в ~1 секунду
        self._step += 1
        if self._step % int(max(1, round(1.0 / self.dt))) == 0:
            self.get_logger().info(
                f"phase={phase}, label={label}, elapsed={elapsed:.2f}"
            )

    # ====== motion patterns ======

    def _cmd_stand(self):
        # force stand
        return pack_highcmd(
            mode=1,
            gait_type=0,
            vx=0.0,
            vy=0.0,
            yaw_speed=0.0,
            body_height=self.stand_body_height,
        )

    def _cmd_walking(self):
        # ходьба вперёд
        return pack_highcmd(
            mode=2,
            gait_type=1,
            vx=self.walk_vx,
            vy=0.0,
            yaw_speed=0.0,
            foot_raise_height=0.08,
            body_height=self.stand_body_height,
        )

    def _cmd_salute(self, elapsed: float):
        """
        Глубокий поклон за счёт euler_pitch.
        Пример: 2-секундный цикл: 1 c наклон вперёд, 1 c возврат в ноль.
        """
        cycle = 2.0
        phase = elapsed % cycle

        # Более глубокий наклон, например -0.6 рад
        if phase < 1.0:
            pitch = -0.6
        else:
            pitch = 0.0

        return pack_highcmd(
            mode=1,
            gait_type=0,
            vx=0.0,
            vy=0.0,
            yaw_speed=0.0,
            body_height=self.stand_body_height,
            euler_pitch=pitch,
        )

    def _cmd_jumping(self, elapsed: float):
        # jumping: 1 c вперёд, 1 c назад
        cycle = 2.0
        phase = elapsed % cycle

        if phase < 1.0:
            vx = +self.jump_vx
        else:
            vx = -self.jump_vx

        return pack_highcmd(
            mode=2,
            gait_type=1,
            vx=vx,
            vy=0.0,
            yaw_speed=0.0,
            foot_raise_height=0.10,
            body_height=self.stand_body_height,
        )

    def _cmd_step_back(self, elapsed: float):
        """
        "come closer": отойти назад примерно на 0.5 м и остановиться.
        s = 0.5 м, v = step_back_vx => t = s / v.
        Пока t не истёк — идём назад, потом встаём.
        """
        distance = 0.5
        speed = abs(self.step_back_vx) if self.step_back_vx != 0.0 else 0.25
        duration = distance / speed

        if elapsed < duration:
            vx = -speed  # назад
            return pack_highcmd(
                mode=2,
                gait_type=1,
                vx=vx,
                vy=0.0,
                yaw_speed=0.0,
                foot_raise_height=0.08,
                body_height=self.stand_body_height,
            )
        else:
            # дистанция пройдена — стоим
            return self._cmd_stand()

    def _cmd_come_to_me(self, elapsed: float):
        """
        "come to me": сначала повернуться к человеку, потом пройти 1 м вперёд.
        Логика:
          - 0..turn_duration      : поворот на месте
          - turn_duration..+Twalk : движение вперёд
          - дальше                : стоим
        """
        # Если по какой-то причине не успели инициализировать — fallback: как walking на 1 м
        if not self.come_to_me_initialized:
            distance = self.come_to_me_forward_distance
            speed = abs(self.come_to_me_vx) if self.come_to_me_vx != 0.0 else 0.25
            duration = distance / speed
            if elapsed < duration:
                return pack_highcmd(
                    mode=2,
                    gait_type=1,
                    vx=speed,
                    vy=0.0,
                    yaw_speed=0.0,
                    foot_raise_height=0.08,
                    body_height=self.stand_body_height,
                )
            else:
                return self._cmd_stand()

        t_turn = self.come_to_me_turn_duration
        t_fwd = self.come_to_me_forward_duration

        # Поворот
        if (elapsed < t_turn) and (self.come_to_me_turn_dir != 0):
            yaw = self.come_to_me_yaw_speed * float(self.come_to_me_turn_dir)
            return pack_highcmd(
                mode=2,
                gait_type=1,
                vx=0.0,
                vy=0.0,
                yaw_speed=yaw,
                foot_raise_height=0.08,
                body_height=self.stand_body_height,
            )

        # Движение вперёд
        if elapsed < (t_turn + t_fwd):
            vx = abs(self.come_to_me_vx) if self.come_to_me_vx != 0.0 else 0.25
            return pack_highcmd(
                mode=2,
                gait_type=1,
                vx=vx,
                vy=0.0,
                yaw_speed=0.0,
                foot_raise_height=0.08,
                body_height=self.stand_body_height,
            )

        # Завершили манёвр — стоим
        return self._cmd_stand()

    # ====== shutdown ======

    def destroy_node(self):
        self.lcm_client.stop()
        super(A1LCMControlNode, self).destroy_node()


def main(args=None):
        rclpy.init(args=args)
        node = A1LCMControlNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
