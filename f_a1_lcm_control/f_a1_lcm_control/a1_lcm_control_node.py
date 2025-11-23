#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import struct
from typing import Optional

import lcm  # sudo apt-get install liblcm-dev python3-lcm
import rclpy
from rclpy.node import Node

from f_interfaces.msg import FoxCommand


# Каналы — из include/unitree_legged_sdk/lcm.h
HIGH_CMD_CHANNEL = "LCM_High_Cmd"
HIGH_STATE_CHANNEL = "LCM_High_State"

# Формат HighCmd — строго по comm.h (#pragma pack(1))
HIGH_CMD_FMT = "<B H H I B B B B f f 2f 3f 2f f 12B 40B I I"
HIGH_CMD_SIZE = struct.calcsize(HIGH_CMD_FMT)


def pack_highcmd(
    mode=2,
    gait_type=1,
    speed_level=0,
    vx=0.3,
    vy=0.0,
    yaw_speed=0.0,
    foot_raise_height=0.08,
    body_height=0.0,
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

    euler = (0.0, 0.0, 0.0)   # roll, pitch, yaw (для mode=1)
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
        self._last_state_raw = None  # type: Optional[bytes]

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

    def send_raw_cmd(self, cmd_bytes):
        assert len(cmd_bytes) == HIGH_CMD_SIZE
        self.lc.publish(HIGH_CMD_CHANNEL, cmd_bytes)

    def get_last_state_raw(self):
        # type: () -> Optional[bytes]
        with self._state_lock:
            return self._last_state_raw


class A1LCMControlNode(Node):
    """
    Управление A1 через LCM по high-level командам на основе PersonAction.label:

      - "walking": идти вперёд (mode=2, gaitType=1, vx>0)
      - "waving": крутиться влево/вправо (yawSpeed >0,<0 поочерёдно)
      - "jumping": 1 c вперёд, 1 c назад (vx>0, потом vx<0)
      - иначе или при отсутствии команд дольше action_timeout: force stand (mode=1)

    Важные моменты:

      1) Действие переинициализируется ТОЛЬКО:
           - при смене label (walking→waving и т.п.), ИЛИ
           - если до этого команда была неактивна (таймаут, робот стоял),
             даже если новый label совпадает со старым.

      2) Если сообщений PersonAction нет дольше action_timeout секунд —
         считаем, что команды больше нет → робот стоит.
    """

    def __init__(self):
        super(A1LCMControlNode, self).__init__("a1_lcm_controller")

        # Параметры
        self.declare_parameter("timer_dt", 0.01)            # частота LCM-команд
        self.declare_parameter("walk_vx", 0.3)              # walking speed
        self.declare_parameter("jump_vx", 0.3)              # jumping speed
        self.declare_parameter("wave_yaw_speed", 0.5)       # rad/s для waving
        self.declare_parameter("command_topic", "/f_fox_command/command")
        self.declare_parameter("stand_body_height", 0.0)
        self.declare_parameter("action_timeout", 1.5)       # СЕКУНД без сообщений

        self.dt = float(self.get_parameter("timer_dt").value)
        self.walk_vx = float(self.get_parameter("walk_vx").value)
        self.jump_vx = float(self.get_parameter("jump_vx").value)
        self.wave_yaw_speed = float(self.get_parameter("wave_yaw_speed").value)
        self.command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self.stand_body_height = float(self.get_parameter("stand_body_height").value)
        self.action_timeout = float(self.get_parameter("action_timeout").value)

        # LCM-клиент
        self.lcm_client = A1LCMClient()

        # Текущее действие
        self.current_label = None          # type: Optional[str]
        self.action_start_time = None      # type: Optional[float]

        # Время последнего ПОЛУЧЕННОГО PersonAction
        self.last_msg_time = None          # type: Optional[float]
        self.command_active = False        # есть ли действующая команда

        # Подписка на PersonAction
        self.sub_command = self.create_subscription(
            FoxCommand,
            self.command_topic,
            self.command_cb,
            10,
        )

        self.get_logger().info(
            f"A1LCMControlNode started. "
            f"dt={self.dt}, walk_vx={self.walk_vx}, jump_vx={self.jump_vx}, "
            f"wave_yaw_speed={self.wave_yaw_speed}, action_topic='{self.command_topic}', "
            f"action_timeout={self.action_timeout}"
        )

        # Таймер управления
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self._step = 0
        self._last_timeout_state = False  # чтобы один раз логировать переход в idle

    # ====== PersonAction callback ======

    def command_cb(self, msg: FoxCommand):
        now = self.get_clock().now().nanoseconds / 10**9
        new_label = (msg.command or "").strip().lower()
        if not new_label:
            # пустой label: игнорируем, но фиксируем факт прихода сообщения
            self.last_msg_time = now
            return

        # Нужно ли переинициализировать действие?
        #  - если команда была неактивна (по таймауту) → всегда
        #  - или если label действительно изменился
        if (not self.command_active) or (new_label != self.current_label):
            self.current_label = new_label
            self.action_start_time = now
            self.get_logger().info(
                f"New action label: '{self.current_label}', "
                f"confidence={msg.confidence:.3f}"
            )

        # Обновляем "последнее время команды" и флаг активности
        self.last_msg_time = now
        self.command_active = True

    # ====== main control loop ======

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 10**9

        # Проверяем таймаут команд
        if self.last_msg_time is None:
            # ещё ни одной команды не было
            command_active = False
        else:
            if (now - self.last_msg_time) > self.action_timeout:
                command_active = False
            else:
                command_active = True

        # Для отладки логируем переход в/из idle (по таймауту)
        if command_active != self._last_timeout_state:
            if not command_active:
                self.get_logger().warn(
                    f"No PersonAction received for > {self.action_timeout} s. "
                    f"Switching to stand."
                )
            else:
                self.get_logger().info("PersonAction commands active again.")
            self._last_timeout_state = command_active

        label = self.current_label if command_active else None

        # Считаем elapsed с момента НАЧАЛА ТЕКУЩЕГО действия
        if label is None or self.action_start_time is None:
            elapsed = 0.0
        else:
            elapsed = now - self.action_start_time

        # Выбор команды
        if label is None:
            cmd = self._cmd_stand()
            phase = "idle"
        elif label == "walking":
            cmd = self._cmd_walking()
            phase = "walking"
        elif label == "salute":
            cmd = self._cmd_waving(elapsed)
            phase = "waving"
        elif label == "jumping":
            cmd = self._cmd_jumping(elapsed)
            phase = "jumping"
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

    def _cmd_waving(self, elapsed: float):
        # кручение: 1 c влево, 1 c вправо
        cycle = 2.0
        phase = elapsed % cycle

        if phase < cycle / 2.0:
            yaw = +self.wave_yaw_speed
        else:
            yaw = -self.wave_yaw_speed

        return pack_highcmd(
            mode=2,
            gait_type=1,
            vx=0.0,
            vy=0.0,
            yaw_speed=yaw,
            foot_raise_height=0.08,
            body_height=self.stand_body_height,
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
