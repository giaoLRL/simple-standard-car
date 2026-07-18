"""
ESP32 BLE-UART 透传桥 (MicroPython) — 替代 JDY-23

架构:
  手机小程序 ←→ BLE ←→ ESP32 ←→ UART(9600) ←→ MSPM0G3507

ESP32 模拟 JDY-23 的 BLE GATT, 小程序和 MCU 固件零改动:
  Service UUID:       0000FFE0-0000-1000-8000-00805F9B34FB
  Characteristic UUID: 0000FFE1-0000-1000-8000-00805F9B34FB
  设备名:             JDY-23

v2.0: 增大缓冲区以容纳完整二进制遥测帧 (54 字节 + 余量),
      改用 gatts_notify 替代 gatts_indicate 提高实时性

硬件接线:
  ESP32 GPIO16 (RX) ← MSPM0 PA10 (UART0 TX)
  ESP32 GPIO17 (TX) → MSPM0 PA11 (UART0 RX)
  ESP32 GND         ↔ MSPM0 GND
  ESP32 GPIO2        → 板载 LED (低电平亮, 状态指示)

调试:
  连接 USB 串口 (115200bps) 查看实时日志。
  print() 走 UART0→USB, 不影响 UART2→MCU 的数据通道。

烧录:
  1. 刷入 MicroPython 固件 (ESP32_GENERIC-xxx.bin)
  2. 用 Thonny/mpremote 上传 main.py 到 /pyboard/
  3. 复位 ESP32, 自动运行
"""

import bluetooth
from machine import UART, Pin
import time
import struct

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

_SVC_UUID = bluetooth.UUID("0000FFE0-0000-1000-8000-00805F9B34FB")
_CHR_UUID = bluetooth.UUID("0000FFE1-0000-1000-8000-00805F9B34FB")

_UART_NUM   = const(2)
_UART_BAUD  = const(9600)
_UART_TX    = const(17)
_UART_RX    = const(16)

_ADV_INTERVAL_US = const(20000)
_DEVICE_NAME     = "JDY-23"

_LED_PIN  = const(2)
_LED_ON   = const(0)
_LED_OFF  = const(1)

_HEARTBEAT_MS = const(5000)
_LOOP_DELAY_MS = const(5)

# 增大缓冲区以容纳完整二进制帧 (54 字节 + 余量)
_BUF_SIZE      = const(128)
_FLUSH_MS      = const(200)
_DISCARD_MS    = const(500)


def _ts():
    t = time.ticks_ms() // 1000
    return "[{:4d}.{:01d}]".format(t // 10, t % 10)

def _hex_str(data, max_len=32):
    if data is None:
        return "(None)"
    limit = min(len(data), max_len)
    h = " ".join("{:02X}".format(b) for b in data[:limit])
    if len(data) > max_len:
        h += " ...(+{}B)".format(len(data) - max_len)
    return h


class JDYBleBridge:
    def __init__(self):
        self._led = Pin(_LED_PIN, Pin.OUT, value=_LED_OFF)

        self._rx_from_ble    = 0
        self._rx_from_uart   = 0
        self._tx_to_ble      = 0
        self._tx_packets     = 0
        self._notify_attempts = 0
        self._notify_ok      = 0
        self._notify_fails   = 0
        self._conn_count     = 0
        self._cccd_seen      = False
        self._last_hb        = time.ticks_ms()

        self._evt_connect     = False
        self._evt_disconnect  = False
        self._evt_ble_writes  = []
        self._evt_restart_adv = False
        self._evt_cccd_write  = False
        self._evt_data_write  = False
        self._last_cccd_handle = 0
        self._last_cccd_value  = b""
        self._last_data_value  = b""

        print("{} BLE 初始化...".format(_ts()))
        self._ble = bluetooth.BLE()
        for attempt in range(5):
            try:
                self._ble.active(True)
                break
            except Exception as e:
                print("{} BLE active (尝试{}/5): {}".format(_ts(), attempt + 1, e))
                try:
                    self._ble.active(False)
                except Exception:
                    pass
                if attempt < 4:
                    time.sleep_ms(300)
        else:
            print("{} BLE 初始化彻底失败, 请按 RST 键硬复位 ESP32".format(_ts()))
            raise RuntimeError("BLE unrecoverable")
        self._ble.irq(self._irq)
        print("{} BLE active, MAC: {}".format(_ts(), self._fmt_mac()))

        svc = (
            _SVC_UUID,
            ((_CHR_UUID,
              bluetooth.FLAG_WRITE |
              bluetooth.FLAG_WRITE_NO_RESPONSE |
              bluetooth.FLAG_NOTIFY,
              ((bluetooth.UUID(0x2902), bluetooth.FLAG_READ | bluetooth.FLAG_WRITE),),
            ),),
        )
        ((self._chr_handle, self._cccd_handle),) = self._ble.gatts_register_services((svc,))
        print("{} GATT 服务已注册, chr_handle={}, cccd_handle={}".format(
            _ts(), self._chr_handle, self._cccd_handle))

        self._ble.gatts_set_buffer(self._chr_handle, 256, True)

        self._conn_handle = None

        print("{} UART{} 初始化: {}bps, TX=GPIO{}, RX=GPIO{}".format(
            _ts(), _UART_NUM, _UART_BAUD, _UART_TX, _UART_RX))
        self._uart = UART(
            _UART_NUM,
            baudrate=_UART_BAUD,
            tx=Pin(_UART_TX),
            rx=Pin(_UART_RX),
        )

        self._buf = bytearray(_BUF_SIZE)
        self._buf_len = 0
        self._last_flush = time.ticks_ms()

        self._start_advertising()

    def _fmt_mac(self):
        import ubinascii
        addr = ubinascii.hexlify(self._ble.config("mac")[1], ":").decode()
        return addr

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, addr_type, addr = data
            self._conn_handle = conn_handle
            self._evt_connect = True
            self._cccd_seen = False

        elif event == _IRQ_CENTRAL_DISCONNECT:
            self._conn_handle = None
            self._cccd_seen = False
            self._evt_disconnect = True
            self._evt_restart_adv = True

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, attr_handle = data

            if attr_handle == self._cccd_handle:
                self._cccd_seen = True
                self._evt_cccd_write = True
                self._last_cccd_handle = attr_handle
                try:
                    raw = self._ble.gatts_read(attr_handle)
                    self._last_cccd_value = bytes(raw) if raw else b""
                except Exception:
                    self._last_cccd_value = b""

            elif attr_handle == self._chr_handle:
                value = self._ble.gatts_read(attr_handle)
                if value:
                    self._rx_from_ble += len(value)
                    self._evt_ble_writes.append(value)
                    self._last_data_value = bytes(value)
                    self._evt_data_write = True

    def _start_advertising(self):
        svc_bytes = bytes([
            0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
            0x00, 0x10, 0x00, 0x00, 0xE0, 0xFF, 0x00, 0x00,
        ])
        name_bytes = _DEVICE_NAME.encode()

        adv = (
            b"\x02\x01\x06"
            + b"\x11\x07" + svc_bytes
            + struct.pack("BB", len(name_bytes) + 1, 0x09) + name_bytes
        )
        self._ble.gap_advertise(_ADV_INTERVAL_US, adv_data=adv)

    def _process_events(self):
        if self._evt_connect:
            self._evt_connect = False
            self._conn_count += 1
            self._led.value(_LED_ON)
            print("{} 已连接 (第{}次) | chr_handle={} cccd_handle={}".format(
                _ts(), self._conn_count, self._chr_handle, self._cccd_handle))

        if self._evt_disconnect:
            self._evt_disconnect = False
            self._led.value(_LED_OFF)
            print("{} 已断线, 重新广播中...".format(_ts()))

        if hasattr(self, '_evt_data_write') and self._evt_data_write:
            self._evt_data_write = False
            print("{} 特性写入 handle={} value={}".format(
                _ts(), self._chr_handle, _hex_str(self._last_data_value, 16)))

        if hasattr(self, '_evt_cccd_write') and self._evt_cccd_write:
            self._evt_cccd_write = False
            handle = self._last_cccd_handle
            val = self._last_cccd_value
            val_str = _hex_str(val, 16) if val else "(gatts_read failed)"
            expected = str(self._cccd_handle)
            correct = "v" if handle == self._cccd_handle else "x"
            print("{} CCCD 写入! handle={} (expected cccd_handle={}) {} val={}".format(
                _ts(), handle, expected, correct, val_str))

        while self._evt_ble_writes:
            data = self._evt_ble_writes.pop(0)
            self._uart.write(data)
            txt = data.decode("ascii", "replace").rstrip("\r\n")
            print("{} 手机->MCU [{}B]: {}".format(_ts(), len(data), txt))

        if self._evt_restart_adv:
            self._evt_restart_adv = False
            self._start_advertising()

    def _heartbeat(self, now):
        if time.ticks_diff(now, self._last_hb) < _HEARTBEAT_MS:
            return
        self._last_hb = now

        led_state = "LED" if self._conn_handle is not None else "OFF"
        cccd_state = "SUB" if self._cccd_seen else "NOSUB"
        print("{} {} {} 统计: UART->MCU {}B | BLE->APP {}B/{}包 | notify OK:{} FAIL:{} | 连接:{}次".format(
            _ts(), led_state, cccd_state,
            self._rx_from_uart,
            self._tx_to_ble, self._tx_packets,
            self._notify_ok, self._notify_fails,
            self._conn_count,
        ))

    def run(self):
        print("{} 广播中... 设备名: {}".format(_ts(), _DEVICE_NAME))
        self._led_pulse(3)

        try:
            while True:
                self._process_events()

                n = self._uart.any()
                if n > 0:
                    space = _BUF_SIZE - self._buf_len
                    if n > space:
                        n = space
                    chunk = self._uart.read(n)
                    if chunk:
                        self._buf[self._buf_len : self._buf_len + n] = chunk
                        self._buf_len += n
                        self._rx_from_uart += n

                now = time.ticks_ms()

                if self._buf_len > 0:
                    flush = False
                    if self._buf_len >= _BUF_SIZE:
                        flush = True
                    elif time.ticks_diff(now, self._last_flush) >= _FLUSH_MS:
                        flush = True

                    if flush:
                        if self._conn_handle is not None:
                            self._notify_attempts += 1
                            data = bytes(self._buf[:self._buf_len])
                            try:
                                # v2.0: 使用 notify 替代 indicate, 提高实时性
                                self._ble.gatts_notify(
                                    self._conn_handle, self._chr_handle, data)
                                self._notify_ok += 1
                                self._tx_to_ble += self._buf_len
                                self._tx_packets += 1
                            except Exception as e:
                                self._notify_fails += 1
                                if self._notify_fails % 20 == 1:
                                    print("{} notify 异常 ({}次): {}".format(
                                        _ts(), self._notify_fails, e))
                        self._buf_len = 0
                        self._last_flush = now

                if self._buf_len > 0 and time.ticks_diff(now, self._last_flush) > _DISCARD_MS:
                    print("{} UART 半包丢弃: {}B (CCCD={})".format(
                        _ts(), self._buf_len, "SUB" if self._cccd_seen else "NOSUB"))
                    self._buf_len = 0
                    self._last_flush = now

                self._heartbeat(now)

                time.sleep_ms(_LOOP_DELAY_MS)

        except KeyboardInterrupt:
            pass
        finally:
            print("{} 正在关闭...".format(_ts()))
            try:
                self._ble.gap_advertise(None)
            except Exception:
                pass
            try:
                self._ble.active(False)
            except Exception:
                pass
            self._led.value(_LED_OFF)
            print("{} 已退出, 可重新上传".format(_ts()))

    def _led_pulse(self, count):
        for _ in range(count):
            self._led.value(_LED_ON)
            time.sleep_ms(80)
            self._led.value(_LED_OFF)
            time.sleep_ms(120)


if __name__ == "__main__":
    print("=" * 48)
    print("  ESP32 BLE-UART 透传桥")
    print("  模拟 JDY-23 | MicroPython v2.0")
    print("=" * 48)
    try:
        bridge = JDYBleBridge()
        bridge.run()
    except Exception as e:
        print("FATAL: {}".format(e))
        import sys
        sys.print_exception(e)
        try:
            ble = bluetooth.BLE()
            ble.active(False)
        except Exception:
            pass
    print("--- 脚本结束 ---")
