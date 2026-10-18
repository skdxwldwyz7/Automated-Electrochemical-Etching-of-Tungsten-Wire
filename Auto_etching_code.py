import serial
import pyvisa
import time
import statistics
import matplotlib.pyplot as plt  # 导入绘图库

# --- 硬件参数配置 ---
MOTOR_PORT = 'COM7'
K2400_ADDR = 'GPIB0::24::INSTR'
PULSE_PER_MM = 3200
TARGET_DEPTH_MM = 1.0
ETCH_VOLTAGE = 5.0

# 液面搜索参数：540 脉冲约 0.17 mm；每一步只等待很短时间，减少找液面耗时。
LIQUID_SEARCH_STEP_PULSES = 540
LIQUID_SEARCH_SPEED = 3000
LIQUID_SEARCH_INTERVAL_S = 0.05
LIQUID_CURRENT_THRESHOLD_MA = 0.05

# 断点判定参数（电流单位为 mA）。
# 断点优先由“短时间内的相对突降”判断，而不是等待固定绝对电流。
# 7 组数据表明，结构性台阶通常同时满足绝对下降 >= 0.20 mA、
# 相对突降前基线下降 >= 25%；启动阶段的正常波动通常达不到这个比例。
SUDDEN_DROP_MA = 0.20
SUDDEN_DROP_RATIO = 0.25
# 突降后的新平台必须低于 0.60 mA 才允许拔针。
# 0.7 mA 左右的台阶通常表示已经变细但尚未完全断裂，只继续刻蚀。
MAX_SUDDEN_DROP_RESULT_MA = 0.60
MIN_MONITOR_TIME_S = 2.0
MIN_PRE_DROP_CURRENT_MA = 0.60
DROP_BASELINE_SAMPLES = 3
DROP_CONFIRM_SAMPLES = 2
DROP_CONFIRM_TOLERANCE = 0.10

# 低电流保护仍然保留，避免突降没有被及时采到时发生过刻。
BREAK_CURRENT_MA = 0.45
OVERETCH_CURRENT_MA = 0.10
BREAK_CONFIRM_SAMPLES = 2
MONITOR_INTERVAL_S = 0.01

# 断点后只抬升 2.5 mm，避免原来的 5 mm 抬升过多。
RETRACT_MM = 2.5
MOTOR_SPEED = 3000

# 用于存储绘图数据
time_data = []
current_data = []

# --- 电机十六进制指令函数 ---
def send_motor_cmd(ser, pulses, direction=0, speed=3000):
    addr = 0x01
    cmd = 0xFD
    speed_hex = int(speed).to_bytes(2, 'big')
    accel = 0x0A 
    pulse_hex = int(abs(pulses)).to_bytes(4, 'big')
    rel_mode = 0x00 
    sync = 0x00
    checksum = 0x6B 
    payload = bytes([addr, cmd, direction]) + speed_hex + bytes([accel]) + pulse_hex + bytes([rel_mode, sync, checksum])
    ser.write(payload)

# --- 获取电流函数 ---
def get_current(device):
    try:
        raw_data = device.query(":MEAS:CURR?")
        parts = raw_data.strip().split(',')
        return float(parts[0]) if len(parts) == 1 else float(parts[1])
    except Exception as exc:
        print(f"读取 K2400 电流失败: {exc}")
        return None

# --- 主程序 ---
def main():
    start_time = time.time()
    k2400 = None
    ser = None
    completed_reason = ""
    try:
        rm = pyvisa.ResourceManager()
        k2400 = rm.open_resource(K2400_ADDR)
        ser = serial.Serial(MOTOR_PORT, 115200, timeout=0.1)
        
        k2400.write(":FORM:ELEM CURR") 
        k2400.write(f":SOUR:VOLT {ETCH_VOLTAGE}")
        k2400.write(":OUTP ON")
        
        print("--- 自动化刻蚀程序开始 (带绘图功能) ---")
        
        # Step 1: 寻找液面
        print("Step 1: 正在寻找液面...")
        while True:
            current = get_current(k2400)
            if current is None:
                raise RuntimeError("K2400 电流读取失败，已停止液面搜索")
            if current * 1000.0 >= LIQUID_CURRENT_THRESHOLD_MA:
                print(f"接触液面！电流: {current*1000:.3f} mA")
                break
            send_motor_cmd(
                ser,
                LIQUID_SEARCH_STEP_PULSES,
                direction=0,
                speed=LIQUID_SEARCH_SPEED,
            )
            time.sleep(LIQUID_SEARCH_INTERVAL_S)

        # Step 2: 深入 1mm
        print(f"Step 2: 深入 {TARGET_DEPTH_MM} mm...")
        send_motor_cmd(
            ser,
            round(TARGET_DEPTH_MM * PULSE_PER_MM),
            direction=0,
            speed=MOTOR_SPEED,
        )
        time.sleep(1)

        # Step 3: 实时刻蚀监控
        print("Step 3: 刻蚀监控中...")
        last_current = get_current(k2400)
        if last_current is None:
            raise RuntimeError("K2400 电流读取失败，已停止刻蚀监控")
        last_current_ma = last_current * 1000.0
        break_confirm_count = 0
        sudden_drop_count = 0
        sudden_drop_level_ma = None
        sudden_drop_from_ma = None
        recent_currents_ma = []
        etch_start_point = time.time() # 记录刻蚀开始的时间点
        
        while True:
            current = get_current(k2400)
            if current is None:
                raise RuntimeError("K2400 电流读取失败，已停止刻蚀监控")
            elapsed = time.time() - etch_start_point
            
            # 记录数据用于绘图
            time_data.append(elapsed)
            current_ma = current * 1000.0
            current_data.append(current_ma) # 转换为 mA 方便观察
            
            # 用最近 3 个有效读数的中值作为突降前基线，避免一个异常高点
            # 或异常低点被误认为断点。中值窗口只有约 0.2 s，不会明显延迟真正台阶。
            if len(recent_currents_ma) >= DROP_BASELINE_SAMPLES:
                drop_baseline_ma = statistics.median(recent_currents_ma)
            else:
                drop_baseline_ma = last_current_ma
            delta_i_ma = drop_baseline_ma - current_ma
            
            # 判定策略：
            # 1) 监控开始前 2 s 只记录，不判断启动瞬态；
            # 2) 优先识别结构性突降，并要求下一点仍处于新平台；
            # 3) 绝对电流 <=0.45 mA 的第二平台作为备用判据；
            # 4) <=0.1 mA 是过刻保护，立即停止。
            if current_ma <= OVERETCH_CURRENT_MA:
                completed_reason = f"电流已降至过刻保护值 {current_ma:.3f} mA"
            elif elapsed >= MIN_MONITOR_TIME_S:
                relative_drop = (
                    delta_i_ma / drop_baseline_ma if drop_baseline_ma > 0 else 0.0
                )
                sudden_drop = (
                    drop_baseline_ma >= MIN_PRE_DROP_CURRENT_MA
                    and delta_i_ma >= SUDDEN_DROP_MA
                    and relative_drop >= SUDDEN_DROP_RATIO
                    and current_ma <= MAX_SUDDEN_DROP_RESULT_MA
                )

                if sudden_drop_count:
                    # 突降后的下一点仍接近新平台，才确认不是单点毛刺。
                    if (
                        sudden_drop_level_ma is not None
                        and current_ma <= sudden_drop_level_ma * (1.0 + DROP_CONFIRM_TOLERANCE)
                    ):
                        sudden_drop_count += 1
                    else:
                        sudden_drop_count = 0
                        sudden_drop_level_ma = None
                        sudden_drop_from_ma = None
                elif sudden_drop:
                    sudden_drop_count = 1
                    sudden_drop_level_ma = current_ma
                    sudden_drop_from_ma = drop_baseline_ma

                if (
                    sudden_drop_count >= DROP_CONFIRM_SAMPLES
                    and current_ma <= MAX_SUDDEN_DROP_RESULT_MA
                ):
                    completed_reason = (
                        f"检测到突降：{sudden_drop_from_ma:.3f} -> "
                        f"{sudden_drop_level_ma:.3f} mA"
                    )

            if not completed_reason and current_ma <= BREAK_CURRENT_MA:
                break_confirm_count += 1
                if break_confirm_count >= BREAK_CONFIRM_SAMPLES:
                    completed_reason = (
                        f"检测到第二平台：{current_ma:.3f} mA，"
                        f"连续确认 {break_confirm_count} 次"
                    )
            elif current_ma > BREAK_CURRENT_MA:
                # 电流回到第一平台以上，低电流确认必须重新开始。
                break_confirm_count = 0

            if completed_reason:
                print(f"!!! 刻蚀完成：{completed_reason} !!!")
                k2400.write(":OUTP OFF")
                send_motor_cmd(
                    ser,
                    round(RETRACT_MM * PULSE_PER_MM),
                    direction=1,
                    speed=MOTOR_SPEED,
                )
                break
            
            recent_currents_ma.append(current_ma)
            if len(recent_currents_ma) > DROP_BASELINE_SAMPLES:
                recent_currents_ma.pop(0)
            last_current_ma = current_ma
            time.sleep(MONITOR_INTERVAL_S)

        # --- 绘制曲线 ---
        print("正在生成刻蚀电流曲线图...")
        plt.figure(figsize=(10, 6))
        plt.plot(time_data, current_data, color='blue', linewidth=1.5)
        plt.title('W-Tip Electrochemical Etching Current Profile')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Current (mA)')
        plt.grid(True, linestyle='--', alpha=0.7)
        # 在图上标出断裂点
        plt.annotate('Break detected', xy=(time_data[-1], current_data[-1]), 
                     xytext=(time_data[-1]-2, current_data[-1]+2),
                     arrowprops=dict(facecolor='red', shrink=0.05))
        plt.show()

    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if k2400 is not None:
            try:
                k2400.write(":OUTP OFF")
            except Exception as exc:
                print(f"关闭 K2400 输出失败: {exc}")
        if ser is not None:
            ser.close()

if __name__ == "__main__":
    main()
