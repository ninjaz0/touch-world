import machine
import utime
import binascii
import rp2

# Set UART Pin
sensor_tx_pin_number = 4
sensor_rx_pin_number = 5
bluetooth_tx_pin_number = 0
bluetooth_rx_pin_number = 1

sensor_tx_pin = machine.Pin(sensor_tx_pin_number)
sensor_rx_pin = machine.Pin(sensor_rx_pin_number)
bluetooth_tx_pin = machine.Pin(bluetooth_tx_pin_number)
bluetooth_rx_pin = machine.Pin(bluetooth_rx_pin_number)

# HMMD-mmWave-Sensor Init
soft_uart = machine.UART(1, baudrate=115200, tx=sensor_tx_pin, rx=sensor_rx_pin)
# Bluetooth Moudle Init
uart = machine.UART(0,baudrate=9600,bits=8,parity=None,stop=1,tx=bluetooth_tx_pin,rx=bluetooth_rx_pin)

# ws2812_OnBoard init
max_lum =100
r=0
g=0
b=0

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=machine.Pin(16))
sm.active(1)

# Anti-shake algorithm
distance = 20
smart_ag = 5
limit_average_distance = distance * smart_ag
pool = []
status_code = 0
former_status_code = 0

def anti_shake(raw_data):
    global pool,status_code,former_status_code
    sum = 0
    pool.append(raw_data)
    if len(pool) > smart_ag:
        pool = pool[-smart_ag:]
    for i in pool:
        sum += i
    if sum > limit_average_distance:
        r,g,b = 0,0,0
        rgb = (g << 24) | (r << 16) | (b << 8)
        sm.put(rgb)
        status_code = 0
    else:
        r,g,b = 255,255,255
        rgb = (g << 24) | (r << 16) | (b << 8)
        sm.put(rgb)
        status_code = 1
    if former_status_code != status_code:
        if status_code == 1:
            uart.write("Light ON!\n")
        else:
            uart.write("Light OFF!\n")
    former_status_code = status_code
        
# Main Function
def send_hex_string(hex_string):
    hex_bytes = binascii.unhexlify(hex_string)
    soft_uart.write(hex_bytes)
    
def extract_number_manual(text):
    text = str(text)
    num_str = ""
    for char in text:
        if char.isdigit():  # 当前字符为数字
            num_str += char
        elif num_str:  # 遇到非数字且已有数字，结束提取
            break
    return int(num_str) if num_str else None

def read_serial_data():
    while True:
        utime.sleep_ms(40)
        if soft_uart.any():
            # 读取串口数据
            data = soft_uart.readline()
            data = extract_number_manual(data)
            if data != None:
                print(data)
                anti_shake(data)
                    

if __name__ == "__main__":
    hex_to_send = "FDFCFBFA0800120000006400000004030201"
    
    send_hex_string(hex_to_send)
    
    response = read_serial_data()
    



