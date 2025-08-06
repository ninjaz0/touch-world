import machine
import utime
import binascii
import rp2
import array

# Pin Configuration
SENSOR_PINS = {"tx": 4, "rx": 5}
BLUETOOTH_PINS = {"tx": 0, "rx": 1}

# Anti-shake Parameters
DISTANCE_THRESHOLD = 40
SMOOTHING_FACTOR = 3
POOL_SIZE = SMOOTHING_FACTOR
MAX_DISTANCE = DISTANCE_THRESHOLD * SMOOTHING_FACTOR

# LED Parameters
LED_PIN = 16
LED_STATE = {
    0: (0, 0, 0),        # OFF - RGB black
    1: (255, 255, 255)   # ON - RGB white
}

# Initialize UARTs
sensor_uart = machine.UART(
    1, 
    baudrate=115200,
    tx=machine.Pin(SENSOR_PINS["tx"]),
    rx=machine.Pin(SENSOR_PINS["rx"])
)

bluetooth_uart = machine.UART(
    0,
    baudrate=9600,
    bits=8,
    parity=None,
    stop=1,
    tx=machine.Pin(BLUETOOTH_PINS["tx"]),
    rx=machine.Pin(BLUETOOTH_PINS["rx"])
)

# Initialize NeoPixel LED
@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=24
)
def ws2812():
    wrap_target()
    out(x, 1).side(0).delay(3-1)
    jmp(not_x, "do_zero").side(1).delay(2-1)
    jmp("bitloop").side(1).delay(5-1)
    label("bitloop")
    label("do_zero")
    nop().side(0).delay(5-1)
    wrap()

sm = rp2.StateMachine(
    0, 
    ws2812, 
    freq=8_000_000, 
    sideset_base=machine.Pin(LED_PIN)
)
sm.active(1)

# Anti-shake state
class SensorState:
    def __init__(self):
        self.pool = array.array('I', [0] * POOL_SIZE)
        self.index = 0
        self.pool_full = False
        self.current_state = 0
        self.previous_state = 0

sensor_state = SensorState()

# LED Control Functions
def set_led(state):
    r, g, b = LED_STATE[state]
    rgb = (g << 24) | (r << 16) | (b << 8)
    sm.put(rgb)

def update_bluetooth_state():
    if sensor_state.current_state != sensor_state.previous_state:
        message = "Light ON!\n" if sensor_state.current_state else "Light OFF!\n"
        bluetooth_uart.write(message)
        sensor_state.previous_state = sensor_state.current_state

# Anti-shake Algorithm
def process_distance(raw_distance):
    # Update circular buffer
    sensor_state.pool[sensor_state.index] = raw_distance
    sensor_state.index = (sensor_state.index + 1) % POOL_SIZE
    
    # Check if buffer is full
    if not sensor_state.pool_full and sensor_state.index == 0:
        sensor_state.pool_full = True
    
    # Calculate average if buffer full
    total = 0
    if sensor_state.pool_full:
        for val in sensor_state.pool:
            total += val
    else:
        total = sum(sensor_state.pool[:sensor_state.index])
        count = sensor_state.index
        total = total * POOL_SIZE / count if count else MAX_DISTANCE
    
    # Update state
    sensor_state.current_state = 1 if total <= MAX_DISTANCE else 0
    set_led(sensor_state.current_state)
    update_bluetooth_state()

# Serial Processing
def extract_distance(data):
    """Extracts first numeric value from byte data"""
    num_str = bytearray()
    for byte in data:
        char = chr(byte)
        if char.isdigit():
            num_str.append(byte)
        elif num_str:
            break
    return int(num_str.decode()) if num_str else None

def read_sensor_data():
    buffer = bytearray()
    while True:
        if sensor_uart.any():
            buffer.extend(sensor_uart.read(sensor_uart.any()))
            
            # Process each line in buffer
            while b'\n' in buffer:
                line, _, buffer = buffer.partition(b'\n')
                if line:
                    yield line
        
        # Add small delay to prevent busy-waiting
        utime.sleep_ms(5)

# Main Application
def main():
    # Send configuration to sensor
    config = binascii.unhexlify("FDFCFBFA0800120000006400000004030201")
    sensor_uart.write(config)
    
    # Main processing loop
    for data in read_sensor_data():
        distance = extract_distance(data)
        if distance is not None:
            print(distance)
            process_distance(distance)

if __name__ == "__main__":
    main()