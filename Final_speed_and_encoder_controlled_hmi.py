import RPi.GPIO as GPIO
import time
import threading
import math
import serial
import sys

# === GPIO Definitions (BCM numbers) ===
# DC Motor 1 (BTS7960)
M1_R_EN = 5
M1_L_EN = 6
M1_R_PWM = 12
M1_L_PWM = 13

# DC Motor 2 (BTS7960)
M2_R_EN = 16
M2_L_EN = 20
M2_R_PWM = 18
M2_L_PWM = 21

# Stepper Motor Pins
Y_STEP = 17
Y_DIR  = 27
X_STEP = 25
X_DIR  = 23

# Limit Switches
X_LIMIT_1 = 10  # X min
X_LIMIT_2 = 9   # X max
Y_LIMIT_1 = 8   # Y min
Y_LIMIT_2 = 11  # Y max

# Encoder Pins (BCM)
ENC1_A = 2
ENC1_B = 3
ENC2_A = 19
ENC2_B = 24

# === Constants ===
PWM_FREQ = 10000              # Higher frequency for better motor control
STEPPER_DELAY = 0.002          # Step pulse duration
DC_RUN_TIME = 3                # 3 seconds runtime for DC motors
STEPPER_HOME_POSITION = 1000   # Position after homing
STEPPER_BACKOFF_STEPS = 50     # Steps to back off after homing
MIN_PWM = 20                   # Lower minimum PWM for reduced speed
MAX_EXTRA_STEPS = 500          # Max additional steps for homing
MAX_SPEED = 100                # Maximum allowed PWM speed
DOUBLE_PRESS_THRESHOLD = 0.5   # 500ms for double press detection
MANUAL_DC_SPEED = 40           # Speed for manual DC control
MANUAL_DC_DURATION = 0.5       # 0.5 seconds for manual DC control
MANUAL_STEPPER_STEPS = 800     # 800 steps for manual stepper control

# === Global Variables ===
encoder1_count = 0
encoder2_count = 0
encoder1_last_state = 0
encoder2_last_state = 0
current_x_position = 0
current_y_position = 0
emergency_stop = False         # Global emergency stop flag
stepper_active = False         # Stepper motors moving flag
dc_active = False              # DC motors running flag
x_max_triggered = False
y_max_triggered = False
encoder_polling_active = True  # Control for encoder thread
motor1_calibration = 1.0       # Motor1 speed calibration factor
motor2_calibration = 1.0       # Motor2 speed calibration factor
pwm_objects = {}               # Dictionary to store PWM objects
system_active = False          # System activation state from display
function_state = "System Ready"  # Display function state
ser = None                     # Serial object for display
sequence_running = False       # Flag for main sequence
threeFF = 0                    # Counter for FF bytes
inBuffer = []                  # Buffer for display data
last_off_press_time = 0        # For double press detection
in_homing_sequence = False     # Track if we're in homing sequence
current_page = 0               # Track current display page (0=main, 1=individual, 2=combined)

# Sequence tracking variables
total_cycles = 10
current_cycle = 0
current_step = 0
sequence_paused = False

# Define sequence steps
sequence_steps = [
    # Step 0: Homing
    {"name": "Homing Steppers", "func": "home_steppers_together"},
    
    # Step 1: Test 1 - Run DC motors CCW
    {"name": "Test 1: DC CCW", "func": "run_motors_timed", "args": (-39.515, 33.52, 6.2)},
    
    # Step 2: Move steppers CW
    {"name": "Moving Steppers CW", "func": "run_stepper_motors", "args": ('CW', 5000)},
    
    # Step 3: Test 2 - Run DC motors CW
    {"name": "Test 2: DC CW", "func": "run_motors_timed", "args": (40, -32.04, 8.1)},
    
    # Step 4: Test 3 - Run DC motors with different speeds
    {"name": "Test 3: DC Diff Speeds", "func": "run_motors_timed", "args": (-39.515, 33.52, 6.7)},
    
    # Step 5: Move steppers CCW
    {"name": "Moving Steppers CCW", "func": "run_stepper_motors", "args": ('CCW', 5000)},
    
    # Step 6: Test 4 - Final verification
    {"name": "Test 4: DC Custom", "func": "run_motors_timed", "args": (40.0, -30.55, 7.0)},
]

# === Display Functions ===
def init_display():
    """Initialize the serial connection to the display"""
    global ser
    try:
        # Using /dev/ttyAMA0 (GPIO serial pins)
        ser = serial.Serial(
            port="/dev/ttyAMA0",
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        print("Display serial port initialized")
        return True
    except Exception as e:
        print(f"Display initialization failed: {e}")
        return False

def display_send(command):
    """Send a command to the display using same encoding as your working code"""
    global ser
    if ser is None or not ser.is_open:
        return
    
    try:
        # Use the exact same encoding as your working code
        full_command = command + "\xFF\xFF\xFF"
        ser.write(bytes(full_command, encoding="raw_unicode_escape"))
        print(f"Sent to display: {command}")
    except Exception as e:
        print(f"Display send error: {e}")

def display_receive():
    """Receive and process display commands using your exact logic"""
    global inBuffer, threeFF
    
    if ser is None or not ser.is_open:
        return False, None
    
    try:
        while ser.in_waiting > 0:
            v = ser.read(1)
            if v == b'\xff':
                threeFF += 1
            else:
                threeFF = 0
            inBuffer.append(v)
            if threeFF == 3:
                threeFF = 0
                if len(inBuffer) < 4:
                    inBuffer = []
                    break
                returnBuffer = inBuffer[:-3]
                inBuffer = []
                return True, returnBuffer
        return False, None
    except Exception as e:
        print(f"Display receive error: {e}")
        return False, None

def process_display_message(buffer):
    """Process display message using your exact logic"""
    global function_state, system_active, emergency_stop, sequence_paused
    global last_off_press_time, current_cycle, current_step, current_page
    
    # 'e' => 0x65  touch event
    if buffer[0] == b'e':
        # Check length of buffer
        if len(buffer) == 4:
            page_id = ord(buffer[1])
            button_id = ord(buffer[2])
            event_type = ord(buffer[3])
            
            print(f"Received display event: page={page_id}, button={button_id}, event={event_type}")
            
            # Only process press events (event_type=1)
            if event_type == 1:
                # Page 0: Main Control Page
                if page_id == 0:
                    # Button IDs: 1=ON, 2=OFF, 3=IC, 4=CC
                    if button_id == 1:  # ON button
                        system_active = True
                        emergency_stop = False
                        sequence_paused = False
                        function_state = "Resuming..."
                        print("System activated")
                    elif button_id == 2:  # OFF button
                        current_time = time.time()
                        # Check for double press
                        if current_time - last_off_press_time < DOUBLE_PRESS_THRESHOLD:
                            # Double press - reset entire sequence
                            current_cycle = 0
                            current_step = 0
                            function_state = "Sequence Reset"
                            print("Double OFF press - resetting sequence")
                        # Single press - pause sequence
                        system_active = False
                        sequence_paused = True
                        function_state = "Paused"
                        last_off_press_time = current_time
                        print("System paused")
                    elif button_id == 3:  # IC - Individual Control Page
                        display_send("page 1")
                        current_page = 1
                        print("Switched to Individual Control Page")
                    elif button_id == 4:  # CC - Combined Control Page
                        display_send("page 2")
                        current_page = 2
                        print("Switched to Combined Control Page")
                
                # Page 1: Individual Motor Control (Screen2)
                elif page_id == 1:
                    if button_id == 1:  # MW - back to main
                        display_send("page 0")
                        current_page = 0
                        print("Switched to Main Page")
                    else:
                        # Manual control doesn't require system_active for safety
                        if button_id == 2:  # DC1 D1 (CCW)
                            run_motors_timed(-MANUAL_DC_SPEED, 0, MANUAL_DC_DURATION)
                            function_state = "DC1 CCW"
                        elif button_id == 3:  # DC1 D1 (CW)
                            run_motors_timed(MANUAL_DC_SPEED, 0, MANUAL_DC_DURATION)
                            function_state = "DC1 CW"
                        elif button_id == 4:  # DC2 D1 (CW)
                            run_motors_timed(0, MANUAL_DC_SPEED, MANUAL_DC_DURATION)
                            function_state = "DC2 CW"
                        elif button_id == 5:  # DC2 D2 (CCW)
                            run_motors_timed(0, -MANUAL_DC_SPEED, MANUAL_DC_DURATION)
                            function_state = "DC2 CCW"
                        elif button_id == 6:  # St1 D1 (CW)
                            move_stepper('X', 'CW', MANUAL_STEPPER_STEPS)
                            function_state = "Stepper1 CW"
                        elif button_id == 7:  # St1 D2 (CCW)
                            move_stepper('X', 'CCW', MANUAL_STEPPER_STEPS)
                            function_state = "Stepper1 CCW"
                        elif button_id == 8:  # St2 D1 (CW)
                            move_stepper('Y', 'CW', MANUAL_STEPPER_STEPS)
                            function_state = "Stepper2 CW"
                        elif button_id == 9:  # St2 D2 (CCW)
                            move_stepper('Y', 'CCW', MANUAL_STEPPER_STEPS)
                            function_state = "Stepper2 CCW"
                
                # Page 2: Combined Motor Control (Screen3)
                elif page_id == 2:
                    if button_id == 3:  # MW - back to main
                        display_send("page 0")
                        current_page = 0
                        print("Switched to Main Page")
                    else:
                        # Manual control doesn't require system_active for safety
                        if button_id == 4:  # StC D1 (CW)
                            run_stepper_motors('CW', MANUAL_STEPPER_STEPS)
                            function_state = "Steppers CW"
                        elif button_id == 5:  # StC D2 (CCW)
                            run_stepper_motors('CCW', MANUAL_STEPPER_STEPS)
                            function_state = "Steppers CCW"
                        elif button_id == 6:  # DCC D1
                            run_motors_timed(-MANUAL_DC_SPEED, MANUAL_DC_SPEED, MANUAL_DC_DURATION)
                            function_state = "DCs Dir1"
                        elif button_id == 7:  # DCC D2
                            run_motors_timed(MANUAL_DC_SPEED, -MANUAL_DC_SPEED, MANUAL_DC_DURATION)
                            function_state = "DCs Dir2"

def display_update_thread():
    """Thread to handle display communication"""
    global function_state, current_page
    print("Display thread started")
    
    # Initialize display to main page
    display_send("page 0")
    time.sleep(0.1)
    display_send("FunctionValue.txt=\"System Ready\"")
    
    # Setup main page (page 0) - Button IDs: 1=ON, 2=OFF, 3=IC, 4=CC
    display_send("t0.txt=\"ON\"")      # Button 1
    display_send("t1.txt=\"OFF\"")     # Button 2
    display_send("t2.txt=\"IC\"")      # Button 3 - go to individual control
    display_send("t3.txt=\"CC\"")      # Button 4 - go to combined control
    
    # Setup individual control page (page 1)
    display_send("page 1")  # Switch to page 1 to set it up
    time.sleep(0.1)
    # Button IDs for screen2
    display_send("t0.txt=\"MW\"")        # Button 1 - back to main
    display_send("t1.txt=\"DC1 D1\"")    # Button 2 - DC1 CCW
    display_send("t2.txt=\"DC1 D2\"")    # Button 3 - DC1 CW
    display_send("t3.txt=\"DC2 D1\"")    # Button 4 - DC2 CW
    display_send("t4.txt=\"DC2 D2\"")    # Button 5 - DC2 CCW
    display_send("t5.txt=\"St1 D1\"")    # Button 6 - Stepper1 CW
    display_send("t6.txt=\"St1 D2\"")    # Button 7 - Stepper1 CCW
    display_send("t7.txt=\"St2 D1\"")    # Button 8 - Stepper2 CW
    display_send("t8.txt=\"St2 D2\"")    # Button 9 - Stepper2 CCW
    
    # Setup combined control page (page 2)
    display_send("page 2")  # Switch to page 2 to set it up
    time.sleep(0.1)
    # Button IDs for screen3
    display_send("t0.txt=\"\"")         # Not used
    display_send("t1.txt=\"\"")         # Not used
    display_send("t2.txt=\"\"")         # Not used
    display_send("t3.txt=\"MW\"")       # Button 3 - back to main
    display_send("t4.txt=\"StC D1\"")   # Button 4 - Steppers Combined CW
    display_send("t5.txt=\"StC D2\"")   # Button 5 - Steppers Combined CCW
    display_send("t6.txt=\"DCC D1\"")   # Button 6 - DC Combined Direction 1
    display_send("t7.txt=\"DCC D2\"")   # Button 7 - DC Combined Direction 2
    
    # Return to main page
    display_send("page 0")
    current_page = 0
    
    while True:
        try:
            # Only update status on main page
            if current_page == 0:
                # Add cycle/step info to display
                status = f"Cycle {current_cycle+1}/{total_cycles} Step {current_step+1}/{len(sequence_steps)}"
                display_send(f"FunctionValue.txt=\"{function_state}\\n{status}\"")
            
            # Process incoming display messages
            flag, buffer = display_receive()
            if flag:
                process_display_message(buffer)
            
            time.sleep(0.01)
        except Exception as e:
            print(f"Display thread error: {e}")
            time.sleep(1)

# === GPIO Setup ===
def setup_gpio():
    """Initialize GPIO hardware"""
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup DC motor pins
        dc_pins = [M1_R_EN, M1_L_EN, M1_R_PWM, M1_L_PWM,
                   M2_R_EN, M2_L_EN, M2_R_PWM, M2_L_PWM]
        for pin in dc_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Setup stepper motor pins
        stepper_pins = [X_STEP, X_DIR, Y_STEP, Y_DIR]
        for pin in stepper_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Setup limit switches with pull-ups
        limit_pins = [X_LIMIT_1, X_LIMIT_2, Y_LIMIT_1, Y_LIMIT_2]
        for pin in limit_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Setup encoder pins with pull-ups
        encoder_pins = [ENC1_A, ENC1_B, ENC2_A, ENC2_B]
        for pin in encoder_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize PWM on DC motors
        pwm_objects[M1_R_PWM] = GPIO.PWM(M1_R_PWM, PWM_FREQ)
        pwm_objects[M1_L_PWM] = GPIO.PWM(M1_L_PWM, PWM_FREQ)
        pwm_objects[M2_R_PWM] = GPIO.PWM(M2_R_PWM, PWM_FREQ)
        pwm_objects[M2_L_PWM] = GPIO.PWM(M2_L_PWM, PWM_FREQ)
        
        # Start all PWM channels at 0% duty cycle
        for pwm in pwm_objects.values():
            pwm.start(0)
        
        # Get initial encoder states
        global encoder1_last_state, encoder2_last_state
        encoder1_last_state = (GPIO.input(ENC1_A) << 1 | GPIO.input(ENC1_B))
        encoder2_last_state = (GPIO.input(ENC2_A) << 1 | GPIO.input(ENC2_B))
        
    except Exception as e:
        print(f"GPIO setup failed: {e}")
        stop_all()
        raise

# === UPDATED Encoder Polling Thread ===
def encoder_polling_thread():
    """Thread that continuously polls encoder states with consistent logic"""
    global encoder1_count, encoder2_count
    global encoder1_last_state, encoder2_last_state, encoder_polling_active
    
    print("Encoder polling thread started")
    
    # Consistent transition table across both implementations
    transitions = {
        (0, 1): 1,   # 00 -> 01: CW
        (0, 2): -1,  # 00 -> 10: CCW
        (1, 3): -1,  # 01 -> 11: CCW
        (1, 0): 1,   # 01 -> 00: CW
        (2, 0): 1,   # 10 -> 00: CW
        (2, 3): -1,  # 10 -> 11: CCW
        (3, 2): 1,   # 11 -> 10: CW
        (3, 1): -1   # 11 -> 01: CCW
    }
    
    while encoder_polling_active:
        try:
            # For encoder 1
            a1 = GPIO.input(ENC1_A)
            b1 = GPIO.input(ENC1_B)
            new_state1 = (a1 << 1) | b1
            
            # For encoder 2
            a2 = GPIO.input(ENC2_A)
            b2 = GPIO.input(ENC2_B)
            new_state2 = (a2 << 1) | b2
            
            # Update encoder 1
            direction1 = transitions.get((encoder1_last_state, new_state1), 0)
            if direction1:
                encoder1_count += direction1
                encoder1_last_state = new_state1
            
            # Update encoder 2
            direction2 = transitions.get((encoder2_last_state, new_state2), 0)
            if direction2:
                encoder2_count += direction2
                encoder2_last_state = new_state2
            
            time.sleep(0.001)  # Poll at 1ms intervals
        except Exception as e:
            if encoder_polling_active:
                print(f"Encoder polling error: {e}")
            break

# === Interrupt Handler ===
def limit_switch_callback(channel):
    """Callback for any limit switch activation"""
    global emergency_stop, x_max_triggered, y_max_triggered, function_state
    global current_step, system_active, sequence_paused, in_homing_sequence
    
    # Only trigger on falling edge (switch pressed)
    if GPIO.input(channel) == GPIO.LOW:
        if channel in [X_LIMIT_2, Y_LIMIT_2] and not in_homing_sequence:
            # Treat as OFF button press (but don't reset sequence)
            print(f"Limit switch {channel} pressed - acting as OFF button")
            system_active = False
            sequence_paused = True
            function_state = f"Paused: Limit {channel}"
        
        if channel == X_LIMIT_2:  # X max limit switch
            print(f"EMERGENCY STOP: X-axis max limit switch triggered!")
            x_max_triggered = True
            emergency_stop = True
            function_state = "STOPPED: X-Limit"
            current_step = 0  # Reset sequence position
        elif channel == Y_LIMIT_2:  # Y max limit switch
            print(f"EMERGENCY STOP: Y-axis max limit switch triggered!")
            y_max_triggered = True
            emergency_stop = True
            function_state = "STOPPED: Y-Limit"
            current_step = 0  # Reset sequence position
        else:
            # For min limit switches, log but don't emergency stop
            print(f"Limit switch {channel} triggered (non-emergency)")

# === Motor Control Functions ===
def enable_all_motor_drivers():
    """Enable all motor driver chips"""
    for pin in [M1_R_EN, M1_L_EN, M2_R_EN, M2_L_EN]:
        GPIO.output(pin, GPIO.HIGH)

def set_motor_speed(motor, speed):
    """Set motor speed with direction based on sign and minimum PWM"""
    # Apply minimum PWM to overcome friction
    effective_speed = math.copysign(max(MIN_PWM, min(MAX_SPEED, abs(speed))), speed)
    
    if motor == 1:
        if effective_speed >= 0:
            # FORWARD: Enable R_PWM, disable L_PWM
            pwm_objects[M1_R_PWM].ChangeDutyCycle(abs(effective_speed))
            pwm_objects[M1_L_PWM].ChangeDutyCycle(0)
        else:
            # REVERSE: Enable L_PWM, disable R_PWM
            pwm_objects[M1_R_PWM].ChangeDutyCycle(0)
            pwm_objects[M1_L_PWM].ChangeDutyCycle(abs(effective_speed))
    elif motor == 2:
        if effective_speed >= 0:
            # FORWARD: Enable L_PWM, disable R_PWM (inverted for motor2)
            pwm_objects[M2_R_PWM].ChangeDutyCycle(0)
            pwm_objects[M2_L_PWM].ChangeDutyCycle(abs(effective_speed))
        else:
            # REVERSE: Enable R_PWM, disable L_PWM (inverted for motor2)
            pwm_objects[M2_R_PWM].ChangeDutyCycle(abs(effective_speed))
            pwm_objects[M2_L_PWM].ChangeDutyCycle(0)

def run_motors_timed(motor1_speed, motor2_speed, duration=DC_RUN_TIME):
    """Run both DC motors for specified duration with individual speed control"""
    global emergency_stop, dc_active, function_state, sequence_paused
    
    if emergency_stop:
        return
    
    emergency_stop = False
    dc_active = True
    function_state = "Running DC Motors"
    
    enable_all_motor_drivers()
    
    # Record starting encoder positions for reporting
    start_enc1 = encoder1_count
    start_enc2 = encoder2_count
    
    start_time = time.time()
    last_print_time = time.time()
    
    print(f"\nRunning motors for {duration} seconds")
    print(f"Motor1 Speed: {motor1_speed}% | Motor2 Speed: {motor2_speed}%")
    
    # Apply motor speeds
    set_motor_speed(1, motor1_speed)
    set_motor_speed(2, motor2_speed)
    
    while not emergency_stop:
        current_time = time.time()
        elapsed = current_time - start_time
        
        # Safety timeout
        if elapsed >= duration:
            print("Time duration reached, stopping motors")
            break
            
        # Print status every 0.5 seconds
        if current_time - last_print_time > 0.5:
            # Calculate relative movement from start
            rel_count1 = encoder1_count - start_enc1
            rel_count2 = encoder2_count - start_enc2
            
            # Calculate speeds (counts per second)
            speed1 = abs(rel_count1) / elapsed if elapsed > 0 else 0
            speed2 = abs(rel_count2) / elapsed if elapsed > 0 else 0
            
            print(f"Motor1: {rel_count1} counts @ {speed1:.1f} cps | "
                  f"Motor2: {rel_count2} counts @ {speed2:.1f} cps | "
                  f"Time: {elapsed:.1f}/{duration}s")
            last_print_time = current_time
        
        time.sleep(0.01)
    
    stop_motors()
    dc_active = False
    function_state = "DC Motors Stopped"
    
    # Final counts
    final_count1 = encoder1_count - start_enc1
    final_count2 = encoder2_count - start_enc2
    print(f"Motor run complete. Motor1 moved: {final_count1}, Motor2 moved: {final_count2}")
    print(f"Final difference: {abs(final_count1 - final_count2)} counts")

def stop_motors():
    """Stop both DC motors immediately"""
    for pwm_pin in [M1_R_PWM, M1_L_PWM, M2_R_PWM, M2_L_PWM]:
        try:
            pwm_objects[pwm_pin].ChangeDutyCycle(0)
        except KeyError:
            pass

# === Enhanced Stepper Homing ===
def home_steppers_together():
    """Home both X and Y steppers simultaneously with independent stopping"""
    global emergency_stop, current_x_position, current_y_position
    global x_max_triggered, y_max_triggered, stepper_active, function_state
    global sequence_paused, in_homing_sequence
    
    if emergency_stop:
        return
    
    # Set homing flag
    in_homing_sequence = True
    
    # Reset emergency flags
    emergency_stop = False
    x_max_triggered = False
    y_max_triggered = False
    stepper_active = True
    function_state = "Homing Steppers"
    
    # Set directions for homing (CCW for both)
    GPIO.output(X_DIR, GPIO.LOW)
    GPIO.output(Y_DIR, GPIO.LOW)
    print("Homing both steppers simultaneously...")
    
    # Initialize homing status and counters
    x_homed = False
    y_homed = False
    step_count = 0
    extra_steps_after_x_home = 0
    extra_steps_after_y_home = 0
    
    # Move until both limit switches are activated or max extra steps reached
    while (not x_homed or not y_homed) and not emergency_stop:
        # Step X motor if not yet homed and Y hasn't been homed for too long
        step_x = not x_homed and extra_steps_after_y_home < MAX_EXTRA_STEPS
        step_y = not y_homed and extra_steps_after_x_home < MAX_EXTRA_STEPS
        
        if step_x:
            GPIO.output(X_STEP, GPIO.HIGH)
        
        if step_y:
            GPIO.output(Y_STEP, GPIO.HIGH)
        
        # Pulse delay
        time.sleep(STEPPER_DELAY)
        
        # Lower step pins
        GPIO.output(X_STEP, GPIO.LOW)
        GPIO.output(Y_STEP, GPIO.LOW)
        time.sleep(STEPPER_DELAY)
        
        # Check limit switches
        if not x_homed and GPIO.input(X_LIMIT_1) == GPIO.LOW:  # Active low
            x_homed = True
            print("X home found")
        
        if not y_homed and GPIO.input(Y_LIMIT_1) == GPIO.LOW:  # Active low
            y_homed = True
            print("Y home found")
        
        # Update extra step counters
        if x_homed and not y_homed:
            extra_steps_after_x_home += 1
        if y_homed and not x_homed:
            extra_steps_after_y_home += 1
        
        step_count += 1
        
        # Print status every 100 steps
        if step_count % 100 == 0:
            status = f"Steps: {step_count}, "
            status += f"X: {'HOMED' if x_homed else 'homing'}, "
            status += f"Y: {'HOMED' if y_homed else 'homing'}, "
            status += f"Extra steps after home: X={extra_steps_after_x_home}, Y={extra_steps_after_y_home}"
            print(status)
    
    # Clear homing flag
    in_homing_sequence = False
    
    # Check why we exited the loop
    if emergency_stop:
        print(f"Homing aborted after {step_count} steps")
        stepper_active = False
        function_state = "Homing Aborted"
        return
    
    # Check if one axis didn't home
    if not x_homed and extra_steps_after_y_home >= MAX_EXTRA_STEPS:
        print(f"X axis failed to home after {MAX_EXTRA_STEPS} extra steps!")
        emergency_stop = True
        stepper_active = False
        function_state = "Homing Failed: X"
        return
    
    if not y_homed and extra_steps_after_x_home >= MAX_EXTRA_STEPS:
        print(f"Y axis failed to home after {MAX_EXTRA_STEPS} extra steps!")
        emergency_stop = True
        stepper_active = False
        function_state = "Homing Failed: Y"
        return
    
    print(f"Both axes homed after {step_count} steps")
    
    # Back off each axis individually
    print("Backing off X axis...")
    backoff_stepper('X', STEPPER_BACKOFF_STEPS, 'CW')
    
    print("Backing off Y axis...")
    backoff_stepper('Y', STEPPER_BACKOFF_STEPS, 'CW')
    
    # Reset positions
    current_x_position = 0
    current_y_position = 0
    
    # Move both to home position simultaneously
    print(f"Moving to home position ({STEPPER_HOME_POSITION} steps)...")
    run_stepper_motors('CW', STEPPER_HOME_POSITION)
    
    stepper_active = False
    function_state = "Homing Complete"
    print("Stepper homing complete")

def backoff_stepper(axis, steps, direction):
    """Back off a single stepper after homing"""
    global emergency_stop
    
    if axis == 'X':
        step_pin = X_STEP
        dir_pin = X_DIR
    elif axis == 'Y':
        step_pin = Y_STEP
        dir_pin = Y_DIR
    else:
        return
    
    GPIO.output(dir_pin, GPIO.HIGH if direction == 'CW' else GPIO.LOW)
    
    for i in range(steps):
        if emergency_stop:
            print(f"{axis} backoff paused at step {i+1}")
            return
            
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(STEPPER_DELAY)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(STEPPER_DELAY)

def run_stepper_motors(direction, steps):
    """Move both stepper motors simultaneously"""
    global current_x_position, current_y_position, emergency_stop
    global stepper_active, x_max_triggered, y_max_triggered, function_state
    
    if emergency_stop:
        return 0
    
    # Reset flags
    emergency_stop = False
    x_max_triggered = False
    y_max_triggered = False
    stepper_active = True
    function_state = "Moving Steppers"
    
    # Set direction
    GPIO.output(X_DIR, GPIO.HIGH if direction == 'CW' else GPIO.LOW)
    GPIO.output(Y_DIR, GPIO.HIGH if direction == 'CW' else GPIO.LOW)
    print(f"Moving both steppers {direction} for {steps} steps")
    
    # Move both motors
    steps_completed = 0
    for i in range(steps):
        if emergency_stop:
            print(f"Stepper movement stopped at step {i}")
            return steps_completed
            
        # Step both motors
        GPIO.output(X_STEP, GPIO.HIGH)
        GPIO.output(Y_STEP, GPIO.HIGH)
        time.sleep(STEPPER_DELAY)
        GPIO.output(X_STEP, GPIO.LOW)
        GPIO.output(Y_STEP, GPIO.LOW)
        time.sleep(STEPPER_DELAY)
        
        # Update positions
        if direction == 'CW':
            current_x_position += 1
            current_y_position += 1
        else:
            current_x_position -= 1
            current_y_position -= 1
            
        steps_completed += 1
        
        if i % 100 == 0:
            print(f"  Step {i}/{steps}, X: {current_x_position}, Y: {current_y_position}")
    
    stepper_active = False
    function_state = "Steppers Stopped"
    print(f"Moved {steps_completed} of {steps} steps")
    return steps_completed

def move_stepper(axis, direction, steps):
    """Move a single stepper motor"""
    global emergency_stop, stepper_active, function_state
    
    if emergency_stop:
        return
    
    # Reset flags
    emergency_stop = False
    stepper_active = True
    function_state = f"Moving {axis} Stepper"
    
    # Set direction and pins
    if axis == 'X':
        step_pin = X_STEP
        dir_pin = X_DIR
    elif axis == 'Y':
        step_pin = Y_STEP
        dir_pin = Y_DIR
    else:
        return
    
    GPIO.output(dir_pin, GPIO.HIGH if direction == 'CW' else GPIO.LOW)
    print(f"Moving {axis} stepper {direction} for {steps} steps")
    
    # Move motor
    for i in range(steps):
        if emergency_stop:
            print(f"Stepper movement stopped at step {i}")
            return
            
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(STEPPER_DELAY)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(STEPPER_DELAY)
        
        # Update position
        if axis == 'X':
            current_x_position += 1 if direction == 'CW' else -1
        else:
            current_y_position += 1 if direction == 'CW' else -1
    
    stepper_active = False
    function_state = "Stepper Stopped"
    print(f"{axis} stepper moved {steps} steps")

# === Cleanup ===
def stop_all():
    """Complete system shutdown"""
    global emergency_stop, encoder_polling_active, system_active
    
    print("\nInitiating system shutdown...")
    system_active = False
    
    # Set flags to stop threads
    emergency_stop = True
    encoder_polling_active = False
    
    # Stop all motors
    try:
        stop_motors()
    except:
        pass
    
    # Disable motor drivers
    for pin in [M1_R_EN, M1_L_EN, M2_R_EN, M2_L_EN]:
        try:
            GPIO.output(pin, GPIO.LOW)
        except:
            pass
    
    # Reset stepper pins
    for pin in [X_STEP, X_DIR, Y_STEP, Y_DIR]:
        try:
            GPIO.output(pin, GPIO.LOW)
        except:
            pass
    
    # Stop PWM channels
    for pwm in pwm_objects.values():
        try:
            pwm.stop()
        except:
            pass
    
    # Close serial connection
    global ser
    if ser is not None:
        try:
            ser.close()
        except:
            pass
    
    # Cleanup GPIO
    try:
        GPIO.cleanup()
        print("GPIO cleanup complete")
    except:
        pass
    
    print("System halted")

# === Main Sequence ===
def run_main_sequence():
    """Run the complete motor sequence for 10 cycles with pause/resume support"""
    global function_state, sequence_running, system_active
    global current_cycle, current_step, sequence_paused
    
    sequence_running = True
    function_state = "Sequence Running"
    
    try:
        # Run cycles
        while current_cycle < total_cycles and system_active:
            function_state = f"Cycle {current_cycle+1}/{total_cycles}"
            print(f"\n===== STARTING CYCLE {current_cycle+1}/{total_cycles} =====")
            
            # Start from current step in sequence
            step_index = current_step
            while step_index < len(sequence_steps) and system_active:
                if sequence_paused:
                    print("\nSequence paused during step execution")
                    function_state = f"Paused at Cycle {current_cycle+1} Step {step_index+1}"
                    current_step = step_index  # Remember where we paused
                    break
                
                step_info = sequence_steps[step_index]
                function_state = f"Cycle {current_cycle+1}: {step_info['name']}"
                print(f"\nStarting step {step_index+1}/{len(sequence_steps)}: {step_info['name']}")
                
                # Execute the step function with arguments
                if step_info['func'] == "run_motors_timed":
                    run_motors_timed(*step_info['args'])
                elif step_info['func'] == "run_stepper_motors":
                    run_stepper_motors(*step_info['args'])
                elif step_info['func'] == "home_steppers_together":
                    home_steppers_together()
                
                # Move to next step
                step_index += 1
                current_step = step_index  # Update global step counter
            
            # Check if we completed all steps for this cycle
            if step_index >= len(sequence_steps) and system_active:
                print(f"\n===== COMPLETED CYCLE {current_cycle+1}/{total_cycles} =====")
                current_cycle += 1
                current_step = 0  # Reset step counter for next cycle
        
        # Check completion status
        if current_cycle >= total_cycles:
            function_state = "Sequence Complete"
            print("\nOperation completed successfully")
            # Reset for next run
            current_cycle = 0
            current_step = 0
        elif sequence_paused:
            print("\nSequence paused by user")
        else:
            function_state = "Sequence Stopped"
            print("\nSequence stopped before completing")
        
    finally:
        sequence_running = False

# === Main Execution ===
if __name__ == "__main__":
    try:
        # Initialize display
        if not init_display():
            print("Failed to initialize display, continuing without it")
        else:
            print("Display initialized successfully")
        
        # Start display thread
        display_thread = threading.Thread(target=display_update_thread, daemon=True)
        display_thread.start()
        time.sleep(1)  # Let display initialize
        
        # Initialize GPIO
        setup_gpio()
        
        # Register callbacks for max limit switches
        GPIO.add_event_detect(X_LIMIT_2, GPIO.FALLING, callback=limit_switch_callback, bouncetime=50)
        GPIO.add_event_detect(Y_LIMIT_2, GPIO.FALLING, callback=limit_switch_callback, bouncetime=50)
        print("Limit switch callbacks registered")
        
        # Create and start encoder polling thread
        encoder_thread = threading.Thread(target=encoder_polling_thread, daemon=True)
        encoder_thread.start()
        time.sleep(0.1)  # Let encoder thread start
        
        # Test encoder pins
        print("Testing encoder pins...")
        print(f"ENC1_A: {GPIO.input(ENC1_A)}, ENC1_B: {GPIO.input(ENC1_B)}")
        print(f"ENC2_A: {GPIO.input(ENC2_A)}, ENC2_B: {GPIO.input(ENC2_B)}")
        
        # Main operation loop
        print("Waiting for display activation...")
        while True:
            if system_active and not emergency_stop and not sequence_running:
                print("Starting main sequence")
                run_main_sequence()
            else:
                # System inactive or paused, sleep to reduce CPU usage
                time.sleep(0.1)

    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        stop_all()
        print("Cleanup complete")