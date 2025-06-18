#!/usr/bin/env python3

import threading
import time
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from fcuflex_ros2.msg import AFDStatus
from fcuflex_ros2.srv import GetValue, SetValue

import dearpygui.dearpygui as dpg

# Utility to sanitize parameter names for use as DearPyGui tags
def sanitize(param_name):
    return param_name.strip('/').replace('/', '_')

# Constants for control modes and units
CONTROL_MODES = {
    "0": "Position Mode",    # BIT0 = 0: Position Mode
    "1": "Force Mode",       # BIT0 = 1: Force Mode
    "2": "Position + SoftTouch",  # BIT0 = 0, BIT1 = 1
    "3": "Force + SoftTouch",     # BIT0 = 1, BIT1 = 1
    "7": "Force + SoftTouch (Alt)"  # BIT0-2 all set
}

# Create a reverse mapping for sending commands
MODE_COMMAND_VALUES = {
    "Position Mode": "0",        # Binary: 00000000
    "Force Mode": "1",           # Binary: 00000001
    "Position + SoftTouch": "2", # Binary: 00000010
    "Force + SoftTouch": "3",    # Binary: 00000011
    "Force + SoftTouch (Alt)": "3"  # Send as 3
}

UNIT_OPTIONS = {
    "0": "Standard (Imperial)",
    "1": "Metric"
}

# Unit-specific axis labels and conversion factors
UNIT_LABELS = {
    "0": {  # Standard/Imperial
        "force": "Force (lbf)",
        "position": "Position (in)"
    },
    "1": {  # Metric
        "force": "Force (N)",
        "position": "Position (mm)"
    }
}

# Conversion factors (metric to imperial)
UNIT_CONVERSIONS = {
    "force": {
        "to_imperial": 0.224809,  # N to lbf
        "to_metric": 4.44822      # lbf to N
    },
    "position": {
        "to_imperial": 0.0393701, # mm to inches
        "to_metric": 25.4         # inches to mm
    }
}

# Full manufacturer parameters list
PARAMETERS = [
    {'name':'/afd/help','desc':'Help','read':True,'write':False},
    {'name':'/afd/firmWareVersion','desc':'Firmware Version','read':True,'write':False},
    {'name':'/afd/hardWareVersion','desc':'Hardware Version','read':True,'write':False},
    {'name':'/afd/modelName','desc':'Model Name','read':True,'write':False},
    {'name':'/afd/serialNo','desc':'Serial Number','read':True,'write':False},
    {'name':'/afd/metricUnits','desc':'Metric Units','read':True,'write':True},
    {'name':'/afd/maxForce','desc':'Max Force','read':True,'write':False},
    {'name':'/afd/maxPosition','desc':'Max Position','read':True,'write':False},
    {'name':'/afd/deviceName','desc':'Device Name','read':True,'write':True},
    {'name':'/afd/initialWeigh','desc':'Initial Weigh','read':True,'write':True},
    {'name':'/afd/initialMode','desc':'Initial Mode','read':True,'write':True},
    {'name':'/afd/initialForce','desc':'Initial Force','read':True,'write':True},
    {'name':'/afd/initialPosition','desc':'Initial Position','read':True,'write':True},
    {'name':'/afd/posLimitPosition','desc':'Pos Limit Position','read':True,'write':True},
    {'name':'/afd/negLimitPosition','desc':'Neg Limit Position','read':True,'write':True},
    {'name':'/afd/posLimit','desc':'Position Limit Status','read':True,'write':False},
    {'name':'/afd/softTouchEnabled','desc':'Soft Touch Enabled','read':True,'write':True},
    {'name':'/afd/softTouchForce','desc':'Soft Touch Force','read':True,'write':True},
    {'name':'/afd/softTouchPosition','desc':'Soft Touch Position','read':True,'write':True},
    {'name':'/afd/softTouchActive','desc':'Soft Touch Active','read':True,'write':False},
    {'name':'/afd/controlMode','desc':'Control Mode','read':True,'write':True},
    {'name':'/afd/commandForce','desc':'Cmd Force','read':True,'write':True},
    {'name':'/afd/commandPosition','desc':'Cmd Position','read':True,'write':True},
    {'name':'/afd/payloadWeight','desc':'Payload Weight','read':True,'write':False},
    {'name':'/afd/weighPayload','desc':'Weigh Payload','read':True,'write':True},
    {'name':'/afd/weighAtPosition','desc':'Weigh At Position','read':True,'write':False},
    {'name':'/afd/weighingInProgress','desc':'Weighing In Progress','read':True,'write':False},
    {'name':'/afd/weightValid','desc':'Weight Valid','read':True,'write':False},
    {'name':'/afd/actualForce','desc':'Actual Force','read':True,'write':False},
    {'name':'/afd/actualPosition','desc':'Actual Position','read':True,'write':False},
    {'name':'/afd/accelGravity','desc':'Accel Gravity','read':True,'write':False},
    {'name':'/afd/stateObject','desc':'State Object','read':True,'write':False},
    {'name':'/afd/dumpCfg','desc':'Dump Config','read':True,'write':False},
    {'name':'/afd/saveConfig','desc':'Save Config','read':False,'write':True},
]

class FCUFLEXPlotterNode(Node):
    def __init__(self):
        super().__init__('fcuflex_plotter')
        # Plot and data settings
        self.topic_name = '/fcuflex_node/status'
        self.max_points = 1000
        self.update_rate = 10.0
        self.force_min = -180.0
        self.force_max = 180.0
        self.position_min = 0.0
        self.position_max = 1.55
        # Data buffers
        self.times = []
        self.force_data = []
        self.position_data = []
        # Live status fields
        self.status_fields = {}
        self.collecting_data = False
        self.start_time = None
        self.lock = threading.Lock()
        # Plot auto-scaling
        self.update_plot_limits = False
        self.time_window = 60.0  # Show last 60 seconds by default
        self.auto_scroll = True  # Auto-scroll enabled by default
        # Units tracking
        self.current_units = "1"  # Default to metric
        self.units_changed = False
        # Mode tracking
        self.is_position_mode = True  # Default to position mode
        # ROS subscription
        self.create_subscription(AFDStatus, self.topic_name, self.status_callback, 10)
        # Parameter service clients
        self.get_clients = {p['name']: self.create_client(GetValue, '/fcuflex_node/get_param') for p in PARAMETERS}
        self.set_clients = {p['name']: self.create_client(SetValue, '/fcuflex_node/set_param') for p in PARAMETERS if p['write']}
        # Command service clients
        self.force_client = self.create_client(SetValue, '/fcuflex_node/set_force')
        self.pos_client = self.create_client(SetValue, '/fcuflex_node/set_position')
        self.mode_client = self.create_client(SetValue, '/fcuflex_node/set_control_mode')

    def status_callback(self, msg: AFDStatus):
        with self.lock:
            # Update live stats
            for key in ['actual_force','actual_position','command_force','command_position',
                        'payload_weight','accel_gravity','control_mode','tool_connected']:
                self.status_fields[key] = getattr(msg, key)
            
            # Check mode bitmap
            try:
                control_mode = int(self.status_fields.get('control_mode', 0))
                self.is_position_mode = (control_mode & 0x01) == 0  # Check if BIT0 is 0
            except (ValueError, TypeError):
                pass
                
            # Append for plots
            if self.collecting_data:
                elapsed = time.time() - self.start_time
                self.times.append(elapsed)
                self.force_data.append(msg.actual_force)
                self.position_data.append(msg.actual_position)
                if len(self.times) > self.max_points:
                    self.times.pop(0)
                    self.force_data.pop(0)
                    self.position_data.pop(0)
                # Update the plot time window to show the latest data
                self.update_plot_limits = True

# GUI callbacks

def get_param_cb(sender, app_data, user_data):
    node, param = user_data
    tag = f"val_{sanitize(param)}"
    client = node.get_clients[param]
    if client.wait_for_service(0.1):
        req = GetValue.Request(); req.param = param
        fut = client.call_async(req)
        fut.add_done_callback(lambda f: dpg.set_value(tag, f.result().value))


def set_param_cb(sender, app_data, user_data):
    node, param = user_data
    client = node.set_clients[param]
    tag = f"in_{sanitize(param)}"
    if client.wait_for_service(0.1):
        val = dpg.get_value(tag)
        # For dropdown parameters, convert label back to value
        if param == '/afd/metricUnits':
            for value, label in UNIT_OPTIONS.items():
                if label == val:
                    val = value
                    # Track units change for plot label updates
                    if node.current_units != val:
                        node.current_units = val
                        node.units_changed = True
                    break
        elif param == '/afd/controlMode':
            for value, label in CONTROL_MODES.items():
                if label == val:
                    val = value
                    break
        req = SetValue.Request(); req.param = param; req.value = str(val)
        client.call_async(req)


def send_force(sender, app_data, user_data):
    node = user_data
    val = dpg.get_value('cmd_force')
    
    # If we're in position mode, try to switch to force mode first
    if node.is_position_mode:
        print("In Position Mode, switching to Force Mode before sending force command...")
        set_force_mode(node)
        time.sleep(0.2)  # Small delay
    
    print(f"Sending force command: {val}")
    if node.force_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = str(val)
        node.force_client.call_async(req)


def send_position(sender, app_data, user_data):
    node = user_data
    val = dpg.get_value('cmd_pos')
    
    # Check if we're in position control mode by looking at BIT0 of control mode
    try:
        if not node.is_position_mode:
            print("Not in Position Mode. Attempting to set Position Mode (BIT0=0)...")
            set_position_mode(node)
            time.sleep(0.2)  # Small delay to allow mode change
        else:
            print("Device is in Position Mode. Sending position command...")
    except Exception as e:
        print(f"Error checking mode: {e}")
    
    # Send the position command
    print(f"Sending position command: {val}")
    if node.pos_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = str(val)
        node.pos_client.call_async(req)


def send_mode(sender, app_data, user_data):
    node = user_data
    mode_label = dpg.get_value('cmd_mode')
    
    # Use the dedicated command value mapping
    if mode_label in MODE_COMMAND_VALUES:
        mode_value = MODE_COMMAND_VALUES[mode_label]
    else:
        mode_value = "0"  # Default to Position Mode if not found
    
    # Convert to integer for binary representation
    try:
        mode_int = int(mode_value)
        is_position_mode = (mode_int & 0x01) == 0  # Check if BIT0 is 0
        mode_type = "Position Mode" if is_position_mode else "Force Mode"
        print(f"Setting mode to: {mode_label}")
        print(f"Value: {mode_value} (binary: {bin(mode_int)})")
        print(f"Mode type: {mode_type}")
    except ValueError:
        print(f"Setting mode to: {mode_label} (value: {mode_value})")
    
    if node.mode_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = str(mode_value)
        node.mode_client.call_async(req)

# Helpers

def save_csv(node):
    with node.lock:
        if not node.times: return
        fname = datetime.now().strftime('fcuflex_%Y%m%d_%H%M%S.csv')
        with open(fname,'w',newline='') as f:
            w = csv.writer(f); w.writerow(['Time','Force','Pos'])
            for t,fv,pv in zip(node.times,node.force_data,node.position_data): w.writerow([t,fv,pv])


def clear_data(node):
    with node.lock:
        node.times.clear(); node.force_data.clear(); node.position_data.clear()


# Helper to display readable mode and unit values
def format_display_value(param_name, value):
    try:
        if param_name == '/afd/controlMode' and value in CONTROL_MODES:
            return f"{CONTROL_MODES[value]} ({value})"
        elif param_name == '/afd/metricUnits' and value in UNIT_OPTIONS:
            return f"{UNIT_OPTIONS[value]} ({value})"
        return value
    except:
        return value


# Custom callback for getting values that need special display formatting
def get_param_with_format_cb(sender, app_data, user_data):
    node, param = user_data
    tag = f"val_{sanitize(param)}"
    client = node.get_clients[param]
    if client.wait_for_service(0.1):
        req = GetValue.Request(); req.param = param
        fut = client.call_async(req)
        fut.add_done_callback(lambda f: dpg.set_value(tag, format_display_value(param, f.result().value)))


# Helper function to set position mode using bitmap understanding
def set_position_mode(node):
    # To set Position Mode: BIT0 must be 0
    # We'll use 0 for basic Position Mode or 2 for Position+SoftTouch
    mode_value = "0"  # Binary: 00000000 (Position Mode, no SoftTouch)
    
    print(f"Setting Position Mode with value: {mode_value} (BIT0=0)")
    if node.mode_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = mode_value
        node.mode_client.call_async(req)


# Helper function to set force mode using bitmap understanding
def set_force_mode(node):
    # To set Force Mode: BIT0 must be 1
    # We'll use 1 for basic Force Mode or 3 for Force+SoftTouch
    mode_value = "1"  # Binary: 00000001 (Force Mode, no SoftTouch)
    
    print(f"Setting Force Mode with value: {mode_value} (BIT0=1)")
    if node.mode_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = mode_value
        node.mode_client.call_async(req)


# Helper to check current control mode with bitmap interpretation
def get_current_mode_bitmap(node):
    try:
        mode = node.status_fields.get('control_mode')
        if mode is not None:
            mode_int = int(mode)
            is_force_mode = (mode_int & 0x01) > 0  # Check BIT0
            is_soft_touch = (mode_int & 0x02) > 0  # Check BIT1
            
            mode_desc = "Force Mode" if is_force_mode else "Position Mode"
            if is_soft_touch:
                mode_desc += " with SoftTouch"
                
            print(f"Control mode: {mode} (binary: {bin(mode_int)})")
            print(f"Mode interpretation: {mode_desc}")
            print(f"BIT0 (Force/Position): {'ON (Force)' if is_force_mode else 'OFF (Position)'}")
            print(f"BIT1 (SoftTouch): {'ON' if is_soft_touch else 'OFF'}")
            
            # Extract other bitmap values if present
            if mode_int > 3:
                soft_touch_pos = (mode_int >> 3) & 0x1F  # BIT3-BIT7
                soft_touch_force = (mode_int >> 8) & 0x1F  # BIT8-BIT12
                print(f"SoftTouchPosition scale: {soft_touch_pos}/31")
                print(f"SoftTouchForce scale: {soft_touch_force}/31")
            
            return is_force_mode, is_soft_touch
    except (ValueError, TypeError) as e:
        print(f"Error interpreting control mode: {e}")
        print(f"Raw control mode value: {mode}")
        return None, None


# Function to build and send a custom control mode bitmap
def send_custom_mode_bitmap(node):
    # Build the bitmap value from UI components
    bit0 = 1 if dpg.get_value("bit0_force_mode") else 0
    bit1 = 1 if dpg.get_value("bit1_soft_touch") else 0
    soft_touch_pos = dpg.get_value("soft_touch_pos") & 0x1F  # Ensure 5 bits only
    soft_touch_force = dpg.get_value("soft_touch_force") & 0x1F  # Ensure 5 bits only
    
    # Construct the bitmap value
    bitmap_value = (bit0) | (bit1 << 1) | (soft_touch_pos << 3) | (soft_touch_force << 8)
    
    # Display the composed bitmap
    print(f"Custom Control Mode Bitmap: {bitmap_value} (binary: {bin(bitmap_value)})")
    print(f"BIT0 (Force/Position): {'ON (Force)' if bit0 else 'OFF (Position)'}")
    print(f"BIT1 (SoftTouch): {'ON' if bit1 else 'OFF'}")
    print(f"SoftTouch Position (BIT3-7): {soft_touch_pos}/31")
    print(f"SoftTouch Force (BIT8-12): {soft_touch_force}/31")
    
    # Send the bitmap value to the device
    if node.mode_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = str(bitmap_value)
        node.mode_client.call_async(req)


# Helper function to send raw mode value for debugging
def send_raw_mode(node):
    raw_value = dpg.get_value("raw_mode_value")
    print(f"Sending raw mode value: {raw_value}")
    if node.mode_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = raw_value
        node.mode_client.call_async(req)


# Helper to print all device status information
def print_device_status(node):
    print("\n--- DEVICE STATUS ---")
    with node.lock:
        for key, val in node.status_fields.items():
            print(f"{key}: {val}")
    print("-------------------\n")
    get_current_mode_bitmap(node)


# Apply settings function
def apply_settings(node):
    node.topic_name = dpg.get_value('setting_topic')
    node.max_points = dpg.get_value('setting_max_pts')
    node.update_rate = dpg.get_value('setting_rate')
    node.force_min, node.force_max = dpg.get_value('setting_fmin'), dpg.get_value('setting_fmax')
    node.position_min, node.position_max = dpg.get_value('setting_pmin'), dpg.get_value('setting_pmax')
    node.time_window = dpg.get_value('setting_time_window')
    node.auto_scroll = dpg.get_value('setting_auto_scroll')
    # Re-subscribe if topic changed
    node.create_subscription(AFDStatus, node.topic_name, node.status_callback, 10)
    # Update axis limits
    dpg.set_axis_limits('force_y', node.force_min, node.force_max)
    dpg.set_axis_limits('pos_y', node.position_min, node.position_max)
    node.update_plot_limits = True  # Trigger x-axis update


# Convert data values based on units
def convert_values_for_display(node, force_data, position_data):
    """Convert force and position data to the currently selected units for display"""
    if not force_data or not position_data:
        return [], []
    
    # Make a copy to avoid modifying the original data
    force_values = force_data.copy()
    position_values = position_data.copy()
    
    # Convert if in imperial mode
    if node.current_units == "0":  # Imperial
        force_values = [f * UNIT_CONVERSIONS["force"]["to_imperial"] for f in force_values]
        position_values = [p * UNIT_CONVERSIONS["position"]["to_imperial"] for p in position_values]
        
    return force_values, position_values


# GUI Construction and Main Loop
def main():
    rclpy.init()
    node = FCUFLEXPlotterNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    dpg.create_context()
    dpg.create_viewport(title='FCUFLEX Plotter', width=1400, height=900)

    with dpg.window(label='Controls', pos=(10,10), width=400, height=880):
        # Data Collection Controls
        with dpg.group(horizontal=True):
            dpg.add_button(label='Start Collection', callback=lambda s,a: setattr(node,'collecting_data',True) or setattr(node,'start_time',time.time()))
            dpg.add_button(label='Stop Collection', callback=lambda s,a: setattr(node,'collecting_data',False))
            dpg.add_button(label='Save CSV', callback=lambda s,a: save_csv(node))
            dpg.add_button(label='Clear Data', callback=lambda s,a: clear_data(node))
        
        dpg.add_separator()
        
        # Primary Commands Section - Moved to the top for easier access
        dpg.add_text('Commands:', tag='cmd_header')
        
        # Units selector in commands section
        dpg.add_text("Units")
        dpg.add_combo(list(UNIT_OPTIONS.values()), default_value=UNIT_OPTIONS["1"], tag=f"in_{sanitize('/afd/metricUnits')}")
        dpg.add_button(label='Set Units', callback=set_param_cb, user_data=(node,'/afd/metricUnits'))
        
        dpg.add_slider_float(label='Force Cmd', tag='cmd_force', min_value=-150, max_value=150, format='%.1f')
        dpg.add_button(label='Send Force', callback=send_force, user_data=node, tag="send_force_button")
        dpg.add_slider_float(label='Pos Cmd', tag='cmd_pos', min_value=node.position_min, max_value=node.position_max, format='%.1f')
        dpg.add_button(label='Send Pos', callback=send_position, user_data=node, tag="send_pos_button")
        
        # Replace numeric mode with descriptive dropdown - use unique labels in dropdown
        mode_labels = []
        # Get unique mode labels while preserving order
        for value, label in CONTROL_MODES.items():
            if label not in mode_labels:
                mode_labels.append(label)
                
        dpg.add_combo(mode_labels, label='Mode', tag='cmd_mode', default_value=CONTROL_MODES["0"])
        dpg.add_button(label='Send Mode', callback=send_mode, user_data=node)
        
        dpg.add_separator()
        
        # Live Status Display
        dpg.add_text('Status:', tag='status_header')
        for key in ['actual_force','actual_position','command_force','command_position',
                    'payload_weight','accel_gravity','control_mode','tool_connected']:
            dpg.add_text('', tag=f'stat_{key}')
        
        dpg.add_separator()
        
        # All Parameters grouped into collapsible sections
        with dpg.collapsing_header(label='Device Parameters', default_open=False):
            for p in PARAMETERS:
                # Skip metricUnits as it's now in the commands section
                if p['name'] == '/afd/metricUnits':
                    continue
                    
                safe = sanitize(p['name'])
                dpg.add_text(p['desc'])
                if p['read']:
                    with dpg.group(horizontal=True):
                        # Use special callback for parameters that need formatting
                        if p['name'] in ['/afd/controlMode', '/afd/metricUnits']:
                            dpg.add_button(label='Get', callback=get_param_with_format_cb, user_data=(node,p['name']))
                        else:
                            dpg.add_button(label='Get', callback=get_param_cb, user_data=(node,p['name']))
                        dpg.add_text('', tag=f"val_{safe}")
                if p['write']:
                    # Special handling for dropdown parameters
                    if p['name'] == '/afd/controlMode':
                        dpg.add_combo(list(CONTROL_MODES.values()), default_value=CONTROL_MODES["0"], tag=f"in_{safe}")
                    else:
                        dpg.add_input_text(tag=f"in_{safe}", default_value='')
                    dpg.add_button(label='Set', callback=set_param_cb, user_data=(node,p['name']))
                dpg.add_separator()
        
        # Add debug buttons for troubleshooting
        with dpg.collapsing_header(label='Debugging Tools', default_open=False):
            dpg.add_button(label='Check Control Mode Bitmap', callback=lambda s,a,u: get_current_mode_bitmap(u), user_data=node)
            dpg.add_button(label='Set Position Mode (BIT0=0)', callback=lambda s,a,u: set_position_mode(u), user_data=node)
            dpg.add_button(label='Set Force Mode (BIT0=1)', callback=lambda s,a,u: set_force_mode(u), user_data=node)
            dpg.add_button(label='Print Device Status', callback=lambda s,a,u: print_device_status(u), user_data=node)
            
            # Add bitmap editor for full control mode customization
            dpg.add_text("Custom Control Mode Bitmap:")
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="BIT0: Force Mode", default_value=False, tag="bit0_force_mode")
                dpg.add_text("(OFF = Position Mode, ON = Force Mode)")
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="BIT1: SoftTouch", default_value=False, tag="bit1_soft_touch")
            dpg.add_slider_int(label="SoftTouch Position (BIT3-7)", default_value=0, min_value=0, max_value=31, tag="soft_touch_pos")
            dpg.add_slider_int(label="SoftTouch Force (BIT8-12)", default_value=0, min_value=0, max_value=31, tag="soft_touch_force")
            dpg.add_button(label='Send Custom Mode Bitmap', callback=lambda s,a,u: send_custom_mode_bitmap(u), user_data=node)
            
            # Raw value input
            dpg.add_input_text(label='Raw Mode Value', default_value="0", tag="raw_mode_value")
            dpg.add_button(label='Send Raw Mode', callback=lambda s,a,u: send_raw_mode(u), user_data=node)
        
        # Plot Settings
        with dpg.collapsing_header(label='Plot Settings', default_open=False):
            dpg.add_input_text(label='Status Topic', default_value=node.topic_name, tag='setting_topic')
            dpg.add_drag_int(label='Max Points', default_value=node.max_points, min_value=1, max_value=10000, tag='setting_max_pts')
            dpg.add_drag_float(label='Update Rate (Hz)', default_value=node.update_rate, min_value=0.1, max_value=100, format='%.1f', tag='setting_rate')
            dpg.add_drag_float(label='Force Min', default_value=node.force_min, tag='setting_fmin')
            dpg.add_drag_float(label='Force Max', default_value=node.force_max, tag='setting_fmax')
            dpg.add_drag_float(label='Pos Min', default_value=node.position_min, tag='setting_pmin')
            dpg.add_drag_float(label='Pos Max', default_value=node.position_max, tag='setting_pmax')
            dpg.add_drag_float(label='Time Window (s)', default_value=node.time_window, min_value=1.0, max_value=600.0, format='%.1f', tag='setting_time_window')
            dpg.add_checkbox(label='Auto-scroll Plot', default_value=node.auto_scroll, tag='setting_auto_scroll')
            dpg.add_button(label='Apply Settings', callback=lambda s,a: apply_settings(node))

    with dpg.window(label='Plots', pos=(420,10), width=960, height=880):
        with dpg.plot(label='Force vs Time', height=430, width=-1):
            dpg.add_plot_axis(dpg.mvXAxis, label='Time (s)', tag='force_x')
            dpg.add_plot_axis(dpg.mvYAxis, label=UNIT_LABELS["1"]["force"], tag='force_y')  # Default to metric
            dpg.add_line_series([], [], parent='force_y', tag='series_force')
        with dpg.plot(label='Position vs Time', height=430, width=-1):
            dpg.add_plot_axis(dpg.mvXAxis, label='Time (s)', tag='pos_x')
            dpg.add_plot_axis(dpg.mvYAxis, label=UNIT_LABELS["1"]["position"], tag='pos_y')  # Default to metric
            dpg.add_line_series([], [], parent='pos_y', tag='series_pos')

    dpg.setup_dearpygui()
    dpg.set_axis_limits('force_y', node.force_min, node.force_max)
    dpg.set_axis_limits('pos_y', node.position_min, node.position_max)
    dpg.show_viewport()

    # Helper function to format control mode display in the status
    def format_status_display(key, val):
        if key == 'control_mode':
            try:
                int_val = int(val)
                is_force_mode = (int_val & 0x01) > 0  # Check BIT0
                is_soft_touch = (int_val & 0x02) > 0  # Check BIT1
                
                mode_desc = "Force Mode" if is_force_mode else "Position Mode"
                if is_soft_touch:
                    mode_desc += " with SoftTouch"
                
                return f"{key}: {mode_desc} ({val})"
            except:
                pass
            
            # Default display if failed to parse
            if val in CONTROL_MODES:
                return f"{key}: {CONTROL_MODES[val]} ({val})"
        
        return f"{key}: {val}"

    # Update the plot labels based on current units
    def update_plot_labels(node):
        if node.units_changed:
            node.units_changed = False
            # Update Y-axis labels based on current units
            dpg.configure_item('force_y', label=UNIT_LABELS[node.current_units]["force"])
            dpg.configure_item('pos_y', label=UNIT_LABELS[node.current_units]["position"])

    # Update the display with relevant information based on unit and mode
    def update_force_position_display(node):
        with node.lock:
            try:
                # Get current mode
                current_mode = node.status_fields.get('control_mode')
                
                # In case status doesn't have control_mode yet
                if current_mode is None:
                    return
                    
                try:
                    # Check if we should enable force or position controls
                    mode_int = int(current_mode)
                    is_force_mode = (mode_int & 0x01) > 0  # Check BIT0
                    
                    # Enable position controls when in position mode, disable when in force mode
                    try:
                        dpg.configure_item("send_pos_button", enabled=not is_force_mode)
                        dpg.configure_item("cmd_pos", enabled=not is_force_mode)
                    except:
                        pass
                        
                    # Enable force controls when in force mode, disable when in position mode
                    try:
                        dpg.configure_item("send_force_button", enabled=is_force_mode)
                        dpg.configure_item("cmd_force", enabled=is_force_mode)
                    except:
                        pass
                        
                    # Update mode display for visual feedback
                    mode_tag = "stat_control_mode"
                    if is_force_mode:
                        # No need to modify color as DearPyGui doesn't support this directly
                        pass
                except ValueError:
                    # If we can't parse mode as int, just leave controls enabled
                    pass
            except Exception as e:
                print(f"Error updating force/position display: {e}")

    while dpg.is_dearpygui_running():
        with node.lock:
            for key, val in node.status_fields.items():
                dpg.set_value(f'stat_{key}', format_status_display(key, str(val)))
            
            if node.times:
                # Convert values based on current units
                force_display, position_display = convert_values_for_display(node, node.force_data, node.position_data)
                
                # Update the plots with converted data
                dpg.configure_item('series_force', x=node.times, y=force_display)
                dpg.configure_item('series_pos', x=node.times, y=position_display)
                
                # Auto-scroll the x-axis if enabled
                if node.auto_scroll and (node.update_plot_limits or len(node.times) > 1):
                    node.update_plot_limits = False
                    latest_time = node.times[-1]
                    if latest_time > node.time_window:
                        # Set the time axis to show the most recent window of data
                        start_time = max(0, latest_time - node.time_window)
                        dpg.set_axis_limits('force_x', start_time, latest_time)
                        dpg.set_axis_limits('pos_x', start_time, latest_time)
                    else:
                        # For the beginning of data collection, show from 0 to time_window
                        dpg.set_axis_limits('force_x', 0, max(node.time_window, latest_time * 1.1))
                        dpg.set_axis_limits('pos_x', 0, max(node.time_window, latest_time * 1.1))
        
        # Update plot labels if units changed
        update_plot_labels(node)
        
        # Update display elements based on current mode and settings
        update_force_position_display(node)
                
        dpg.render_dearpygui_frame()
        time.sleep(1.0 / node.update_rate)

    dpg.destroy_context()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()