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

# Simplified control modes and units
CONTROL_MODES = {
    "0": "Position Mode",
    "1": "Force Mode"
}

UNIT_OPTIONS = {
    "0": "Standard (Imperial)",
    "1": "Metric"
}

# Unit-specific axis labels
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
        self.position_max = 1.5
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
    if node.force_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = str(val)
        node.force_client.call_async(req)


def send_position(sender, app_data, user_data):
    node = user_data
    val = dpg.get_value('cmd_pos')
    if node.pos_client.wait_for_service(0.1):
        req = SetValue.Request(); req.value = str(val)
        node.pos_client.call_async(req)


def send_mode(sender, app_data, user_data):
    node = user_data
    mode_label = dpg.get_value('cmd_mode')
    # Convert mode label to value
    mode_value = "0"  # Default to Position Mode
    for value, label in CONTROL_MODES.items():
        if label == mode_label:
            mode_value = value
            break
    
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
        
        # Primary Commands Section
        dpg.add_text('Commands:', tag='cmd_header')
        
        # Units selector in commands section
        dpg.add_text("Units")
        dpg.add_combo(list(UNIT_OPTIONS.values()), default_value=UNIT_OPTIONS["1"], tag=f"in_{sanitize('/afd/metricUnits')}")
        dpg.add_button(label='Set Units', callback=set_param_cb, user_data=(node,'/afd/metricUnits'))
        
        dpg.add_slider_float(label='Force Cmd', tag='cmd_force', min_value=-150, max_value=150, format='%.1f')
        dpg.add_button(label='Send Force', callback=send_force, user_data=node)
        dpg.add_slider_float(label='Pos Cmd', tag='cmd_pos', min_value=node.position_min, max_value=node.position_max, format='%.1f')
        dpg.add_button(label='Send Pos', callback=send_position, user_data=node)
        
        # Mode dropdown
        dpg.add_combo(list(CONTROL_MODES.values()), label='Mode', tag='cmd_mode', default_value=CONTROL_MODES["0"])
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
        if key == 'control_mode' and val in CONTROL_MODES:
            return f"{key}: {CONTROL_MODES[val]} ({val})"
        return f"{key}: {val}"

    # Update the plot labels based on current units
    def update_plot_labels(node):
        if node.units_changed:
            node.units_changed = False
            # Update Y-axis labels based on current units
            dpg.configure_item('force_y', label=UNIT_LABELS[node.current_units]["force"])
            dpg.configure_item('pos_y', label=UNIT_LABELS[node.current_units]["position"])

    while dpg.is_dearpygui_running():
        with node.lock:
            for key, val in node.status_fields.items():
                dpg.set_value(f'stat_{key}', format_status_display(key, str(val)))
            
            if node.times:
                # Update the plots with data
                dpg.configure_item('series_force', x=node.times, y=node.force_data)
                dpg.configure_item('series_pos', x=node.times, y=node.position_data)
                
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
                
        dpg.render_dearpygui_frame()
        time.sleep(1.0 / node.update_rate)

    dpg.destroy_context()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()