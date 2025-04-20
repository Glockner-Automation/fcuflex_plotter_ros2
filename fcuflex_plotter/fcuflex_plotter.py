#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import threading
import time
import csv
from datetime import datetime
from fcuflex_ros2.msg import AFDStatus

class FCUFLEXPlotter(Node):
    def __init__(self):
        super().__init__('fcuflex_plotter')
        
        # Create the subscription to the status topic
        self.subscription = self.create_subscription(
            AFDStatus,
            '/fcuflex_node/status',
            self.status_callback,
            10)
        
        # Initialize data storage
        self.max_points = 1000  # Store the last 1000 data points
        self.times = []
        self.force_data = []
        self.position_data = []
        self.collecting_data = False
        self.start_time = None
        
        # Thread lock for data access
        self.lock = threading.Lock()
        
        # Create the GUI (will be called from main thread)
        self.root = None
    
    def status_callback(self, msg):
        with self.lock:
            if not self.collecting_data:
                return
                
            # Get current time relative to start
            current_time = time.time() - self.start_time
            
            # Store the data
            self.times.append(current_time)
            self.force_data.append(msg.actual_force)
            self.position_data.append(msg.actual_position)
            
            # Limit data points to max_points
            if len(self.times) > self.max_points:
                self.times.pop(0)
                self.force_data.pop(0)
                self.position_data.pop(0)
        
        # Update GUI elements (need to use after() to run in main thread)
        if self.root is not None:
            self.root.after(0, self.update_gui_values, msg)
    
    def update_gui_values(self, msg):
        """Update GUI elements with new values (runs in main thread)"""
        # Update current value displays
        self.force_value_label.config(text=f"{msg.actual_force:.2f} N")
        self.position_value_label.config(text=f"{msg.actual_position:.2f} mm")
        
        # Calculate statistics if we have data
        with self.lock:
            if len(self.force_data) > 0:
                force_min = min(self.force_data)
                force_max = max(self.force_data)
                force_avg = sum(self.force_data) / len(self.force_data)
                
                position_min = min(self.position_data)
                position_max = max(self.position_data)
                position_avg = sum(self.position_data) / len(self.position_data)
                
                self.force_stats_label.config(
                    text=f"Min: {force_min:.2f} N, Max: {force_max:.2f} N, Avg: {force_avg:.2f} N")
                self.position_stats_label.config(
                    text=f"Min: {position_min:.2f} mm, Max: {position_max:.2f} mm, Avg: {position_avg:.2f} mm")
    
    def create_gui(self):
        # Create the main window
        self.root = tk.Tk()
        self.root.title("FCUFLEX Data Plotter")
        self.root.geometry("1000x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Create a frame for the plot
        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create the matplotlib figure and canvas
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Configure the axes
        self.ax1.set_title('Force vs Time')
        self.ax1.set_ylabel('Force (N)')
        self.ax1.grid(True)
        
        self.ax2.set_title('Position vs Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Position (mm)')
        self.ax2.grid(True)
        
        # Create lines for the plots
        self.force_line, = self.ax1.plot([], [], 'r-', lw=2)
        self.position_line, = self.ax2.plot([], [], 'b-', lw=2)
        
        # Create a frame for controls
        control_frame = ttk.Frame(self.root)
        control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Create control buttons
        self.start_button = ttk.Button(
            control_frame, text="Start Collection", command=self.start_collection)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(
            control_frame, text="Stop Collection", command=self.stop_collection, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        self.save_button = ttk.Button(
            control_frame, text="Save to CSV", command=self.save_to_csv, state=tk.DISABLED)
        self.save_button.pack(side=tk.LEFT, padx=5)
        
        self.clear_button = ttk.Button(
            control_frame, text="Clear Data", command=self.clear_data)
        self.clear_button.pack(side=tk.LEFT, padx=5)
        
        # Create a frame for current values
        values_frame = ttk.LabelFrame(self.root, text="Current Values")
        values_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Force current value
        force_frame = ttk.Frame(values_frame)
        force_frame.pack(fill=tk.X, expand=True, padx=5, pady=5)
        ttk.Label(force_frame, text="Force:").pack(side=tk.LEFT)
        self.force_value_label = ttk.Label(force_frame, text="0.00 N")
        self.force_value_label.pack(side=tk.LEFT, padx=5)
        
        # Position current value
        position_frame = ttk.Frame(values_frame)
        position_frame.pack(fill=tk.X, expand=True, padx=5, pady=5)
        ttk.Label(position_frame, text="Position:").pack(side=tk.LEFT)
        self.position_value_label = ttk.Label(position_frame, text="0.00 mm")
        self.position_value_label.pack(side=tk.LEFT, padx=5)
        
        # Create a frame for statistics
        stats_frame = ttk.LabelFrame(self.root, text="Statistics")
        stats_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Force statistics
        force_stats_frame = ttk.Frame(stats_frame)
        force_stats_frame.pack(fill=tk.X, expand=True, padx=5, pady=5)
        ttk.Label(force_stats_frame, text="Force Stats:").pack(side=tk.LEFT)
        self.force_stats_label = ttk.Label(force_stats_frame, text="Min: 0.00 N, Max: 0.00 N, Avg: 0.00 N")
        self.force_stats_label.pack(side=tk.LEFT, padx=5)
        
        # Position statistics
        position_stats_frame = ttk.Frame(stats_frame)
        position_stats_frame.pack(fill=tk.X, expand=True, padx=5, pady=5)
        ttk.Label(position_stats_frame, text="Position Stats:").pack(side=tk.LEFT)
        self.position_stats_label = ttk.Label(position_stats_frame, text="Min: 0.00 mm, Max: 0.00 mm, Avg: 0.00 mm")
        self.position_stats_label.pack(side=tk.LEFT, padx=5)
        
        # Setup the animation for real-time plotting
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=True)
        
        # Start the Tkinter main loop in the main thread
        # This will block until the window is closed
        self.root.mainloop()
    
    def update_plot(self, frame):
        # Update the plots with current data
        with self.lock:
            if len(self.times) > 0:
                self.force_line.set_data(self.times, self.force_data)
                self.position_line.set_data(self.times, self.position_data)
                
                # Adjust the plot limits
                self.ax1.relim()
                self.ax1.autoscale_view()
                self.ax2.relim()
                self.ax2.autoscale_view()
        
        return self.force_line, self.position_line
    
    def start_collection(self):
        with self.lock:
            self.start_time = time.time()
            self.collecting_data = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.get_logger().info('Started data collection')
    
    def stop_collection(self):
        with self.lock:
            self.collecting_data = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.save_button.config(state=tk.NORMAL)
        self.get_logger().info('Stopped data collection')
    
    def clear_data(self):
        with self.lock:
            self.times = []
            self.force_data = []
            self.position_data = []
        self.force_value_label.config(text="0.00 N")
        self.position_value_label.config(text="0.00 mm")
        self.force_stats_label.config(text="Min: 0.00 N, Max: 0.00 N, Avg: 0.00 N")
        self.position_stats_label.config(text="Min: 0.00 mm, Max: 0.00 mm, Avg: 0.00 mm")
        self.save_button.config(state=tk.DISABLED)
        self.get_logger().info('Cleared data')
    
    def save_to_csv(self):
        with self.lock:
            if len(self.times) == 0:
                self.get_logger().warn('No data to save')
                return
                
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"fcuflex_data_{timestamp}.csv"
            
            # Write data to CSV
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time (s)', 'Force (N)', 'Position (mm)'])
                for i in range(len(self.times)):
                    writer.writerow([self.times[i], self.force_data[i], self.position_data[i]])
        
        self.get_logger().info(f'Saved data to {filename}')
    
    def on_closing(self):
        self.get_logger().info('Shutting down plotter')
        self.root.quit()
        self.root.destroy()
        rclpy.shutdown()

def spin_ros(node):
    """Function to spin ROS node in a separate thread"""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
        pass

def main(args=None):
    rclpy.init(args=args)
    plotter = FCUFLEXPlotter()
    
    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=spin_ros, args=(plotter,))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Create and run the GUI in the main thread
    plotter.create_gui()
    
    # Clean up
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()