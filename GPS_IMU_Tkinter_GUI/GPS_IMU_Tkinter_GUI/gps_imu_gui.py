
import tkinter as tk
from tkinter import messagebox, ttk
import serial
import serial.tools.list_ports
import threading
import csv
import time

fields = [
    "Latitude", "Longitude", "Elevation", "Satellites",
    "AV_X", "AV_Y", "AV_Z",
    "Acc_X", "Acc_Y", "Acc_Z",
    "Mag_X", "Mag_Y", "Mag_Z"
]


class GPSIMUGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GPS/IMU Real-Time Viewer")
        self.geometry("400x600")
        self.configure(bg="#f0f0f0")

        self.ser = None
        self.serial_thread = None
        self.stop_thread = False
        self.csv_file = None
        self.csv_writer = None

        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(
            self, textvariable=self.port_var, state="readonly")
        self.port_menu['values'] = self.detect_ports()
        self.port_menu.set("Select COM Port")
        self.port_menu.pack(pady=10)

        self.start_button = ttk.Button(
            self, text="Start Display", command=self.start_display)
        self.start_button.pack(pady=5)

        self.labels = {}
        for field in fields:
            row = tk.Frame(self, bg="#f0f0f0")
            label = tk.Label(row, width=15, text=field +
                             ":", anchor='w', bg="#f0f0f0")
            value = tk.Label(row, width=20, text="--",
                             anchor='w', bg="#ffffff", relief="sunken")
            row.pack(pady=2, padx=20, fill=tk.X)
            label.pack(side=tk.LEFT)
            value.pack(side=tk.RIGHT)
            self.labels[field] = value

        self.stop_button = ttk.Button(
            self, text="End Display", command=self.stop_display)
        self.stop_button.pack(pady=10)

        self.protocol("WM_DELETE_WINDOW", self.close_app)

    def detect_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def start_display(self):
        port = self.port_var.get()
        if "COM" not in port:
            messagebox.showwarning(
                "Select Port", "Please select a valid COM port.")
            return
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
        except serial.SerialException:
            messagebox.showerror("Serial Error", f"Could not open port {port}")
            return

        # Open CSV log file
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(f"gps_imu_log_{timestamp}.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(fields)

        self.stop_thread = False
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.start()

    def stop_display(self):
        self.stop_thread = True
        if self.serial_thread:
            self.serial_thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.csv_file:
            self.csv_file.close()

    def read_serial(self):
        while not self.stop_thread and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    data = line.split(",")
                    if len(data) == len(fields):
                        for i, field in enumerate(fields):
                            self.labels[field].config(text=data[i])
                        self.csv_writer.writerow(data)
            except Exception as e:
                print("Error reading serial:", e)

    def close_app(self):
        self.stop_display()
        self.destroy()


if __name__ == "__main__":
    app = GPSIMUGUI()
    app.mainloop()
