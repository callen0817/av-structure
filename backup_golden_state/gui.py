import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog
import threading
import sys
import logging
import queue
import os
from PIL import Image, ImageTk

from virtual_scanner import run_pipeline

# --- Design Configuration ---
COLORS = {
    "bg": "#0A0A0A",
    "sidebar": "#121212",
    "accent": "#00A3FF",
    "accent_hover": "#0086D1",
    "text": "#E0E0E0",
    "text_dim": "#A0A0A0",
    "border": "#252525"
}

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

def get_scaled_size(image_path, target_width=None, target_height=None):
    """Calculates dimensions while preserving aspect ratio."""
    try:
        if not os.path.exists(image_path):
            return (100, 100)
        with Image.open(image_path) as img:
            orig_w, orig_h = img.size
            if target_width and target_height:
                ratio = min(target_width / orig_w, target_height / orig_h)
                return (int(orig_w * ratio), int(orig_h * ratio))
            elif target_width:
                ratio = target_width / orig_w
                return (target_width, int(orig_h * ratio))
            elif target_height:
                ratio = target_height / orig_h
                return (int(orig_w * ratio), target_height)
            return (orig_w, orig_h)
    except Exception:
        return (100, 100)

class TextHandler(logging.Handler):
    def __init__(self, log_queue):
        super().__init__()
        self.log_queue = log_queue

    def emit(self, record):
        msg = self.format(record)
        self.log_queue.put(msg)

class SplashScreen(ctk.CTkToplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Initializing AV Structure...")
        self.geometry("600x400")
        self.overrideredirect(True)
        self.configure(fg_color=COLORS["bg"])
        
        # Center the splash screen
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        x = (screen_width // 2) - (600 // 2)
        y = (screen_height // 2) - (400 // 2)
        self.geometry(f"600x400+{x}+{y}")

        # Logo Integration
        try:
            logo_path = "assets/av_structure_invert.png"
            scaled_size = get_scaled_size(logo_path, target_width=450, target_height=150)
            self.logo_img = ctk.CTkImage(
                light_image=Image.open(logo_path),
                dark_image=Image.open(logo_path),
                size=scaled_size
            )
            self.logo_label = ctk.CTkLabel(self, image=self.logo_img, text="")
            self.logo_label.pack(pady=(110, 20))
        except Exception:
            self.logo_label = ctk.CTkLabel(self, text="AV STRUCTURE", font=ctk.CTkFont(size=30, weight="bold"), text_color=COLORS["accent"])
            self.logo_label.pack(pady=(120, 20))

        self.status_label = ctk.CTkLabel(self, text="Loading Core Modules...", 
                                         font=ctk.CTkFont(size=12),
                                         text_color=COLORS["text_dim"])
        self.status_label.pack(pady=(40, 10))

        self.progress = ctk.CTkProgressBar(self, width=400, height=4, 
                                           fg_color=COLORS["sidebar"], 
                                           progress_color=COLORS["accent"])
        self.progress.set(0)
        self.progress.pack()

        self.attributes("-alpha", 0.0)
        self.fade_in()

    def fade_in(self):
        alpha = self.attributes("-alpha")
        if alpha < 1.0:
            self.attributes("-alpha", alpha + 0.05)
            self.after(20, self.fade_in)

    def update_progress(self, val, status):
        self.progress.set(val)
        self.status_label.configure(text=status)
        self.update()

class VirtualScannerGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("AV Structure")
        self.geometry("1100x750")
        self.overrideredirect(True) # Frameless
        self.configure(fg_color=COLORS["bg"])

        # Dragging logic
        self.bind("<ButtonPress-1>", self.start_move)
        self.bind("<ButtonRelease-1>", self.stop_move)
        self.bind("<B1-Motion>", self.do_move)

        # Variables
        self.input_file = ctk.StringVar(value="")
        self.output_file = ctk.StringVar(value="")
        self.pose_csv = ctk.StringVar(value="")
        self.panos_dir = ctk.StringVar(value="")
        self.alignment_file = ctk.StringVar(value="")
        self.mode_var = ctk.StringVar(value="auto")
        self.spacing_var = ctk.DoubleVar(value=2.0)
        self.coverage_var = ctk.DoubleVar(value=0.99)
        self.overlap_var = ctk.DoubleVar(value=0.40)

        self.withdraw() # Hide until splash finishes
        self.show_splash()

    def show_splash(self):
        self.splash = SplashScreen(self)
        self.after(500, lambda: self.splash.update_progress(0.3, "Optimizing Shader Pipeline..."))
        self.after(1500, lambda: self.splash.update_progress(0.7, "Loading Spatial Engine..."))
        self.after(2500, self.finish_splash)

    def finish_splash(self):
        self.splash.update_progress(1.0, "Ready")
        self.after(500, self.splash.destroy)
        self.after(600, self.deiconify)
        self.after(600, self.create_layout)
        self.after(700, self.setup_logging_and_queue)

    def start_move(self, event):
        self.x = event.x
        self.y = event.y

    def stop_move(self, event):
        self.x = None
        self.y = None

    def do_move(self, event):
        deltax = event.x - self.x
        deltay = event.y - self.y
        x = self.winfo_x() + deltax
        y = self.winfo_y() + deltay
        self.geometry(f"+{x}+{y}")

    def create_layout(self):
        # Sidebar
        self.sidebar = ctk.CTkFrame(self, width=200, corner_radius=0, fg_color=COLORS["sidebar"], border_width=1, border_color=COLORS["border"])
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)

        # Sidebar Logo (Product)
        try:
            logo_path = "assets/av_structure_invert.png"
            scaled_size = get_scaled_size(logo_path, target_width=160, target_height=60)
            self.side_logo = ctk.CTkImage(Image.open(logo_path), size=scaled_size)
            ctk.CTkLabel(self.sidebar, image=self.side_logo, text="").pack(pady=(40, 20))
        except Exception:
            ctk.CTkLabel(self.sidebar, text="AV STRUCTURE", font=ctk.CTkFont(size=20, weight="bold"), text_color=COLORS["accent"]).pack(pady=(40, 20))

        # Company Logo (Bottom Center)
        try:
            logo_path = "assets/artificialvision_white.png"
            scaled_size = get_scaled_size(logo_path, target_width=140, target_height=40)
            self.comp_logo = ctk.CTkImage(Image.open(logo_path), size=scaled_size)
            self.comp_label = ctk.CTkLabel(self.sidebar, image=self.comp_logo, text="")
            # Center the logo at the bottom
            self.comp_label.pack(side="bottom", pady=30, padx=10)
        except Exception:
            ctk.CTkLabel(self.sidebar, text="ARTIFICIAL VISION", font=ctk.CTkFont(size=12, weight="bold"), text_color=COLORS["text_dim"]).pack(side="bottom", pady=30)

        # Main Content
        self.content = ctk.CTkFrame(self, fg_color="transparent")
        self.content.pack(side="right", fill="both", expand=True)

        # Custom Title Bar
        self.title_bar = ctk.CTkFrame(self.content, height=40, fg_color="transparent")
        self.title_bar.pack(fill="x", padx=20, pady=10)
        
        ctk.CTkLabel(self.title_bar, text="AV STRUCTURE | UNIVERSAL SCAN CONVERTER", 
                     font=ctk.CTkFont(size=12, weight="bold"), text_color=COLORS["text_dim"]).pack(side="left")
        
        ctk.CTkButton(self.title_bar, text="✕", width=30, height=30, fg_color="transparent", 
                      hover_color="#FF4B4B", command=self.quit).pack(side="right")
        ctk.CTkButton(self.title_bar, text="—", width=30, height=30, fg_color="transparent", 
                      hover_color=COLORS["border"], command=self.withdraw).pack(side="right", padx=5)

        # Content Area - Scrollable
        self.scroll_frame = ctk.CTkScrollableFrame(self.content, fg_color="transparent")
        self.scroll_frame.pack(fill="both", expand=True, padx=20, pady=(0, 20))

        self.create_widgets()

    def setup_logging_and_queue(self):
        self.log_queue = queue.Queue()
        self.logger = logging.getLogger("gui_logger")
        self.logger.setLevel(logging.INFO)
        formatter = logging.Formatter("%(asctime)s - %(message)s", datefmt="%H:%M:%S")
        text_handler = TextHandler(self.log_queue)
        text_handler.setFormatter(formatter)
        self.logger.addHandler(text_handler)
        
        for module in ["io_modules", "stations", "simulation", "virtual_scanner"]:
            l = logging.getLogger(module)
            l.addHandler(text_handler)
            l.setLevel(logging.INFO)
        
        self.check_queue()

    def create_widgets(self):
        # Section: Input Data
        self.create_section_header("INPUT DATA SOURCES")
        
        input_grid = ctk.CTkFrame(self.scroll_frame, fg_color="transparent")
        input_grid.pack(fill="x", pady=(0, 10))
        input_grid.grid_columnconfigure(1, weight=1)

        self.create_input_row(input_grid, 0, "Unified Cloud (E57/LAS)", self.input_file, self.browse_input)
        self.create_input_row(input_grid, 1, "Poses CSV (Optional)", self.pose_csv, self.browse_pose)
        self.create_input_row(input_grid, 2, "Panoramas Directory", self.panos_dir, self.browse_panos)
        self.create_input_row(input_grid, 3, "Alignment Bundle", self.alignment_file, self.browse_alignment)

        # Section: Output Configuration
        self.create_section_header("OUTPUT CONFIGURATION")
        output_grid = ctk.CTkFrame(self.scroll_frame, fg_color="transparent")
        output_grid.pack(fill="x", pady=(0, 10))
        output_grid.grid_columnconfigure(1, weight=1)
        self.create_input_row(output_grid, 0, "Destination Path", self.output_file, self.browse_output)

        # Section: Generation Settings
        self.create_section_header("SCAN GENERATION SETTINGS")
        settings_frame = ctk.CTkFrame(self.scroll_frame, fg_color=COLORS["sidebar"], border_width=1, border_color=COLORS["border"])
        settings_frame.pack(fill="x", pady=(0, 20), ipady=10)

        # Mode Selection
        ctk.CTkLabel(settings_frame, text="Generation Mode", font=ctk.CTkFont(weight="bold")).pack(anchor="w", padx=20, pady=(10, 5))
        mode_box = ctk.CTkFrame(settings_frame, fg_color="transparent")
        mode_box.pack(fill="x", padx=20)
        ctk.CTkRadioButton(mode_box, text="Auto-Detection", variable=self.mode_var, value="auto", fg_color=COLORS["accent"]).pack(side="left", padx=(0, 20))
        ctk.CTkRadioButton(mode_box, text="Trajectory Driven", variable=self.mode_var, value="trajectory", fg_color=COLORS["accent"]).pack(side="left", padx=20)
        ctk.CTkRadioButton(mode_box, text="Geometry Focused", variable=self.mode_var, value="geometry", fg_color=COLORS["accent"]).pack(side="left", padx=20)

        # Sliders
        self.create_slider_row(settings_frame, "Station Spacing (m)", self.spacing_var, 0.5, 10.0)
        self.create_slider_row(settings_frame, "Target Coverage (%)", self.coverage_var, 0.5, 1.0)

        # Console Output
        self.create_section_header("PROCESS LOGS")
        self.textbox = ctk.CTkTextbox(self.scroll_frame, height=280, fg_color="#050505", border_width=1, border_color=COLORS["border"], 
                                      font=ctk.CTkFont(family="Consolas", size=12), text_color="#00FF41")
        self.textbox.pack(fill="x", pady=(0, 20))
        self.textbox.configure(state="disabled")

        # Actions
        actions_frame = ctk.CTkFrame(self.scroll_frame, fg_color="transparent")
        actions_frame.pack(fill="x")
        
        self.start_btn = ctk.CTkButton(actions_frame, text="INITIALIZE PIPELINE", 
                                       font=ctk.CTkFont(size=14, weight="bold"),
                                       height=50, width=300, fg_color=COLORS["accent"], 
                                       hover_color=COLORS["accent_hover"],
                                       command=self.start_pipeline)
        self.start_btn.pack(side="left")
        
        ctk.CTkButton(actions_frame, text="CLEAR LOGS", height=50, width=150, 
                      fg_color="transparent", border_width=1, border_color=COLORS["border"],
                      command=self.clear_log).pack(side="left", padx=20)

    def create_section_header(self, text):
        h = ctk.CTkFrame(self.scroll_frame, fg_color="transparent")
        h.pack(fill="x", pady=(10, 10))
        ctk.CTkLabel(h, text=text, font=ctk.CTkFont(size=12, weight="bold"), text_color=COLORS["accent"]).pack(side="left")
        ctk.CTkFrame(h, height=1, fg_color=COLORS["border"]).pack(side="left", fill="x", expand=True, padx=(10, 0))

    def create_input_row(self, parent, row, label, var, command):
        ctk.CTkLabel(parent, text=label, text_color=COLORS["text_dim"], font=ctk.CTkFont(size=11)).grid(row=row, column=0, sticky="w", pady=5)
        entry = ctk.CTkEntry(parent, textvariable=var, height=35, fg_color=COLORS["sidebar"], border_color=COLORS["border"])
        entry.grid(row=row, column=1, sticky="ew", padx=15, pady=5)
        ctk.CTkButton(parent, text="BROWSE", width=80, height=35, fg_color=COLORS["border"], hover_color="#333", command=command).grid(row=row, column=2, pady=5)

    def create_slider_row(self, parent, label, var, min_v, max_v):
        f = ctk.CTkFrame(parent, fg_color="transparent")
        f.pack(fill="x", padx=20, pady=5)
        ctk.CTkLabel(f, text=label, width=150, anchor="w").pack(side="left")
        ctk.CTkSlider(f, from_=min_v, to=max_v, variable=var, fg_color=COLORS["border"], progress_color=COLORS["accent"]).pack(side="left", fill="x", expand=True, padx=10)
        ctk.CTkLabel(f, textvariable=var, width=40).pack(side="left")

    def browse_input(self):
        file = filedialog.askopenfilename(title="Select Unified Point Cloud", filetypes=(("Point Cloud files", "*.las;*.laz;*.e57"), ("All files", "*.*")))
        if file: self.input_file.set(file)

    def browse_output(self):
        file = filedialog.asksaveasfilename(title="Save Structured Output", defaultextension=".e57", filetypes=(("E57 files", "*.e57"),))
        if file: self.output_file.set(file)

    def browse_pose(self):
        file = filedialog.askopenfilename(title="Select Pose CSV", filetypes=(("CSV files", "*.csv"), ("All files", "*.*")))
        if file: self.pose_csv.set(file)

    def browse_panos(self):
        d = filedialog.askdirectory(title="Select Panoramas Directory")
        if d: self.panos_dir.set(d)

    def browse_alignment(self):
        file = filedialog.askopenfilename(title="Select Alignment Bundle", filetypes=(("NavVis Bundle", "*.bundle"), ("JSON/XML", "*.json;*.xml"), ("All files", "*.*")))
        if file: self.alignment_file.set(file)

    def append_log(self, text):
        self.textbox.configure(state="normal")
        self.textbox.insert("end", text + "\n")
        self.textbox.see("end")
        self.textbox.configure(state="disabled")

    def clear_log(self):
        self.textbox.configure(state="normal")
        self.textbox.delete("0.0", "end")
        self.textbox.configure(state="disabled")

    def check_queue(self):
        while not self.log_queue.empty():
            try:
                msg = self.log_queue.get_nowait()
                self.append_log(msg)
            except queue.Empty:
                break
        self.after(100, self.check_queue)

    def start_pipeline(self):
        if not self.input_file.get() or not self.output_file.get():
            self.append_log("[ERROR] Missing required input/output paths.")
            return
        self.start_btn.configure(state="disabled", text="PIPELINE ACTIVE")
        threading.Thread(target=self.run_pipeline_thread, daemon=True).start()

    def run_pipeline_thread(self):
        try:
            run_pipeline(
                input_cloud_path=self.input_file.get(),
                output_e57_path=self.output_file.get(),
                pose_csv_path=self.pose_csv.get() or None,
                navvis_images_dir=self.panos_dir.get() or None,
                mode_pref=self.mode_var.get(),
                logger_instance=self.logger,
                alignment_file=self.alignment_file.get() or None,
                spacing=self.spacing_var.get(),
                overlap=self.overlap_var.get(),
                coverage=self.coverage_var.get()
            )
            self.logger.info("PIPELINE COMPLETED SUCCESSFULLY.")
        except Exception as e:
            self.logger.error(f"PIPELINE FAILURE: {str(e)}")
        finally:
            self.after(0, lambda: self.start_btn.configure(state="normal", text="INITIALIZE PIPELINE"))

if __name__ == "__main__":
    app = VirtualScannerGUI()
    app.mainloop()
