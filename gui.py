import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog
import threading
import sys
import logging
import queue
import os
import json
from datetime import datetime
from PIL import Image, ImageTk

from virtual_scanner import run_pipeline

# --- Design Configuration (Aligned with https://prevdc.artificialviz.com/) ---
# Uses (Light, Dark) tuples matching the Artificial Vision OKLCH design system:
#   --primary:            oklch(71.45% 0.0838 87.10) -> #BDA469 (Warm Brass/Gold)
#   --primary-foreground: oklch(21% 0 0)             -> #181818 (Deep Charcoal)
#   --background:         oklch(98.5% 0 0) / 16%     -> #FAFAFA / #0E0E0E
#   --card / .panel:      oklch(100% 0 0)  / 20.8%   -> #FFFFFF / #171717
#   --secondary / --muted:oklch(95.5% 0 0) / 27.9%   -> #F0F0F0 / #262626
#   --muted-foreground:   oklch(48% 0 0)   / 70.4%   -> #5E5E5E / #A0A0A0
#   --accent:             oklch(94% .025 87)/ 30%    -> #F2EDE2 / #2C271D
#   --border / --input:   oklch(90% 0 0)   / 15%     -> #E0E0E0 / #292929
COLORS = {
    "bg": ("#FAFAFA", "#0E0E0E"),
    "sidebar": ("#FFFFFF", "#141414"),
    "card": ("#FFFFFF", "#171717"),
    "secondary": ("#F0F0F0", "#262626"),
    "accent": ("#BDA469", "#BDA469"),
    "accent_hover": ("#A68D52", "#A68D52"),
    "accent_surface": ("#F2EDE2", "#2C271D"),
    "primary_fg": ("#181818", "#181818"),
    "text": ("#181818", "#FAFAFA"),
    "text_dim": ("#5E5E5E", "#A0A0A0"),
    "border": ("#E0E0E0", "#292929"),
    "console_bg": ("#F5F4F0", "#0A0A0A"),
    "console_text": ("#2B261B", "#E6D5A7"),
    "destructive": ("#D93829", "#E04F42")
}

ctk.set_appearance_mode("Light")
ctk.set_default_color_theme("blue")

FONT_SANS = "Segoe UI"
FONT_MONO = "Consolas"

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

def load_brand_logo(target_width=None, target_height=None):
    """Loads the light/dark adaptive AV Structure product logo."""
    light_path = "assets/av_structure_logo.png"
    dark_path = "assets/av_structure_logo_dark.png"
    if not os.path.exists(light_path):
        light_path = "assets/av_structure_invert.png"
    if not os.path.exists(dark_path):
        dark_path = light_path
    scaled_size = get_scaled_size(light_path, target_width=target_width, target_height=target_height)
    return ctk.CTkImage(
        light_image=Image.open(light_path),
        dark_image=Image.open(dark_path),
        size=scaled_size
    )

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

        card = ctk.CTkFrame(
            self,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=8
        )
        card.pack(fill="both", expand=True, padx=16, pady=16)

        # Logo Integration
        try:
            self.logo_img = load_brand_logo(target_width=130, target_height=130)
            self.logo_label = ctk.CTkLabel(card, image=self.logo_img, text="")
            self.logo_label.pack(pady=(48, 12))
        except Exception:
            self.logo_label = ctk.CTkLabel(
                card,
                text="AV Structure",
                font=ctk.CTkFont(family=FONT_SANS, size=28, weight="bold"),
                text_color=COLORS["text"]
            )
            self.logo_label.pack(pady=(65, 12))

        ctk.CTkLabel(
            card,
            text="AV Structure",
            font=ctk.CTkFont(family=FONT_SANS, size=22, weight="bold"),
            text_color=COLORS["text"]
        ).pack(pady=(0, 2))

        ctk.CTkLabel(
            card,
            text="an Artificial Vision product",
            font=ctk.CTkFont(family=FONT_SANS, size=12),
            text_color=COLORS["text_dim"]
        ).pack(pady=(0, 24))

        self.status_label = ctk.CTkLabel(
            card,
            text="LOADING CORE MODULES...",
            font=ctk.CTkFont(family=FONT_MONO, size=11),
            text_color=COLORS["text_dim"]
        )
        self.status_label.pack(pady=(10, 10))

        self.progress = ctk.CTkProgressBar(
            card,
            width=380,
            height=5,
            corner_radius=3,
            fg_color=COLORS["secondary"],
            progress_color=COLORS["accent"]
        )
        self.progress.set(0)
        self.progress.pack()

        self.attributes("-alpha", 0.0)
        self.fade_in()

    def fade_in(self):
        if not self.winfo_exists():
            return
        alpha = self.attributes("-alpha")
        if alpha < 1.0:
            self.attributes("-alpha", alpha + 0.05)
            self.after(20, self.fade_in)

    def update_progress(self, val, status):
        if not self.winfo_exists():
            return
        self.progress.set(val)
        self.status_label.configure(text=status.upper())
        self.update()

class VirtualScannerGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("AV Structure")
        self.geometry("1120x760")
        self.minsize(1050, 700)
        self.configure(fg_color=COLORS["bg"])

        # Center the main window on screen
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        x = max(0, (screen_width // 2) - (1120 // 2))
        y = max(0, (screen_height // 2) - (760 // 2))
        self.geometry(f"1120x760+{x}+{y}")

        # Variables
        self.project_dir = ctk.StringVar(value="")
        self.input_file = ctk.StringVar(value="")
        self.output_file = ctk.StringVar(value="")
        self.pose_csv = ctk.StringVar(value="")
        self.panos_dir = ctk.StringVar(value="")
        self.alignment_file = ctk.StringVar(value="")
        self.mode_var = ctk.StringVar(value="auto")
        self.spacing_var = ctk.DoubleVar(value=6.0)
        self.coverage_var = ctk.DoubleVar(value=0.99)
        self.overlap_var = ctk.DoubleVar(value=0.40)
        self.max_range_var = ctk.DoubleVar(value=50.0)
        self.dry_run_var = ctk.BooleanVar(value=False)
        self.import_workflow_var = ctk.StringVar(value="Auto-Detect Workflow")
        self.scan_density_var = ctk.StringVar(value="100% (1:1 All Panoramas - Default)")
        self.input_rows = {}
        self.x = None
        self.y = None

        self.create_layout()
        self.setup_logging_and_queue()

        # Bring window to front
        self.lift()
        self.focus_force()
        self.attributes("-topmost", True)
        self.after(300, lambda: self.attributes("-topmost", False))

    def start_move(self, event):
        # Initialize drag offset
        self.x = event.x
        self.y = event.y

    def stop_move(self, event):
        # Reset offsets when mouse released
        self.x = None
        self.y = None

    def do_move(self, event):
        # Only move if a drag has been started
        if self.x is None or self.y is None:
            return
        deltax = event.x - self.x
        deltay = event.y - self.y
        x = self.winfo_x() + deltax
        y = self.winfo_y() + deltay
        self.geometry(f"+{x}+{y}")

    def toggle_theme(self):
        current = ctk.get_appearance_mode()
        next_mode = "Dark" if current == "Light" else "Light"
        ctk.set_appearance_mode(next_mode)
        if hasattr(self, "theme_btn"):
            self.theme_btn.configure(text="☾" if next_mode == "Light" else "☀")

    def create_layout(self):
        # Sidebar (matches Artificial Vision surface & border layout)
        self.sidebar = ctk.CTkFrame(
            self,
            width=235,
            corner_radius=0,
            fg_color=COLORS["sidebar"],
            border_width=1,
            border_color=COLORS["border"]
        )
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)

        # Top Brand Accent Bar
        ctk.CTkFrame(self.sidebar, height=4, corner_radius=0, fg_color=COLORS["accent"]).pack(fill="x", side="top")

        # Sidebar Product Identity Lockup (matches brand-identity component on prevdc.artificialviz.com)
        brand_frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        brand_frame.pack(fill="x", padx=20, pady=(28, 18))

        try:
            self.side_logo = load_brand_logo(target_width=92, target_height=92)
            ctk.CTkLabel(brand_frame, image=self.side_logo, text="").pack(anchor="center", pady=(0, 12))
        except Exception:
            pass

        ctk.CTkLabel(
            brand_frame,
            text="AV Structure",
            font=ctk.CTkFont(family=FONT_SANS, size=20, weight="bold"),
            text_color=COLORS["text"]
        ).pack(anchor="center")

        ctk.CTkLabel(
            brand_frame,
            text="an Artificial Vision product",
            font=ctk.CTkFont(family=FONT_SANS, size=11),
            text_color=COLORS["text_dim"]
        ).pack(anchor="center", pady=(2, 0))

        ctk.CTkFrame(self.sidebar, height=1, fg_color=COLORS["border"]).pack(fill="x", padx=20, pady=12)

        # Sidebar Architectural Context Card
        info_card = ctk.CTkFrame(
            self.sidebar,
            fg_color=COLORS["accent_surface"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=6
        )
        info_card.pack(fill="x", padx=16, pady=8)

        ctk.CTkLabel(
            info_card,
            text="SPATIAL INTELLIGENCE",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            text_color=COLORS["text_dim"]
        ).pack(anchor="w", padx=12, pady=(10, 4))

        ctk.CTkLabel(
            info_card,
            text="Converts unstructured SLAM & E57 captures into visibility-verified, multi-level structured scan stations with 360° panospheres.",
            font=ctk.CTkFont(family=FONT_SANS, size=11),
            text_color=COLORS["text"],
            justify="left",
            wraplength=175
        ).pack(anchor="w", padx=12, pady=(0, 10))

        # Company Logo (Bottom Center)
        try:
            light_wm = "assets/artificialvision_dark.png" if os.path.exists("assets/artificialvision_dark.png") else "assets/artificialvision_white.png"
            dark_wm = "assets/artificialvision_white.png"
            scaled_size = get_scaled_size(dark_wm, target_width=145, target_height=36)
            self.comp_logo = ctk.CTkImage(
                light_image=Image.open(light_wm),
                dark_image=Image.open(dark_wm),
                size=scaled_size
            )
            self.comp_label = ctk.CTkLabel(self.sidebar, image=self.comp_logo, text="")
            self.comp_label.pack(side="bottom", pady=(6, 24), padx=10)
        except Exception:
            ctk.CTkLabel(
                self.sidebar,
                text="ARTIFICIAL VISION",
                font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
                text_color=COLORS["text_dim"]
            ).pack(side="bottom", pady=24)

        ctk.CTkLabel(
            self.sidebar,
            text="VERIFY. STRUCTURE. CLARIFY.",
            font=ctk.CTkFont(family=FONT_MONO, size=9),
            text_color=COLORS["text_dim"]
        ).pack(side="bottom", pady=(0, 4))

        # Main Content
        self.content = ctk.CTkFrame(self, fg_color="transparent")
        self.content.pack(side="right", fill="both", expand=True)

        # Custom Header / Title Bar (matches sticky header on prevdc.artificialviz.com)
        self.title_bar = ctk.CTkFrame(
            self.content,
            height=48,
            corner_radius=0,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"]
        )
        self.title_bar.pack(fill="x", padx=0, pady=(0, 8))

        # Dragging logic restricted strictly to title bar
        self.title_bar.bind("<ButtonPress-1>", self.start_move)
        self.title_bar.bind("<ButtonRelease-1>", self.stop_move)
        self.title_bar.bind("<B1-Motion>", self.do_move)
        
        title_label = ctk.CTkLabel(
            self.title_bar,
            text="AV STRUCTURE  ·  UNIVERSAL SCAN CONVERTER",
            font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
            text_color=COLORS["text_dim"]
        )
        title_label.pack(side="left", padx=24, pady=12)
        title_label.bind("<ButtonPress-1>", self.start_move)
        title_label.bind("<ButtonRelease-1>", self.stop_move)
        title_label.bind("<B1-Motion>", self.do_move)
        
        ctk.CTkButton(
            self.title_bar,
            text="✕",
            width=32,
            height=30,
            corner_radius=6,
            fg_color="transparent",
            text_color=COLORS["text"],
            hover_color=COLORS["destructive"],
            command=self.quit
        ).pack(side="right", padx=(4, 16), pady=8)

        ctk.CTkButton(
            self.title_bar,
            text="—",
            width=32,
            height=30,
            corner_radius=6,
            fg_color="transparent",
            text_color=COLORS["text"],
            hover_color=COLORS["secondary"],
            command=self.iconify
        ).pack(side="right", padx=2, pady=8)

        self.theme_btn = ctk.CTkButton(
            self.title_bar,
            text="☾" if ctk.get_appearance_mode() == "Light" else "☀",
            width=32,
            height=30,
            corner_radius=6,
            fg_color="transparent",
            text_color=COLORS["text_dim"],
            hover_color=COLORS["secondary"],
            command=self.toggle_theme
        )
        self.theme_btn.pack(side="right", padx=4, pady=8)

        # Bottom Actions Bar (Pinned)
        actions_frame = ctk.CTkFrame(
            self.content,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=0
        )
        actions_frame.pack(side="bottom", fill="x", padx=0, pady=0)

        actions_inner = ctk.CTkFrame(actions_frame, fg_color="transparent")
        actions_inner.pack(fill="x", padx=24, pady=14)

        self.start_btn = ctk.CTkButton(
            actions_inner,
            text="INITIALIZE PIPELINE",
            font=ctk.CTkFont(family=FONT_SANS, size=13, weight="bold"),
            height=42,
            width=260,
            corner_radius=6,
            fg_color=COLORS["accent"],
            hover_color=COLORS["accent_hover"],
            text_color=COLORS["primary_fg"],
            command=self.start_pipeline
        )
        self.start_btn.pack(side="left")

        ctk.CTkButton(
            actions_inner,
            text="CLEAR LOGS",
            font=ctk.CTkFont(family=FONT_SANS, size=12, weight="bold"),
            height=42,
            width=140,
            corner_radius=6,
            fg_color=COLORS["card"],
            hover_color=COLORS["accent_surface"],
            text_color=COLORS["text"],
            border_width=1,
            border_color=COLORS["border"],
            command=self.clear_log
        ).pack(side="left", padx=14)

        # Content Area - Scrollable
        self.scroll_frame = ctk.CTkScrollableFrame(self.content, fg_color="transparent")
        self.scroll_frame.pack(side="top", fill="both", expand=True, padx=20, pady=(0, 8))

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

        input_panel = ctk.CTkFrame(
            self.scroll_frame,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=6
        )
        input_panel.pack(fill="x", pady=(0, 14))

        # Workflow Dropdown
        wf_frame = ctk.CTkFrame(input_panel, fg_color="transparent")
        wf_frame.pack(fill="x", padx=16, pady=(14, 8))
        ctk.CTkLabel(
            wf_frame,
            text="Import Workflow Mode",
            font=ctk.CTkFont(family=FONT_SANS, size=12, weight="bold"),
            text_color=COLORS["text"]
        ).pack(side="left", padx=(0, 15))

        self.workflow_options = [
            "Auto-Detect Workflow",
            "Structured E57 (Embedded Panoramas + Poses)",
            "E57 + Bundle / Reconstruction File",
            "E57 + Panoramas Directory + Pose CSV"
        ]
        self.workflow_dropdown = ctk.CTkOptionMenu(
            wf_frame,
            variable=self.import_workflow_var,
            values=self.workflow_options,
            fg_color=COLORS["secondary"],
            button_color=COLORS["accent"],
            button_hover_color=COLORS["accent_hover"],
            dropdown_fg_color=COLORS["card"],
            dropdown_hover_color=COLORS["accent_surface"],
            dropdown_text_color=COLORS["text"],
            text_color=COLORS["text"],
            corner_radius=6,
            width=360,
            command=self.on_workflow_change
        )
        self.workflow_dropdown.pack(side="left")
        
        input_grid = ctk.CTkFrame(input_panel, fg_color="transparent")
        input_grid.pack(fill="x", padx=16, pady=(4, 14))
        input_grid.grid_columnconfigure(1, weight=1)

        self.input_rows["project"] = self.create_input_row(input_grid, 0, "Project Directory", self.project_dir, self.browse_project)
        self.input_rows["cloud"] = self.create_input_row(input_grid, 1, "Unified Cloud (E57/LAS)", self.input_file, self.browse_input)
        self.input_rows["pose"] = self.create_input_row(input_grid, 2, "Poses CSV", self.pose_csv, self.browse_pose)
        self.input_rows["panos"] = self.create_input_row(input_grid, 3, "Panoramas Directory", self.panos_dir, self.browse_panos)
        self.input_rows["alignment"] = self.create_input_row(input_grid, 4, "Alignment Bundle", self.alignment_file, self.browse_alignment)

        # Initial visibility based on default option
        self.on_workflow_change()

        # Section: Output Configuration
        self.create_section_header("OUTPUT CONFIGURATION")
        output_panel = ctk.CTkFrame(
            self.scroll_frame,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=6
        )
        output_panel.pack(fill="x", pady=(0, 14))

        output_grid = ctk.CTkFrame(output_panel, fg_color="transparent")
        output_grid.pack(fill="x", padx=16, pady=12)
        output_grid.grid_columnconfigure(1, weight=1)
        self.create_input_row(output_grid, 0, "Destination Path", self.output_file, self.browse_output)

        # Section: Generation Settings
        self.create_section_header("SCAN GENERATION SETTINGS")
        settings_frame = ctk.CTkFrame(
            self.scroll_frame,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=6
        )
        settings_frame.pack(fill="x", pady=(0, 16), ipady=10)

        # Mode Selection
        ctk.CTkLabel(
            settings_frame,
            text="Generation Mode",
            font=ctk.CTkFont(family=FONT_SANS, size=12, weight="bold"),
            text_color=COLORS["text"]
        ).pack(anchor="w", padx=20, pady=(12, 6))
        mode_box = ctk.CTkFrame(settings_frame, fg_color="transparent")
        mode_box.pack(fill="x", padx=20, pady=(0, 6))
        ctk.CTkRadioButton(
            mode_box, text="Auto-Detection", variable=self.mode_var, value="auto",
            fg_color=COLORS["accent"], hover_color=COLORS["accent_hover"], text_color=COLORS["text"]
        ).pack(side="left", padx=(0, 20))
        ctk.CTkRadioButton(
            mode_box, text="Trajectory Driven", variable=self.mode_var, value="trajectory",
            fg_color=COLORS["accent"], hover_color=COLORS["accent_hover"], text_color=COLORS["text"]
        ).pack(side="left", padx=20)
        ctk.CTkRadioButton(
            mode_box, text="Geometry Focused", variable=self.mode_var, value="geometry",
            fg_color=COLORS["accent"], hover_color=COLORS["accent_hover"], text_color=COLORS["text"]
        ).pack(side="left", padx=20)
        ctk.CTkCheckBox(
            mode_box, text="Dry Run Mode", variable=self.dry_run_var,
            fg_color=COLORS["accent"], hover_color=COLORS["accent_hover"],
            checkmark_color=COLORS["primary_fg"], text_color=COLORS["text"]
        ).pack(side="right", padx=(0, 20))

        # Sliders
        # Station Spacing: 0.5m intervals (1.0m to 15.0m)
        self.create_multiplier_slider(settings_frame, "Station Spacing (m)", self.spacing_var, 2, 30, step=1, multiplier=0.5)
        # Max Scan Range: 5m intervals
        self.create_multiplier_slider(settings_frame, "Max Scan Range (m)", self.max_range_var, 1, 20, step=1, multiplier=5)

        # Scan Density (% of Panoramas) Dropdown
        density_box = ctk.CTkFrame(settings_frame, fg_color="transparent")
        density_box.pack(fill="x", padx=20, pady=(6, 4))
        ctk.CTkLabel(
            density_box,
            text="Scan Density (% of Panoramas)",
            font=ctk.CTkFont(family=FONT_SANS, size=12, weight="bold"),
            text_color=COLORS["text"],
            width=220,
            anchor="w"
        ).pack(side="left")
        
        self.density_options = [
            "100% (1:1 All Panoramas - Default)",
            "90%",
            "80%",
            "70%",
            "60%",
            "50%",
            "40%",
            "30%",
            "20%",
            "10%"
        ]
        self.density_dropdown = ctk.CTkOptionMenu(
            density_box,
            variable=self.scan_density_var,
            values=self.density_options,
            fg_color=COLORS["secondary"],
            button_color=COLORS["accent"],
            button_hover_color=COLORS["accent_hover"],
            dropdown_fg_color=COLORS["card"],
            dropdown_hover_color=COLORS["accent_surface"],
            dropdown_text_color=COLORS["text"],
            text_color=COLORS["text"],
            corner_radius=6,
            width=280
        )
        self.density_dropdown.pack(side="left", padx=10)

        # Console Output
        self.create_section_header("PROCESS LOGS")
        self.textbox = ctk.CTkTextbox(
            self.scroll_frame,
            height=240,
            corner_radius=6,
            fg_color=COLORS["console_bg"],
            border_width=1,
            border_color=COLORS["border"],
            font=ctk.CTkFont(family=FONT_MONO, size=12),
            text_color=COLORS["console_text"]
        )
        self.textbox.pack(fill="x", pady=(0, 16))
        self.textbox.configure(state="disabled")

    def on_workflow_change(self, selected_workflow=None):
        wf = self.import_workflow_var.get()
        show_pose = True
        show_panos = True
        show_alignment = True

        if wf == "Structured E57 (Embedded Panoramas + Poses)":
            show_pose = False
            show_panos = False
            show_alignment = True
        elif wf == "E57 + Bundle / Reconstruction File":
            show_pose = False
            show_panos = False
            show_alignment = True
        elif wf == "E57 + Panoramas Directory + Pose CSV":
            show_pose = True
            show_panos = True
            show_alignment = False
        elif wf == "Auto-Detect Workflow":
            show_pose = True
            show_panos = True
            show_alignment = True

        self.set_row_visible("pose", show_pose)
        self.set_row_visible("panos", show_panos)
        self.set_row_visible("alignment", show_alignment)

    def set_row_visible(self, key, visible):
        widgets = self.input_rows.get(key)
        if not widgets:
            return
        for widget in widgets:
            if visible:
                widget.grid()
            else:
                widget.grid_remove()

    def create_section_header(self, text):
        h = ctk.CTkFrame(self.scroll_frame, fg_color="transparent")
        h.pack(fill="x", pady=(8, 6))
        ctk.CTkFrame(h, width=6, height=12, corner_radius=2, fg_color=COLORS["accent"]).pack(side="left", padx=(0, 8))
        ctk.CTkLabel(
            h,
            text=text,
            font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
            text_color=COLORS["text_dim"]
        ).pack(side="left")
        ctk.CTkFrame(h, height=1, fg_color=COLORS["border"]).pack(side="left", fill="x", expand=True, padx=(10, 0))

    def create_input_row(self, parent, row, label, var, command):
        lbl = ctk.CTkLabel(
            parent,
            text=label,
            text_color=COLORS["text_dim"],
            font=ctk.CTkFont(family=FONT_SANS, size=12)
        )
        lbl.grid(row=row, column=0, sticky="w", pady=5)
        entry = ctk.CTkEntry(
            parent,
            textvariable=var,
            height=34,
            corner_radius=6,
            fg_color=COLORS["bg"],
            border_color=COLORS["border"],
            text_color=COLORS["text"]
        )
        entry.grid(row=row, column=1, sticky="ew", padx=15, pady=5)
        btn = ctk.CTkButton(
            parent,
            text="BROWSE",
            font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
            width=86,
            height=34,
            corner_radius=6,
            fg_color=COLORS["secondary"],
            hover_color=COLORS["accent_surface"],
            text_color=COLORS["text"],
            border_width=1,
            border_color=COLORS["border"],
            command=command
        )
        btn.grid(row=row, column=2, pady=5)
        return (lbl, entry, btn)

    def create_slider_row(self, parent, label, var, min_v, max_v, step=1.0, is_int=False):
        f = ctk.CTkFrame(parent, fg_color="transparent")
        f.pack(fill="x", padx=20, pady=5)
        ctk.CTkLabel(f, text=label, width=150, anchor="w", text_color=COLORS["text"]).pack(side="left")

        val_label_var = ctk.StringVar()

        def update_val(val):
            snapped = round(round(float(val) / step) * step, 2)
            if is_int:
                int_val = int(round(snapped))
                var.set(int_val)
                val_label_var.set(str(int_val))
            else:
                var.set(snapped)
                val_label_var.set(f"{snapped:.1f}")

        # Set initial value
        update_val(var.get())

        num_steps = max(1, int(round((max_v - min_v) / step)))
        slider = ctk.CTkSlider(
            f, 
            from_=min_v, 
            to=max_v, 
            number_of_steps=num_steps, 
            variable=var, 
            command=update_val,
            fg_color=COLORS["border"], 
            progress_color=COLORS["accent"],
            button_color=COLORS["accent"],
            button_hover_color=COLORS["accent_hover"]
        )
        slider.pack(side="left", fill="x", expand=True, padx=10)
        ctk.CTkLabel(f, textvariable=val_label_var, width=50, text_color=COLORS["text"]).pack(side="left")

    def create_multiplier_slider(self, parent, label, var, min_steps, max_steps, step, multiplier, is_int=True):
        """Create a slider where displayed integer steps are multiplied to set actual value.
        min_steps and max_steps are integer counts of steps.
        multiplier converts count to actual variable value.
        """
        f = ctk.CTkFrame(parent, fg_color="transparent")
        f.pack(fill="x", padx=20, pady=5)
        ctk.CTkLabel(f, text=label, width=150, anchor="w", text_color=COLORS["text"]).pack(side="left")
        count_var = ctk.IntVar(value=int(var.get() / multiplier))
        val_label_var = ctk.StringVar()
        def update_val(count):
            actual = int(count) * multiplier
            var.set(actual)
            val_label_var.set(str(int(count)))
        # Initialize display
        update_val(count_var.get())
        num_steps = max(1, int((max_steps - min_steps) / step))
        slider = ctk.CTkSlider(
            f,
            from_=min_steps,
            to=max_steps,
            number_of_steps=num_steps,
            variable=count_var,
            command=lambda v: update_val(v),
            fg_color=COLORS["border"],
            progress_color=COLORS["accent"],
            button_color=COLORS["accent"],
            button_hover_color=COLORS["accent_hover"]
        )
        slider.pack(side="left", fill="x", expand=True, padx=10)
        ctk.CTkLabel(f, textvariable=val_label_var, width=50, text_color=COLORS["text"]).pack(side="left")

    def browse_project(self):
        d = filedialog.askdirectory(title="Select Project Directory")
        if d:
            self.project_dir.set(d)
            self.load_project_settings(d)

    def get_scan_density_pct(self) -> float:
        val_str = self.scan_density_var.get()
        try:
            val_clean = val_str.split('%')[0].strip()
            return float(val_clean)
        except Exception:
            return 100.0

    def load_project_settings(self, base_path):
        project_json_path = os.path.join(base_path, 'project.json')
        
        # Ensure directories exist
        for folder in ['input', 'output', 'logs', 'cache', 'thumbnails', 'temp']:
            os.makedirs(os.path.join(base_path, folder), exist_ok=True)
            
        def resolve_path(p):
            if not p:
                return ""
            if os.path.isabs(p):
                return p
            return os.path.abspath(os.path.join(base_path, p))

        if os.path.exists(project_json_path):
            try:
                with open(project_json_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.input_file.set(resolve_path(data.get("input_cloud", "")))
                    self.pose_csv.set(resolve_path(data.get("pose_csv", "")))
                    self.panos_dir.set(resolve_path(data.get("panorama_folder", "")))
                    self.alignment_file.set(resolve_path(data.get("alignment_file", "")))
                    self.output_file.set(resolve_path(data.get("output_e57", "")))
                    self.spacing_var.set(float(data.get("scan_spacing", 6.0)))
                    self.max_range_var.set(float(data.get("max_range", 50.0)))
                    self.mode_var.set(data.get("station_generation", "auto"))
                    self.import_workflow_var.set(data.get("import_workflow", "Auto-Detect Workflow"))

                    # Density setting
                    saved_density = float(data.get("scan_density_pct", 100.0))
                    matching_str = f"{int(saved_density)}%"
                    found = False
                    for opt in getattr(self, 'density_options', []):
                        if opt.startswith(matching_str):
                            self.scan_density_var.set(opt)
                            found = True
                            break
                    if not found:
                        self.scan_density_var.set("100% (1:1 All Panoramas - Default)")

                    self.on_workflow_change()
                    self.append_log(f"[INFO] Loaded project settings from {project_json_path}")
            except Exception as e:
                self.append_log(f"[ERROR] Failed to load project.json: {e}")
        else:
            self.save_project_settings()
            self.append_log(f"[INFO] Initialized new project structure and default project.json at {base_path}")

    def save_project_settings(self):
        d = self.project_dir.get()
        if not d or not os.path.exists(d):
            return
            
        def make_relative(p):
            if not p:
                return ""
            p = os.path.abspath(p)
            base = os.path.abspath(d)
            if p.startswith(base):
                return os.path.relpath(p, base)
            return p

        project_json_path = os.path.join(d, 'project.json')
        created_time = datetime.utcnow().isoformat() + "Z"
        if os.path.exists(project_json_path):
            try:
                with open(project_json_path, 'r', encoding='utf-8') as f:
                    old_data = json.load(f)
                    created_time = old_data.get("created", created_time)
            except Exception:
                pass

        data = {
            "project_name": os.path.basename(d),
            "project_format_version": 1,
            "software_version": "1.0.0",
            "minimum_supported_version": "1.0.0",
            "created": created_time,
            "modified": datetime.utcnow().isoformat() + "Z",
            "import_workflow": self.import_workflow_var.get(),
            "scan_density_pct": self.get_scan_density_pct(),
            "input_cloud": make_relative(self.input_file.get()),
            "pose_csv": make_relative(self.pose_csv.get()),
            "panorama_folder": make_relative(self.panos_dir.get()),
            "alignment_file": make_relative(self.alignment_file.get()),
            "output_e57": make_relative(self.output_file.get()),
            "scan_spacing": self.spacing_var.get(),
            "max_range": self.max_range_var.get(),
            "station_generation": self.mode_var.get(),
            "reduce_scans": True,
            "processing_complete": False
        }
        try:
            with open(project_json_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            self.append_log(f"[ERROR] Failed to save project.json: {e}")

    def browse_input(self):
        file = filedialog.askopenfilename(title="Select Unified Point Cloud", filetypes=(("Point Cloud files", "*.las;*.laz;*.e57"), ("All files", "*.*")))
        if file:
            self.input_file.set(file)
            self.save_project_settings()

    def browse_output(self):
        file = filedialog.asksaveasfilename(title="Save Structured Output", defaultextension=".e57", filetypes=(("E57 files", "*.e57"),))
        if file:
            self.output_file.set(file)
            self.save_project_settings()

    def browse_pose(self):
        file = filedialog.askopenfilename(title="Select Pose CSV", filetypes=(("CSV files", "*.csv"), ("All files", "*.*")))
        if file:
            self.pose_csv.set(file)
            self.save_project_settings()

    def browse_panos(self):
        d = filedialog.askdirectory(title="Select Panoramas Directory")
        if d:
            self.panos_dir.set(d)
            self.save_project_settings()

    def browse_alignment(self):
        file = filedialog.askopenfilename(title="Select Alignment Bundle", filetypes=(("NavVis Bundle", "*.bundle"), ("JSON/XML", "*.json;*.xml"), ("All files", "*.*")))
        if file:
            self.alignment_file.set(file)
            self.save_project_settings()

    def append_log(self, text):
        if not hasattr(self, 'textbox'):
            print(text)
            return
        self.textbox.configure(state="normal")
        self.textbox.insert("end", text + "\n")
        self.textbox.see("end")
        self.textbox.configure(state="disabled")

    def clear_log(self):
        if not hasattr(self, 'textbox'):
            return
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
        wf = self.import_workflow_var.get()
        input_cloud = self.input_file.get()
        output_path = self.output_file.get()
        pose_csv = self.pose_csv.get()
        panos_dir = self.panos_dir.get()
        alignment = self.alignment_file.get()

        if not input_cloud or not output_path:
            self.append_log("[ERROR] Missing required input cloud or output destination path.")
            return

        if wf == "E57 + Bundle / Reconstruction File" and not alignment:
            self.append_log("[ERROR] Workflow 'E57 + Bundle' requires an Alignment Bundle file.")
            return
        elif wf == "E57 + Panoramas Directory + Pose CSV" and not (pose_csv and panos_dir):
            self.append_log("[ERROR] Workflow 'E57 + Panoramas + Pose CSV' requires both Pose CSV and Panoramas Directory.")
            return

        self.save_project_settings()
        self.start_btn.configure(state="disabled", text="PIPELINE ACTIVE")
        threading.Thread(target=self.run_pipeline_thread, daemon=True).start()

    def run_pipeline_thread(self):
        proj_dir = self.project_dir.get()
        job_id = None
        log_file_path = None
        output_e57 = self.output_file.get()
        
        # Setup Job and logging if project dir exists
        if proj_dir and os.path.exists(proj_dir):
            try:
                # Calculate sequential Job ID
                out_dir_base = os.path.join(proj_dir, 'output')
                os.makedirs(out_dir_base, exist_ok=True)
                existing_jobs = [d for d in os.listdir(out_dir_base) if os.path.isdir(os.path.join(out_dir_base, d)) and d.startswith('Job_')]
                job_nums = []
                for job in existing_jobs:
                    try:
                        num = int(job.split('_')[1])
                        job_nums.append(num)
                    except ValueError:
                        pass
                next_num = max(job_nums) + 1 if job_nums else 1
                job_id = f"Job_{next_num:03d}"
                
                # Make Job output folder
                job_out_dir = os.path.join(out_dir_base, job_id)
                os.makedirs(job_out_dir, exist_ok=True)
                
                # Redirect E57 output into the Job folder
                orig_filename = os.path.basename(output_e57) or "structured.e57"
                output_e57 = os.path.join(job_out_dir, orig_filename)
                
                # Setup file logger
                log_dir = os.path.join(proj_dir, 'logs')
                os.makedirs(log_dir, exist_ok=True)
                log_file_path = os.path.join(log_dir, f"{job_id}_run.log")
                
                self.file_handler = logging.FileHandler(log_file_path, encoding='utf-8')
                self.file_handler.setFormatter(logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))
                self.logger.addHandler(self.file_handler)
                for m in ["io_modules", "stations", "simulation", "virtual_scanner"]:
                    logging.getLogger(m).addHandler(self.file_handler)
                    
                self.logger.info(f"Initialized processing run: {job_id}")
                self.logger.info(f"Job Output Directory: {job_out_dir}")
            except Exception as e:
                self.logger.error(f"Failed to initialize Job directory structure: {e}")
                self.after(0, lambda: self.start_btn.configure(state="normal", text="INITIALIZE PIPELINE"))
                return

        wf = self.import_workflow_var.get()
        pose_csv_path = self.pose_csv.get() or None
        panos_dir_path = self.panos_dir.get() or None
        alignment_path = self.alignment_file.get() or None

        if wf == "Structured E57 (Embedded Panoramas + Poses)":
            pose_csv_path = None
            panos_dir_path = None
        elif wf == "E57 + Bundle / Reconstruction File":
            pose_csv_path = None
            panos_dir_path = None
        elif wf == "E57 + Panoramas Directory + Pose CSV":
            alignment_path = None

        try:
            run_pipeline(
                input_cloud_path=self.input_file.get(),
                output_e57_path=output_e57,
                pose_csv_path=pose_csv_path,
                navvis_images_dir=panos_dir_path,
                mode_pref=self.mode_var.get(),
                logger_instance=self.logger,
                alignment_file=alignment_path,
                spacing=self.spacing_var.get(),
                overlap=self.overlap_var.get(),
                coverage=self.coverage_var.get(),
                max_range=self.max_range_var.get(),
                project_dir=proj_dir,
                job_id=job_id,
                dry_run=self.dry_run_var.get(),
                scan_density_pct=self.get_scan_density_pct()
            )
            self.logger.info("PIPELINE COMPLETED SUCCESSFULLY.")
        except Exception as e:
            self.logger.error(f"PIPELINE FAILURE: {str(e)}")
        finally:
            if log_file_path and hasattr(self, 'file_handler') and self.file_handler:
                self.logger.removeHandler(self.file_handler)
                for m in ["io_modules", "stations", "simulation", "virtual_scanner"]:
                    logging.getLogger(m).removeHandler(self.file_handler)
                self.file_handler.close()
                self.file_handler = None
            self.after(0, lambda: self.start_btn.configure(state="normal", text="INITIALIZE PIPELINE"))

if __name__ == "__main__":
    app = VirtualScannerGUI()
    app.mainloop()
