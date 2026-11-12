import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog
import threading
import sys
import time
import logging
import queue
import os
import json
from datetime import datetime
from PIL import Image, ImageTk, ImageDraw

from virtual_scanner import run_pipeline

# ==============================================================================
# ARTIFICIAL VISION — APPLE GLASS (visionOS) WHITE & GOLD AR DESIGN SYSTEM
# Matched to callen0817/av-scanar (src/scanhub_gui/scanhub_app.py) and
# callen0817/av-scan (gui/qml/AppWindow.qml)
# ==============================================================================
AV_THEME = {
    "gold_primary": "#BAA164",     # oklch(71.4455% .0838495 87.1018) — Artificial Vision Primary Gold
    "gold_bright": "#E5C983",      # Specular luminous gold highlight for AR glasses legibility
    "gold_muted": "#D8BF7C",       # Warm champagne gold for mono labels
    "white_pure": "#FFFFFF",       # Pure specular white
    "white_warm": "#FCFBF8",       # Warm alabaster white (prevdc foreground token)
    "white_muted": "#D0CCC2",      # Secondary translucent white-ivory text
    "glass_fill": "rgba(255, 255, 255, 0.06)",
    "glass_gold_fill": "rgba(186, 161, 100, 0.14)",
    "glass_border_white": "rgba(255, 255, 255, 0.24)",
    "glass_border_gold": "rgba(186, 161, 100, 0.52)",
    "status_ok": "#4ADE80",        # Emerald ready indicator
    "status_warn": "#F59E0B",      # Amber gold warning indicator
    "status_err": "#F87171",       # Coral red error/stop indicator
}

# CustomTkinter color tokens mapped to the Apple Glass (visionOS) palette
COLORS = {
    "shell_bg": ("#09090D", "#09090D"),
    "bg": ("#0D0D12", "#0D0D12"),
    "refraction_rim": ("#383844", "#383844"),
    "header_glass": ("#15151C", "#15151C"),
    "control_glass": ("#121218", "#121218"),
    "card": ("#14141B", "#14141B"),
    "card_inner": ("#101016", "#101016"),
    "secondary": ("#1C1C24", "#1C1C24"),
    "secondary_hover": ("#2B271E", "#2B271E"),
    "accent": ("#BAA164", "#BAA164"),
    "accent_bright": ("#E5C983", "#E5C983"),
    "accent_muted": ("#D8BF7C", "#D8BF7C"),
    "accent_hover": ("#CFAF68", "#CFAF68"),
    "accent_surface": ("#242016", "#242016"),
    "accent_btn_fill": ("#38301E", "#38301E"),
    "accent_btn_hover": ("#4F4329", "#4F4329"),
    "primary_fg": ("#FCFBF8", "#FCFBF8"),
    "text": ("#FCFBF8", "#FCFBF8"),
    "text_dim": ("#D0CCC2", "#D0CCC2"),
    "border": ("#6E5F3D", "#6E5F3D"),
    "border_subtle": ("#2E2E38", "#2E2E38"),
    "border_bright": ("#BAA164", "#BAA164"),
    "console_bg": ("#0A0A0E", "#0A0A0E"),
    "console_text": ("#FCFBF8", "#FCFBF8"),
    "destructive": ("#EF4444", "#EF4444"),
    "destructive_surface": ("#2A1518", "#2A1518"),
    "destructive_border": ("#993B3B", "#993B3B"),
    "destructive_text": ("#FCA5A5", "#FCA5A5"),
    "status_ok": ("#4ADE80", "#4ADE80"),
    "status_ok_surface": ("#12261C", "#12261C"),
    "status_ok_border": ("#286643", "#286643"),
}

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

FONT_SANS = "IBM Plex Sans" if sys.platform.startswith("linux") else "Segoe UI"
FONT_MONO = "IBM Plex Mono" if sys.platform.startswith("linux") else "Consolas"

_BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def _resolve_asset_path(*candidates):
    for rel in candidates:
        p = rel if os.path.isabs(rel) else os.path.join(_BASE_DIR, rel)
        if os.path.exists(p):
            return p
    return candidates[0]


def get_scaled_size(image_path, target_width=None, target_height=None):
    """Calculates dimensions while preserving aspect ratio."""
    try:
        if not os.path.exists(image_path):
            return (target_width or 100, target_height or 100)
        with Image.open(image_path) as img:
            orig_w, orig_h = img.size
            if target_width and target_height:
                ratio = min(target_width / orig_w, target_height / orig_h)
                return (max(1, int(orig_w * ratio)), max(1, int(orig_h * ratio)))
            elif target_width:
                ratio = target_width / orig_w
                return (target_width, max(1, int(orig_h * ratio)))
            elif target_height:
                ratio = target_height / orig_h
                return (max(1, int(orig_w * ratio)), target_height)
            return (orig_w, orig_h)
    except Exception:
        return (target_width or 100, target_height or 100)


def load_brand_logo(target_width=None, target_height=None):
    """Loads the Artificial Vision / AV Structure emblem for Apple Glass surfaces."""
    logo_path = _resolve_asset_path(
        "assets/av_structure-01.png",
        "assets/av_logo.png",
        "assets/av_structure_logo_dark.png",
        "assets/av_structure_logo.png",
        "assets/av_structure_invert.png",
    )
    scaled_size = get_scaled_size(logo_path, target_width=target_width, target_height=target_height)
    img = Image.open(logo_path)
    return ctk.CTkImage(light_image=img, dark_image=img, size=scaled_size)


def create_lidar_emblem_image(logo_size=112, canvas_size=188):
    """
    Renders the Concentric LiDAR Horizon Rings & Radial Champagne-Gold Halo
    behind the AV Structure Logo (matching ArtificialVisionSplashScreen
    in callen0817/av-scanar and splashOverlay in callen0817/av-scan).
    """
    surf = Image.new("RGBA", (canvas_size, canvas_size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(surf)
    cx = canvas_size // 2
    cy = canvas_size // 2

    # Soft radial champagne-gold optical halo
    for r in range(72, 8, -4):
        alpha = int(max(0, min(58, (1.0 - (r / 72.0)) * 58)))
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(186, 161, 100, alpha))

    # Concentric LiDAR Horizon Rings (r=86, alpha=55 and r=68, alpha=95)
    for r_px, alpha_v in [(86, 65), (68, 105)]:
        draw.ellipse(
            (cx - r_px, cy - r_px, cx + r_px, cy + r_px),
            outline=(186, 161, 100, alpha_v),
            width=1,
        )

    logo_path = _resolve_asset_path(
        "assets/av_structure-01.png",
        "assets/av_logo.png",
        "assets/av_structure_logo_dark.png",
        "assets/av_structure_logo.png",
    )
    if os.path.exists(logo_path):
        try:
            with Image.open(logo_path).convert("RGBA") as l_img:
                w, h = get_scaled_size(logo_path, target_width=logo_size, target_height=logo_size)
                l_resized = l_img.resize((w, h), Image.Resampling.LANCZOS)
                surf.alpha_composite(l_resized, (cx - w // 2, cy - h // 2))
        except Exception:
            pass
    return ctk.CTkImage(light_image=surf, dark_image=surf, size=(canvas_size, canvas_size))


class TextHandler(logging.Handler):
    def __init__(self, log_queue):
        super().__init__()
        self.log_queue = log_queue

    def emit(self, record):
        msg = self.format(record)
        self.log_queue.put(msg)


class SplashScreen(ctk.CTkToplevel):
    """
    Apple Glass (visionOS) Translucent White & Champagne Gold Splash Screen
    matching ArtificialVisionSplashScreen in callen0817/av-scanar and
    splashOverlay in callen0817/av-scan.
    """

    def __init__(self, parent, auto_close_ms=None):
        super().__init__(parent)
        self.title("AV Structure — Apple Glass AR OS")
        self.geometry("560x380")
        self.overrideredirect(True)
        self.configure(fg_color=COLORS["shell_bg"])

        # Center the splash screen
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        x = max(0, (screen_width // 2) - (560 // 2))
        y = max(0, (screen_height // 2) - (380 // 2))
        self.geometry(f"560x380+{x}+{y}")

        # Outer Champagne Gold Specular Rim (#BAA164)
        outer_shell = ctk.CTkFrame(
            self,
            fg_color=COLORS["bg"],
            border_width=2,
            border_color=AV_THEME["gold_primary"],
            corner_radius=20,
        )
        outer_shell.pack(fill="both", expand=True, padx=2, pady=2)

        # Inner Specular White Glass Hairline Rim
        card = ctk.CTkFrame(
            outer_shell,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["refraction_rim"],
            corner_radius=17,
        )
        card.pack(fill="both", expand=True, padx=3, pady=3)

        # Top Champagne Gold Specular Pill Highlight (104 x 3px)
        ctk.CTkFrame(
            card,
            width=104,
            height=3,
            corner_radius=2,
            fg_color=AV_THEME["gold_primary"],
        ).pack(side="top", pady=(2, 0))

        # Top-Right Circular Apple Glass Close Button ("✕")
        close_btn = ctk.CTkButton(
            card,
            text="✕",
            width=26,
            height=26,
            corner_radius=13,
            fg_color=COLORS["secondary"],
            hover_color=COLORS["accent_btn_fill"],
            border_width=1,
            border_color=COLORS["border"],
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            command=self.destroy,
        )
        close_btn.place(relx=1.0, x=-14, y=12, anchor="ne")

        # Concentric LiDAR Horizon Rings + Artificial Vision Emblem
        try:
            self.logo_img = create_lidar_emblem_image(logo_size=108, canvas_size=172)
            self.logo_label = ctk.CTkLabel(card, image=self.logo_img, text="")
            self.logo_label.pack(pady=(6, 0))
        except Exception:
            self.logo_label = ctk.CTkLabel(
                card,
                text="AV",
                font=ctk.CTkFont(family=FONT_SANS, size=32, weight="bold"),
                text_color=AV_THEME["gold_bright"],
            )
            self.logo_label.pack(pady=(42, 12))

        # Brand Tagline ("VERIFY  ·  DIRECT  ·  CLARIFY")
        ctk.CTkLabel(
            card,
            text="VERIFY  ·  DIRECT  ·  CLARIFY",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            text_color=AV_THEME["gold_bright"],
        ).pack(pady=(2, 2))

        # Product Title ("AV Structure")
        ctk.CTkLabel(
            card,
            text="AV Structure",
            font=ctk.CTkFont(family=FONT_SANS, size=24, weight="bold"),
            text_color=AV_THEME["white_warm"],
        ).pack(pady=(0, 1))

        # Product Subtitle ("an Artificial Vision product")
        ctk.CTkLabel(
            card,
            text="an Artificial Vision product",
            font=ctk.CTkFont(family=FONT_SANS, size=11),
            text_color=AV_THEME["white_muted"],
        ).pack(pady=(0, 16))

        # Minimalist Champagne Gold Glass Progress Bar (4px height)
        self.progress = ctk.CTkProgressBar(
            card,
            width=340,
            height=5,
            corner_radius=3,
            fg_color=COLORS["secondary"],
            progress_color=AV_THEME["gold_primary"],
        )
        self.progress.set(1.0 if auto_close_ms else 0.0)
        self.progress.pack(pady=(4, 8))

        self.status_label = ctk.CTkLabel(
            card,
            text="Artificial Vision  ·  AV Structure Apple Glass OS Ready",
            font=ctk.CTkFont(family=FONT_SANS, size=10),
            text_color=AV_THEME["white_warm"],
        )
        self.status_label.pack(pady=(0, 10))

        self.attributes("-topmost", True)
        try:
            self.attributes("-alpha", 0.0)
            self.fade_in()
        except Exception:
            pass

        if auto_close_ms:
            self.after(auto_close_ms, self._safe_destroy)

    def _safe_destroy(self):
        if self.winfo_exists():
            self.destroy()

    def fade_in(self):
        if not self.winfo_exists():
            return
        try:
            alpha = self.attributes("-alpha")
            if alpha < 0.96:
                self.attributes("-alpha", min(0.96, alpha + 0.08))
                self.after(18, self.fade_in)
        except Exception:
            pass

    def update_progress(self, val, status):
        if not self.winfo_exists():
            return
        self.progress.set(val)
        self.status_label.configure(text=status)
        self.update()


class VirtualScannerGUI(ctk.CTk):
    """
    AV Structure 2.0 — Main Desktop & Spatial OS Window.
    Designed in the Apple Glass (visionOS) + Artificial Vision White & Champagne Gold
    (#FCFBF8, #BAA164, #E5C983) interface matching callen0817/av-scanar and callen0817/av-scan.
    """

    def __init__(self):
        super().__init__()

        self.title("AV Structure — Artificial Vision Spatial Reconstruction")
        self.geometry("1340x860")
        self.minsize(1100, 720)
        self.configure(fg_color=COLORS["shell_bg"])

        # Center the main window on screen
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        x = max(0, (screen_width // 2) - (1340 // 2))
        y = max(0, (screen_height // 2) - (860 // 2))
        self.geometry(f"1340x860+{x}+{y}")

        # Apple Glass Live Opacity State (Default 12% matching scanarMini & av-scan)
        self.glass_opacity_pct = 12
        self._is_maximized = False
        self._active_splash = None
        self._pipeline_running = False
        self._pipeline_start_epoch = None

        # State Variables (Preserving full backwards compatibility)
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
        self.export_bundle_var = ctk.BooleanVar(value=True)
        self.import_workflow_var = ctk.StringVar(value="Auto-Detect Workflow")
        self.scan_density_var = ctk.StringVar(value="100% (1:1 All Panoramas - Default)")
        self.optimization_priority_var = ctk.StringVar(value="Balanced (Geometry + Panorama)")
        self.pano_res_var = ctk.StringVar(value="2048x1024 (Fast / Preview)")
        self.angular_res_var = ctk.DoubleVar(value=0.15)
        self.cancel_event = threading.Event()
        self.input_rows = {}
        self.x = None
        self.y = None

        self.create_layout()
        self.setup_logging_and_queue()
        self._tick_elapsed_clock()

        # Bring window to front
        self.lift()
        self.focus_force()
        self.attributes("-topmost", True)
        self.after(300, lambda: self.attributes("-topmost", False))

    # -------------------------------------------------------------------------
    # Window Dragging, Maximize/Restore, Splash & Live Apple Glass Opacity
    # -------------------------------------------------------------------------

    def start_move(self, event):
        self.x = event.x
        self.y = event.y

    def stop_move(self, event):
        self.x = None
        self.y = None

    def do_move(self, event):
        if self.x is None or self.y is None or self._is_maximized:
            return
        deltax = event.x - self.x
        deltay = event.y - self.y
        x = self.winfo_x() + deltax
        y = self.winfo_y() + deltay
        self.geometry(f"+{x}+{y}")

    def toggle_window_maximize(self, event=None):
        """Toggles between maximized and normal window state (matching ScanHUBOS.toggle_window_maximize)."""
        try:
            if not self._is_maximized:
                self._normal_geo = self.geometry()
                sw = self.winfo_screenwidth()
                sh = self.winfo_screenheight()
                self.geometry(f"{sw}x{sh}+0+0")
                self._is_maximized = True
                if hasattr(self, "btn_win_max"):
                    self.btn_win_max.configure(text="❐")
            else:
                self.geometry(getattr(self, "_normal_geo", "1340x860+100+60"))
                self._is_maximized = False
                if hasattr(self, "btn_win_max"):
                    self.btn_win_max.configure(text="□")
        except Exception:
            pass

    def toggle_theme(self):
        current = ctk.get_appearance_mode()
        next_mode = "Dark" if current == "Light" else "Light"
        ctk.set_appearance_mode(next_mode)

    def show_brand_splash(self, duration_ms=2600):
        """Displays the Artificial Vision Apple Glass Splash Screen when clicking the header emblem."""
        try:
            if self._active_splash is not None and self._active_splash.winfo_exists():
                self._active_splash.destroy()
            self._active_splash = SplashScreen(self, auto_close_ms=duration_ms)
        except Exception:
            pass

    def on_glass_opacity_changed(self, val):
        """
        Adjusts the Apple Glass window surface opacity live (0% to 90%, default 12%
        matching scanarMini and av-scan).
        """
        self.glass_opacity_pct = int(round(float(val)))
        if hasattr(self, "lbl_glass_alpha"):
            self.lbl_glass_alpha.configure(text=f"Glass {self.glass_opacity_pct}%")
        # Map 0..90% glass slider to window alpha (0.72..1.0 on desktop compositors) and card tint
        win_alpha = 0.78 + (self.glass_opacity_pct / 90.0) * 0.22
        try:
            self.attributes("-alpha", float(min(1.0, max(0.75, win_alpha))))
        except Exception:
            pass

        # Dynamically shift card glass surface luminance based on glass_opacity_pct
        base_v = int(16 + (self.glass_opacity_pct / 90.0) * 18)
        hdr_v = int(19 + (self.glass_opacity_pct / 90.0) * 20)
        card_hex = f"#{base_v:02x}{base_v:02x}{min(255, base_v + 6):02x}"
        hdr_hex = f"#{hdr_v:02x}{hdr_v:02x}{min(255, hdr_v + 7):02x}"
        for frame_attr in ("header_bar", "ctrl_bar", "input_panel", "output_panel", "telemetry_panel", "settings_frame", "console_card"):
            widget = getattr(self, frame_attr, None)
            if widget is not None:
                try:
                    widget.configure(fg_color=hdr_hex if "bar" in frame_attr else card_hex)
                except Exception:
                    pass

    def _tick_elapsed_clock(self):
        if self._pipeline_running and self._pipeline_start_epoch is not None:
            elapsed = max(0, int(time.time() - self._pipeline_start_epoch))
            mins, secs = divmod(elapsed, 60)
            if hasattr(self, "clock_lbl"):
                self.clock_lbl.configure(text=f"⏱ {mins:02d}:{secs:02d}")
        self.after(500, self._tick_elapsed_clock)

    # -------------------------------------------------------------------------
    # Apple Glass 4-Row Layout (Matches AppWindow.qml & ScanHUBOS.init_ui)
    # -------------------------------------------------------------------------

    def create_layout(self):
        # Outer Apple Glass Shell with 1.5px Champagne Gold Specular Rim (#BAA164)
        self.outer_glass_shell = ctk.CTkFrame(
            self,
            corner_radius=16,
            fg_color=COLORS["bg"],
            border_width=2,
            border_color=AV_THEME["gold_primary"],
        )
        self.outer_glass_shell.pack(fill="both", expand=True, padx=4, pady=4)

        # Inner White Glass Refraction Rim (inset by 2px, matching outerGlassShell in AppWindow.qml)
        self.inner_refraction_rim = ctk.CTkFrame(
            self.outer_glass_shell,
            corner_radius=14,
            fg_color=COLORS["bg"],
            border_width=1,
            border_color=COLORS["refraction_rim"],
        )
        self.inner_refraction_rim.pack(fill="both", expand=True, padx=2, pady=2)

        # Main 4-Row Container
        self.main_column = ctk.CTkFrame(self.inner_refraction_rim, fg_color="transparent")
        self.main_column.pack(fill="both", expand=True, padx=8, pady=6)

        # =====================================================================
        # ROW 1: TOP APPLE GLASS HEADER BAR (height: 52px)
        # =====================================================================
        self.header_bar = ctk.CTkFrame(
            self.main_column,
            height=52,
            corner_radius=11,
            fg_color=COLORS["header_glass"],
            border_width=1,
            border_color=COLORS["border"],
        )
        self.header_bar.pack(fill="x", side="top", pady=(0, 6))
        self.header_bar.pack_propagate(False)

        # Bind window drag & double-click maximize to header bar
        for w_drag in (self.header_bar,):
            w_drag.bind("<ButtonPress-1>", self.start_move)
            w_drag.bind("<ButtonRelease-1>", self.stop_move)
            w_drag.bind("<B1-Motion>", self.do_move)
            w_drag.bind("<Double-Button-1>", self.toggle_window_maximize)

        # 1A. 36x36 Circular Artificial Vision Emblem Badge (clicks open Splash Screen)
        try:
            self.hdr_logo_img = load_brand_logo(target_width=24, target_height=24)
            self.logo_btn = ctk.CTkButton(
                self.header_bar,
                text="",
                image=self.hdr_logo_img,
                width=36,
                height=36,
                corner_radius=18,
                fg_color=COLORS["accent_surface"],
                hover_color=COLORS["accent_btn_fill"],
                border_width=1,
                border_color=AV_THEME["gold_primary"],
                command=lambda: self.show_brand_splash(2600),
            )
        except Exception:
            self.logo_btn = ctk.CTkButton(
                self.header_bar,
                text="AV",
                width=36,
                height=36,
                corner_radius=18,
                fg_color=COLORS["accent_surface"],
                hover_color=COLORS["accent_btn_fill"],
                border_width=1,
                border_color=AV_THEME["gold_primary"],
                text_color=AV_THEME["gold_bright"],
                font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
                command=lambda: self.show_brand_splash(2600),
            )
        self.logo_btn.pack(side="left", padx=(12, 10), pady=8)

        # 1B. Title & Subtitle Stack ("AV Structure" / "an Artificial Vision product")
        title_stack = ctk.CTkFrame(self.header_bar, fg_color="transparent")
        title_stack.pack(side="left", pady=6)
        lbl_main_title = ctk.CTkLabel(
            title_stack,
            text="AV Structure",
            font=ctk.CTkFont(family=FONT_SANS, size=17, weight="bold"),
            text_color=AV_THEME["white_warm"],
            height=20,
        )
        lbl_main_title.pack(anchor="w")
        lbl_sub_title = ctk.CTkLabel(
            title_stack,
            text="an Artificial Vision product",
            font=ctk.CTkFont(family=FONT_SANS, size=10),
            text_color=AV_THEME["white_muted"],
            height=14,
        )
        lbl_sub_title.pack(anchor="w")
        for w_t in (title_stack, lbl_main_title, lbl_sub_title):
            w_t.bind("<ButtonPress-1>", self.start_move)
            w_t.bind("<ButtonRelease-1>", self.stop_move)
            w_t.bind("<B1-Motion>", self.do_move)
            w_t.bind("<Double-Button-1>", self.toggle_window_maximize)

        # 1C. Brand Pill ("VERIFY · DIRECT · CLARIFY" — matched to av-scanar & av-scan)
        brand_pill_frame = ctk.CTkFrame(
            self.header_bar,
            fg_color=COLORS["accent_surface"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=10,
            height=24,
        )
        brand_pill_frame.pack(side="left", padx=(12, 8), pady=14)
        ctk.CTkLabel(
            brand_pill_frame,
            text="VERIFY · DIRECT · CLARIFY",
            font=ctk.CTkFont(family=FONT_MONO, size=9, weight="bold"),
            text_color=AV_THEME["gold_bright"],
        ).pack(padx=10, pady=2)

        # 1D. Right-Side Circular Apple Glass Window Controls (Close, Max/Restore, Minimize)
        self.btn_win_close = ctk.CTkButton(
            self.header_bar,
            text="✕",
            width=26,
            height=26,
            corner_radius=13,
            fg_color=COLORS["secondary"],
            hover_color=COLORS["destructive"],
            border_width=1,
            border_color=COLORS["border_subtle"],
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            command=self.quit,
        )
        self.btn_win_close.pack(side="right", padx=(4, 12), pady=13)

        self.btn_win_max = ctk.CTkButton(
            self.header_bar,
            text="□",
            width=26,
            height=26,
            corner_radius=13,
            fg_color=COLORS["accent_surface"],
            hover_color=COLORS["accent_btn_hover"],
            border_width=1,
            border_color=AV_THEME["gold_primary"],
            text_color=AV_THEME["gold_bright"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            command=self.toggle_window_maximize,
        )
        self.btn_win_max.pack(side="right", padx=3, pady=13)

        self.btn_win_min = ctk.CTkButton(
            self.header_bar,
            text="—",
            width=26,
            height=26,
            corner_radius=13,
            fg_color=COLORS["secondary"],
            hover_color=COLORS["accent_btn_fill"],
            border_width=1,
            border_color=COLORS["border_subtle"],
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            command=self.iconify,
        )
        self.btn_win_min.pack(side="right", padx=(8, 3), pady=13)

        # 1E. Apple Glass Status Pills (Engine Status, Workflow Pill, E57 Spec Pill)
        self.pill_e57 = self._create_status_pill(
            self.header_bar,
            "● E57: ASTM E2807",
            text_color=AV_THEME["white_muted"],
            fg_color=COLORS["secondary"],
            border_color=COLORS["border_subtle"],
        )
        self.pill_e57.pack(side="right", padx=4, pady=14)

        self.pill_workflow = self._create_status_pill(
            self.header_bar,
            "● WORKFLOW: AUTO-DETECT",
            text_color=AV_THEME["gold_bright"],
            fg_color=COLORS["accent_surface"],
            border_color=COLORS["border"],
        )
        self.pill_workflow.pack(side="right", padx=4, pady=14)

        self.pill_engine = self._create_status_pill(
            self.header_bar,
            "● ENGINE: READY",
            text_color=AV_THEME["status_ok"],
            fg_color=COLORS["status_ok_surface"],
            border_color=COLORS["status_ok_border"],
        )
        self.pill_engine.pack(side="right", padx=4, pady=14)

        # 1F. Live AR Glass Opacity Slider ("Glass 12%", 0..90% matching scanarMini & av-scan)
        glass_ctrl_box = ctk.CTkFrame(self.header_bar, fg_color="transparent")
        glass_ctrl_box.pack(side="right", padx=(4, 8), pady=12)

        self.lbl_glass_alpha = ctk.CTkLabel(
            glass_ctrl_box,
            text=f"Glass {self.glass_opacity_pct}%",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            text_color=AV_THEME["gold_bright"],
        )
        self.lbl_glass_alpha.pack(side="left", padx=(0, 6))

        self.slider_glass_alpha = ctk.CTkSlider(
            glass_ctrl_box,
            from_=0,
            to=90,
            number_of_steps=90,
            width=74,
            height=14,
            fg_color=COLORS["border_subtle"],
            progress_color=AV_THEME["gold_primary"],
            button_color=AV_THEME["white_warm"],
            button_hover_color=AV_THEME["gold_bright"],
            command=self.on_glass_opacity_changed,
        )
        self.slider_glass_alpha.set(self.glass_opacity_pct)
        self.slider_glass_alpha.pack(side="left")

        # =====================================================================
        # ROW 2: ACTION & SESSION CONTROL BAR (height: 48px, matches ctrl_bar in ScanHUBOS)
        # =====================================================================
        self.ctrl_bar = ctk.CTkFrame(
            self.main_column,
            height=48,
            corner_radius=10,
            fg_color=COLORS["control_glass"],
            border_width=1,
            border_color=COLORS["border_subtle"],
        )
        self.ctrl_bar.pack(fill="x", side="top", pady=(0, 6))
        self.ctrl_bar.pack_propagate(False)

        self.start_btn = ctk.CTkButton(
            self.ctrl_bar,
            text="▶ INITIALIZE PIPELINE",
            font=ctk.CTkFont(family=FONT_SANS, size=12, weight="bold"),
            height=34,
            width=210,
            corner_radius=9,
            fg_color=COLORS["accent_btn_fill"],
            hover_color=COLORS["accent_btn_hover"],
            border_width=1,
            border_color=AV_THEME["gold_primary"],
            text_color=AV_THEME["white_warm"],
            command=self.start_pipeline,
        )
        self.start_btn.pack(side="left", padx=(12, 6), pady=7)

        self.cancel_btn = ctk.CTkButton(
            self.ctrl_bar,
            text="■ CANCEL",
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            height=34,
            width=106,
            corner_radius=9,
            fg_color=COLORS["destructive_surface"],
            hover_color=COLORS["destructive"],
            border_width=1,
            border_color=COLORS["destructive_border"],
            text_color=COLORS["destructive_text"],
            state="disabled",
            command=self.cancel_pipeline,
        )
        self.cancel_btn.pack(side="left", padx=4, pady=7)

        self.btn_quick_dry_run = ctk.CTkButton(
            self.ctrl_bar,
            text="✦ DRY RUN: OFF",
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            height=34,
            width=132,
            corner_radius=9,
            fg_color=COLORS["secondary"],
            hover_color=COLORS["accent_surface"],
            border_width=1,
            border_color=COLORS["border"],
            text_color=AV_THEME["white_muted"],
            command=self._toggle_quick_dry_run,
        )
        self.btn_quick_dry_run.pack(side="left", padx=4, pady=7)

        ctk.CTkButton(
            self.ctrl_bar,
            text="↺ CLEAR LOGS",
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            height=34,
            width=120,
            corner_radius=9,
            fg_color=COLORS["secondary"],
            hover_color=COLORS["accent_surface"],
            border_width=1,
            border_color=COLORS["border_subtle"],
            text_color=AV_THEME["white_warm"],
            command=self.clear_log,
        ).pack(side="left", padx=4, pady=7)

        # Elapsed Timer Badge ("⏱ 00:00" — matches clock_lbl in ScanHUBOS)
        clock_frame = ctk.CTkFrame(
            self.ctrl_bar,
            fg_color=COLORS["secondary"],
            border_width=1,
            border_color=COLORS["border_subtle"],
            corner_radius=9,
            height=32,
        )
        clock_frame.pack(side="left", padx=6, pady=8)
        self.clock_lbl = ctk.CTkLabel(
            clock_frame,
            text="⏱ 00:00",
            font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
            text_color=AV_THEME["white_warm"],
        )
        self.clock_lbl.pack(padx=12, pady=4)

        # Right-side Project Badge & Quick Project Folder Selector
        ctk.CTkButton(
            self.ctrl_bar,
            text="📁 PROJECT DIR",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            height=32,
            width=126,
            corner_radius=9,
            fg_color=COLORS["accent_surface"],
            hover_color=COLORS["accent_btn_fill"],
            border_width=1,
            border_color=COLORS["border"],
            text_color=AV_THEME["gold_bright"],
            command=self.browse_project,
        ).pack(side="right", padx=(6, 12), pady=8)

        self.lbl_active_project_badge = ctk.CTkLabel(
            self.ctrl_bar,
            text="PROJECT: STANDALONE SESSION",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            text_color=AV_THEME["gold_muted"],
        )
        self.lbl_active_project_badge.pack(side="right", padx=6, pady=8)

        # =====================================================================
        # ROW 4: TRANSLUCENT APPLE GLASS FOOTER STATUS BAR (height: 28px)
        # Packed side="bottom" before Row 3 so it stays pinned at bottom of shell
        # =====================================================================
        self.footer_bar = ctk.CTkFrame(
            self.main_column,
            height=28,
            corner_radius=10,
            fg_color=COLORS["header_glass"],
            border_width=1,
            border_color=COLORS["border"],
        )
        self.footer_bar.pack(fill="x", side="bottom", pady=(6, 0))
        self.footer_bar.pack_propagate(False)

        self.footer_status_lbl = ctk.CTkLabel(
            self.footer_bar,
            text="VERIFY · DIRECT · CLARIFY  |  AV Structure 2.0  ·  Universal Virtual Scan & 360° Panorama Reconstruction Engine",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            text_color=AV_THEME["gold_bright"],
        )
        self.footer_status_lbl.pack(side="left", padx=14, pady=3)

        ctk.CTkLabel(
            self.footer_bar,
            text="APPLE GLASS SPATIAL UI",
            font=ctk.CTkFont(family=FONT_MONO, size=9, weight="bold"),
            text_color=AV_THEME["white_muted"],
        ).pack(side="right", padx=14, pady=3)

        # =====================================================================
        # ROW 3: SPLIT 2-COLUMN APPLE GLASS WORKSPACE
        #   Left Column (56%): Input Data Sources, Output Config & Telemetry HUD
        #   Right Column (44%): Scan Generation Settings Deck & Process Logs Console
        # =====================================================================
        self.workspace_split = ctk.CTkFrame(self.main_column, fg_color="transparent")
        self.workspace_split.pack(fill="both", expand=True, side="top")
        self.workspace_split.grid_columnconfigure(0, weight=56, uniform="ws_col")
        self.workspace_split.grid_columnconfigure(1, weight=44, uniform="ws_col")
        self.workspace_split.grid_rowconfigure(0, weight=1)

        self.left_column = ctk.CTkScrollableFrame(
            self.workspace_split,
            fg_color="transparent",
            corner_radius=0,
        )
        self.left_column.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=0)

        self.right_column = ctk.CTkFrame(
            self.workspace_split,
            fg_color="transparent",
        )
        self.right_column.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=0)

        self.create_widgets()

    def _create_status_pill(self, parent, text, text_color, fg_color, border_color):
        pill = ctk.CTkFrame(
            parent,
            fg_color=fg_color,
            border_width=1,
            border_color=border_color,
            corner_radius=10,
            height=24,
        )
        lbl = ctk.CTkLabel(
            pill,
            text=text,
            font=ctk.CTkFont(family=FONT_MONO, size=9, weight="bold"),
            text_color=text_color,
        )
        lbl.pack(padx=9, pady=2)
        pill._label = lbl
        return pill

    def _set_pill_state(self, pill, text, text_color=None, fg_color=None, border_color=None):
        if not hasattr(pill, "_label"):
            return
        pill._label.configure(text=text)
        if text_color:
            pill._label.configure(text_color=text_color)
        if fg_color:
            pill.configure(fg_color=fg_color)
        if border_color:
            pill.configure(border_color=border_color)

    def _toggle_quick_dry_run(self):
        new_val = not self.dry_run_var.get()
        self.dry_run_var.set(new_val)
        self._sync_dry_run_button()

    def _sync_dry_run_button(self):
        is_on = bool(self.dry_run_var.get())
        if hasattr(self, "btn_quick_dry_run"):
            if is_on:
                self.btn_quick_dry_run.configure(
                    text="✦ DRY RUN: ON",
                    fg_color=COLORS["accent_btn_fill"],
                    border_color=AV_THEME["gold_bright"],
                    text_color=AV_THEME["gold_bright"],
                )
            else:
                self.btn_quick_dry_run.configure(
                    text="✦ DRY RUN: OFF",
                    fg_color=COLORS["secondary"],
                    border_color=COLORS["border"],
                    text_color=AV_THEME["white_muted"],
                )

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
        # =====================================================================
        # LEFT COLUMN: INPUT DATA SOURCES + OUTPUT CONFIG + TELEMETRY MATRIX
        # =====================================================================
        self.create_section_header(self.left_column, "INPUT DATA SOURCES  ·  UNIVERSAL INGESTION")

        self.input_panel = ctk.CTkFrame(
            self.left_column,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=12,
        )
        self.input_panel.pack(fill="x", pady=(0, 10))

        # Workflow Dropdown Row
        wf_frame = ctk.CTkFrame(self.input_panel, fg_color="transparent")
        wf_frame.pack(fill="x", padx=14, pady=(12, 6))
        ctk.CTkLabel(
            wf_frame,
            text="Import Workflow Mode",
            font=ctk.CTkFont(family=FONT_SANS, size=12, weight="bold"),
            text_color=AV_THEME["white_warm"],
        ).pack(side="left", padx=(0, 12))

        self.workflow_options = [
            "Auto-Detect Workflow",
            "Structured E57 (Embedded Panoramas + Poses)",
            "E57 + Bundle / Reconstruction File",
            "E57 + Panoramas Directory + Pose CSV",
            "Generic Colorized Point Cloud (No Panoramas)",
        ]
        self.workflow_dropdown = ctk.CTkOptionMenu(
            wf_frame,
            variable=self.import_workflow_var,
            values=self.workflow_options,
            fg_color=COLORS["secondary"],
            button_color=COLORS["accent_btn_fill"],
            button_hover_color=COLORS["accent_btn_hover"],
            dropdown_fg_color=COLORS["card"],
            dropdown_hover_color=COLORS["accent_surface"],
            dropdown_text_color=AV_THEME["white_warm"],
            text_color=AV_THEME["gold_bright"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            corner_radius=8,
            height=32,
            width=340,
            command=self.on_workflow_change,
        )
        self.workflow_dropdown.pack(side="right", fill="x", expand=True)

        input_grid = ctk.CTkFrame(self.input_panel, fg_color="transparent")
        input_grid.pack(fill="x", padx=14, pady=(4, 12))
        input_grid.grid_columnconfigure(1, weight=1)

        self.input_rows["project"] = self.create_input_row(
            input_grid, 0, "Project Directory", self.project_dir, self.browse_project
        )
        self.input_rows["cloud"] = self.create_input_row(
            input_grid, 1, "Point Cloud (E57/LAS/PLY/PTS)", self.input_file, self.browse_input
        )
        self.input_rows["pose"] = self.create_input_row(
            input_grid, 2, "Poses CSV", self.pose_csv, self.browse_pose
        )
        self.input_rows["panos"] = self.create_input_row(
            input_grid, 3, "Panoramas Directory", self.panos_dir, self.browse_panos
        )
        self.input_rows["alignment"] = self.create_input_row(
            input_grid, 4, "Alignment Bundle", self.alignment_file, self.browse_alignment
        )

        # Output Configuration Card
        self.create_section_header(self.left_column, "OUTPUT CONFIGURATION  ·  STRUCTURED E57 DELIVERABLE")
        self.output_panel = ctk.CTkFrame(
            self.left_column,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=12,
        )
        self.output_panel.pack(fill="x", pady=(0, 10))

        output_grid = ctk.CTkFrame(self.output_panel, fg_color="transparent")
        output_grid.pack(fill="x", padx=14, pady=10)
        output_grid.grid_columnconfigure(1, weight=1)
        self.create_input_row(output_grid, 0, "Destination Path (.e57)", self.output_file, self.browse_output)

        self.chk_export_bundle = ctk.CTkCheckBox(
            self.output_panel,
            text="Export AV Showcase Walkthrough Bundle (v1.1 + Keyplans + lod0.bin)",
            variable=self.export_bundle_var,
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            text_color=AV_THEME["white_warm"],
            fg_color=AV_THEME["gold_primary"],
            hover_color=COLORS["accent_hover"],
            border_color=COLORS["border"],
        )
        self.chk_export_bundle.pack(anchor="w", padx=18, pady=(0, 10))

        # Spatial Reconstruction Telemetry & E57 Structural Specification Card
        self.create_section_header(self.left_column, "SPATIAL RECONSTRUCTION TELEMETRY  ·  ASTM E2807")
        self.telemetry_panel = ctk.CTkFrame(
            self.left_column,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=12,
        )
        self.telemetry_panel.pack(fill="x", pady=(0, 8))

        stage_row = ctk.CTkFrame(self.telemetry_panel, fg_color="transparent")
        stage_row.pack(fill="x", padx=14, pady=(10, 4))
        self.lbl_stage_status = ctk.CTkLabel(
            stage_row,
            text="STAGE 0  ·  Awaiting input point cloud & destination path",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            text_color=AV_THEME["gold_bright"],
        )
        self.lbl_stage_status.pack(side="left")

        self.pipeline_progress = ctk.CTkProgressBar(
            self.telemetry_panel,
            height=6,
            corner_radius=3,
            fg_color=COLORS["secondary"],
            progress_color=AV_THEME["gold_primary"],
        )
        self.pipeline_progress.set(0.0)
        self.pipeline_progress.pack(fill="x", padx=14, pady=(2, 10))

        # 2x2 Apple Glass Architectural Capability Badges
        spec_grid = ctk.CTkFrame(self.telemetry_panel, fg_color="transparent")
        spec_grid.pack(fill="x", padx=14, pady=(0, 12))
        spec_grid.grid_columnconfigure((0, 1), weight=1, uniform="spec_c")

        specs = [
            ("2D SPHERICAL GRID", "rowIndex · columnIndex · sphericalBounds · groupingByLine"),
            ("PANORAMA ENGINE", "Preserve Source JPEG or Render Occlusion-Culled 360° RGB"),
            ("GUID ASSOCIATION", "Data3D Pose Quaternion + images2D associatedData3DGuid"),
            ("REIMPORT VALIDATION", "Automatic Post-Export pye57 / libe57 Structural Verification"),
        ]
        for idx, (sp_title, sp_desc) in enumerate(specs):
            r_i, c_i = divmod(idx, 2)
            sp_card = ctk.CTkFrame(
                spec_grid,
                fg_color=COLORS["card_inner"],
                border_width=1,
                border_color=COLORS["border_subtle"],
                corner_radius=8,
            )
            sp_card.grid(row=r_i, column=c_i, sticky="nsew", padx=4, pady=4)
            ctk.CTkLabel(
                sp_card,
                text=f"✦ {sp_title}",
                font=ctk.CTkFont(family=FONT_MONO, size=9, weight="bold"),
                text_color=AV_THEME["gold_bright"],
            ).pack(anchor="w", padx=10, pady=(6, 1))
            ctk.CTkLabel(
                sp_card,
                text=sp_desc,
                font=ctk.CTkFont(family=FONT_SANS, size=10),
                text_color=AV_THEME["white_muted"],
                justify="left",
            ).pack(anchor="w", padx=10, pady=(0, 6))

        # =====================================================================
        # RIGHT COLUMN: PERIPHERAL HUD DECK (SETTINGS + PROCESS LOGS CONSOLE)
        # =====================================================================
        self.create_section_header(self.right_column, "SCAN GENERATION SETTINGS  ·  PERIPHERAL HUD")
        self.settings_frame = ctk.CTkFrame(
            self.right_column,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=12,
        )
        self.settings_frame.pack(fill="x", pady=(0, 10), ipady=6)

        # Mode Selection Row
        ctk.CTkLabel(
            self.settings_frame,
            text="VIRTUAL STATION GENERATION MODE",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            text_color=AV_THEME["gold_muted"],
        ).pack(anchor="w", padx=16, pady=(10, 4))

        mode_box = ctk.CTkFrame(self.settings_frame, fg_color="transparent")
        mode_box.pack(fill="x", padx=16, pady=(0, 6))
        ctk.CTkRadioButton(
            mode_box,
            text="Auto-Detection",
            variable=self.mode_var,
            value="auto",
            fg_color=AV_THEME["gold_primary"],
            hover_color=AV_THEME["gold_bright"],
            border_color=COLORS["border"],
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
        ).pack(side="left", padx=(0, 14))
        ctk.CTkRadioButton(
            mode_box,
            text="Trajectory Driven",
            variable=self.mode_var,
            value="trajectory",
            fg_color=AV_THEME["gold_primary"],
            hover_color=AV_THEME["gold_bright"],
            border_color=COLORS["border"],
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
        ).pack(side="left", padx=10)
        ctk.CTkRadioButton(
            mode_box,
            text="Geometry Focused",
            variable=self.mode_var,
            value="geometry",
            fg_color=AV_THEME["gold_primary"],
            hover_color=AV_THEME["gold_bright"],
            border_color=COLORS["border"],
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
        ).pack(side="left", padx=10)
        ctk.CTkCheckBox(
            mode_box,
            text="Dry Run",
            variable=self.dry_run_var,
            fg_color=AV_THEME["gold_primary"],
            hover_color=AV_THEME["gold_bright"],
            border_color=COLORS["border"],
            checkmark_color=COLORS["bg"],
            text_color=AV_THEME["gold_bright"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            command=self._sync_dry_run_button,
        ).pack(side="right", padx=(0, 4))

        # Champagne Gold Sliders
        self.create_multiplier_slider(
            self.settings_frame, "Station Spacing (m)", self.spacing_var, 2, 30, step=1, multiplier=0.5
        )
        self.create_multiplier_slider(
            self.settings_frame, "Max Scan Range (m)", self.max_range_var, 1, 20, step=1, multiplier=5
        )

        # Scan Density (% of Panoramas) Dropdown
        self.density_box = ctk.CTkFrame(self.settings_frame, fg_color="transparent")
        self.density_box.pack(fill="x", padx=16, pady=(4, 3))
        ctk.CTkLabel(
            self.density_box,
            text="Scan Density (% of Panoramas)",
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            text_color=AV_THEME["white_muted"],
            width=205,
            anchor="w",
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
            "10%",
        ]
        self.density_dropdown = ctk.CTkOptionMenu(
            self.density_box,
            variable=self.scan_density_var,
            values=self.density_options,
            fg_color=COLORS["secondary"],
            button_color=COLORS["accent_btn_fill"],
            button_hover_color=COLORS["accent_btn_hover"],
            dropdown_fg_color=COLORS["card"],
            dropdown_hover_color=COLORS["accent_surface"],
            dropdown_text_color=AV_THEME["white_warm"],
            text_color=AV_THEME["gold_bright"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            corner_radius=8,
            height=30,
            width=240,
        )
        self.density_dropdown.pack(side="right", fill="x", expand=True, padx=(8, 0))

        # Optimization Priority Dropdown
        self.priority_box = ctk.CTkFrame(self.settings_frame, fg_color="transparent")
        self.priority_box.pack(fill="x", padx=16, pady=(4, 3))
        ctk.CTkLabel(
            self.priority_box,
            text="Station Optimization Priority",
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            text_color=AV_THEME["white_muted"],
            width=205,
            anchor="w",
        ).pack(side="left")

        self.priority_options = [
            "Balanced (Geometry + Panorama)",
            "Maximum Geometric Coverage",
            "Best Panorama Quality",
            "Minimum Station Count (Fastest)",
        ]
        self.priority_dropdown = ctk.CTkOptionMenu(
            self.priority_box,
            variable=self.optimization_priority_var,
            values=self.priority_options,
            fg_color=COLORS["secondary"],
            button_color=COLORS["accent_btn_fill"],
            button_hover_color=COLORS["accent_btn_hover"],
            dropdown_fg_color=COLORS["card"],
            dropdown_hover_color=COLORS["accent_surface"],
            dropdown_text_color=AV_THEME["white_warm"],
            text_color=AV_THEME["gold_bright"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            corner_radius=8,
            height=30,
            width=240,
        )
        self.priority_dropdown.pack(side="right", fill="x", expand=True, padx=(8, 0))

        # Synthesized Panorama Resolution Dropdown
        self.pano_res_box = ctk.CTkFrame(self.settings_frame, fg_color="transparent")
        self.pano_res_box.pack(fill="x", padx=16, pady=(4, 4))
        ctk.CTkLabel(
            self.pano_res_box,
            text="Synthesized Panorama Resolution",
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            text_color=AV_THEME["white_muted"],
            width=205,
            anchor="w",
        ).pack(side="left")

        self.pano_res_options = [
            "2048x1024 (Fast / Preview)",
            "4096x2048 (Standard)",
            "8192x4096 (High Density)",
        ]
        self.pano_res_dropdown = ctk.CTkOptionMenu(
            self.pano_res_box,
            variable=self.pano_res_var,
            values=self.pano_res_options,
            fg_color=COLORS["secondary"],
            button_color=COLORS["accent_btn_fill"],
            button_hover_color=COLORS["accent_btn_hover"],
            dropdown_fg_color=COLORS["card"],
            dropdown_hover_color=COLORS["accent_surface"],
            dropdown_text_color=AV_THEME["white_warm"],
            text_color=AV_THEME["gold_bright"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
            corner_radius=8,
            height=30,
            width=240,
        )
        self.pano_res_dropdown.pack(side="right", fill="x", expand=True, padx=(8, 0))

        # Initial visibility based on default workflow option
        self.on_workflow_change()

        # =====================================================================
        # TRANSLUCENT APPLE GLASS STUDIO LOG CONSOLE (matches self.console in ScanHUBOS)
        # =====================================================================
        self.create_section_header(self.right_column, "TELEMETRY & PROCESS LOGS CONSOLE")
        self.console_card = ctk.CTkFrame(
            self.right_column,
            fg_color=COLORS["card"],
            border_width=1,
            border_color=COLORS["border"],
            corner_radius=12,
        )
        self.console_card.pack(fill="both", expand=True, pady=(0, 2))

        self.textbox = ctk.CTkTextbox(
            self.console_card,
            corner_radius=9,
            fg_color=COLORS["console_bg"],
            border_width=1,
            border_color=COLORS["border_subtle"],
            font=ctk.CTkFont(family=FONT_MONO, size=11),
            text_color=COLORS["console_text"],
        )
        self.textbox.pack(fill="both", expand=True, padx=8, pady=8)
        self.textbox.insert(
            "end",
            "[Artificial Vision · AV Structure 2.0] Apple Glass White & Gold Spatial OS Initialized.\n"
            "[E57 ENGINE] ASTM E2807 2D Spherical Grid (rowIndex/columnIndex) + associatedData3DGuid ONLINE.\n"
            "[PANORAMA ENGINE] Dual-mode: Byte-preserving camera JPEG pass-through & Depth-Buffered 360° synthesis READY.\n",
        )
        self.textbox.configure(state="disabled")

    def on_workflow_change(self, selected_workflow=None):
        wf = self.import_workflow_var.get()
        show_pose = True
        show_panos = True
        show_alignment = True
        show_existing_pano_density = True
        show_synthesis_controls = True
        short_wf = "AUTO-DETECT"

        if wf == "Structured E57 (Embedded Panoramas + Poses)":
            show_pose = False
            show_panos = False
            show_alignment = True
            show_existing_pano_density = True
            show_synthesis_controls = False
            short_wf = "EMBEDDED E57"
        elif wf == "E57 + Bundle / Reconstruction File":
            show_pose = False
            show_panos = False
            show_alignment = True
            show_existing_pano_density = True
            show_synthesis_controls = False
            short_wf = "E57 + BUNDLE"
        elif wf == "E57 + Panoramas Directory + Pose CSV":
            show_pose = True
            show_panos = True
            show_alignment = False
            show_existing_pano_density = True
            show_synthesis_controls = False
            short_wf = "E57 + PANO CSV"
        elif wf == "Generic Colorized Point Cloud (No Panoramas)":
            show_pose = False
            show_panos = False
            show_alignment = False
            show_existing_pano_density = False
            show_synthesis_controls = True
            short_wf = "SYNTHETIC 360°"
        elif wf == "Auto-Detect Workflow":
            show_pose = True
            show_panos = True
            show_alignment = True
            show_existing_pano_density = True
            show_synthesis_controls = True
            short_wf = "AUTO-DETECT"

        self.set_row_visible("pose", show_pose)
        self.set_row_visible("panos", show_panos)
        self.set_row_visible("alignment", show_alignment)

        if hasattr(self, "pill_workflow"):
            self._set_pill_state(self.pill_workflow, f"● WORKFLOW: {short_wf}")

        if hasattr(self, "density_box"):
            if show_existing_pano_density:
                self.density_box.pack(fill="x", padx=16, pady=(4, 3))
            else:
                self.density_box.pack_forget()

        if hasattr(self, "priority_box") and hasattr(self, "pano_res_box"):
            if show_synthesis_controls:
                self.priority_box.pack(fill="x", padx=16, pady=(4, 3))
                self.pano_res_box.pack(fill="x", padx=16, pady=(4, 4))
            else:
                self.priority_box.pack_forget()
                self.pano_res_box.pack_forget()

    def set_row_visible(self, key, visible):
        widgets = self.input_rows.get(key)
        if not widgets:
            return
        for widget in widgets:
            if visible:
                widget.grid()
            else:
                widget.grid_remove()

    def create_section_header(self, parent, text):
        h = ctk.CTkFrame(parent, fg_color="transparent")
        h.pack(fill="x", pady=(4, 4))
        ctk.CTkFrame(
            h, width=6, height=12, corner_radius=2, fg_color=AV_THEME["gold_primary"]
        ).pack(side="left", padx=(2, 8))
        ctk.CTkLabel(
            h,
            text=text,
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            text_color=AV_THEME["gold_bright"],
        ).pack(side="left")
        ctk.CTkFrame(h, height=1, fg_color=COLORS["border_subtle"]).pack(
            side="left", fill="x", expand=True, padx=(10, 2)
        )

    def create_input_row(self, parent, row, label, var, command):
        lbl = ctk.CTkLabel(
            parent,
            text=label,
            text_color=AV_THEME["white_muted"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
        )
        lbl.grid(row=row, column=0, sticky="w", pady=4)
        entry = ctk.CTkEntry(
            parent,
            textvariable=var,
            height=32,
            corner_radius=8,
            fg_color=COLORS["console_bg"],
            border_width=1,
            border_color=COLORS["border"],
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_MONO, size=11),
        )
        entry.grid(row=row, column=1, sticky="ew", padx=12, pady=4)
        btn = ctk.CTkButton(
            parent,
            text="BROWSE",
            font=ctk.CTkFont(family=FONT_MONO, size=10, weight="bold"),
            width=82,
            height=32,
            corner_radius=8,
            fg_color=COLORS["accent_surface"],
            hover_color=COLORS["accent_btn_fill"],
            text_color=AV_THEME["gold_bright"],
            border_width=1,
            border_color=AV_THEME["gold_primary"],
            command=command,
        )
        btn.grid(row=row, column=2, pady=4)
        return (lbl, entry, btn)

    def create_slider_row(self, parent, label, var, min_v, max_v, step=1.0, is_int=False):
        f = ctk.CTkFrame(parent, fg_color="transparent")
        f.pack(fill="x", padx=16, pady=4)
        ctk.CTkLabel(
            f,
            text=label,
            width=155,
            anchor="w",
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
        ).pack(side="left")

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

        update_val(var.get())

        num_steps = max(1, int(round((max_v - min_v) / step)))
        slider = ctk.CTkSlider(
            f,
            from_=min_v,
            to=max_v,
            number_of_steps=num_steps,
            variable=var,
            command=update_val,
            fg_color=COLORS["border_subtle"],
            progress_color=AV_THEME["gold_primary"],
            button_color=AV_THEME["white_warm"],
            button_hover_color=AV_THEME["gold_bright"],
        )
        slider.pack(side="left", fill="x", expand=True, padx=10)
        ctk.CTkLabel(
            f,
            textvariable=val_label_var,
            width=52,
            text_color=AV_THEME["gold_bright"],
            font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
        ).pack(side="left")

    def create_multiplier_slider(self, parent, label, var, min_steps, max_steps, step, multiplier, is_int=True):
        """Create a Champagne Gold Apple Glass slider where integer steps are multiplied to set actual value."""
        f = ctk.CTkFrame(parent, fg_color="transparent")
        f.pack(fill="x", padx=16, pady=4)
        ctk.CTkLabel(
            f,
            text=label,
            width=155,
            anchor="w",
            text_color=AV_THEME["white_warm"],
            font=ctk.CTkFont(family=FONT_SANS, size=11, weight="bold"),
        ).pack(side="left")
        count_var = ctk.IntVar(value=int(var.get() / multiplier))
        val_label_var = ctk.StringVar()

        def update_val(count):
            actual = int(count) * multiplier
            var.set(actual)
            if float(multiplier).is_integer():
                val_label_var.set(f"{int(actual)} m")
            else:
                val_label_var.set(f"{actual:.1f} m")

        update_val(count_var.get())
        num_steps = max(1, int((max_steps - min_steps) / step))
        slider = ctk.CTkSlider(
            f,
            from_=min_steps,
            to=max_steps,
            number_of_steps=num_steps,
            variable=count_var,
            command=lambda v: update_val(v),
            fg_color=COLORS["border_subtle"],
            progress_color=AV_THEME["gold_primary"],
            button_color=AV_THEME["white_warm"],
            button_hover_color=AV_THEME["gold_bright"],
        )
        slider.pack(side="left", fill="x", expand=True, padx=10)
        ctk.CTkLabel(
            f,
            textvariable=val_label_var,
            width=56,
            text_color=AV_THEME["gold_bright"],
            font=ctk.CTkFont(family=FONT_MONO, size=11, weight="bold"),
        ).pack(side="left")

    # -------------------------------------------------------------------------
    # Project Settings & File Dialogs
    # -------------------------------------------------------------------------

    def browse_project(self):
        d = filedialog.askdirectory(title="Select Project Directory")
        if d:
            self.project_dir.set(d)
            if hasattr(self, "lbl_active_project_badge"):
                self.lbl_active_project_badge.configure(
                    text=f"PROJECT: {os.path.basename(d).upper()}"
                )
            self.load_project_settings(d)

    def get_scan_density_pct(self) -> float:
        val_str = self.scan_density_var.get()
        try:
            val_clean = val_str.split("%")[0].strip()
            return float(val_clean)
        except Exception:
            return 100.0

    def get_optimization_priority_code(self) -> str:
        val = self.optimization_priority_var.get()
        if "Geometric" in val:
            return "geometry"
        if "Panorama" in val and "Balanced" not in val:
            return "panorama"
        if "Minimum" in val:
            return "efficiency"
        return "balanced"

    def get_pano_resolution_tuple(self):
        val = self.pano_res_var.get().split()[0]
        try:
            w, h = val.lower().split("x")
            return (int(w), int(h))
        except Exception:
            return (2048, 1024)

    def load_project_settings(self, base_path):
        project_json_path = os.path.join(base_path, "project.json")

        for folder in ["input", "output", "logs", "cache", "thumbnails", "temp"]:
            os.makedirs(os.path.join(base_path, folder), exist_ok=True)

        def resolve_path(p):
            if not p:
                return ""
            if os.path.isabs(p):
                return p
            return os.path.abspath(os.path.join(base_path, p))

        if os.path.exists(project_json_path):
            try:
                with open(project_json_path, "r", encoding="utf-8") as f:
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

                    saved_density = float(data.get("scan_density_pct", 100.0))
                    matching_str = f"{int(saved_density)}%"
                    found = False
                    for opt in getattr(self, "density_options", []):
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
            self.append_log(
                f"[INFO] Initialized new project structure and default project.json at {base_path}"
            )

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

        project_json_path = os.path.join(d, "project.json")
        created_time = datetime.utcnow().isoformat() + "Z"
        if os.path.exists(project_json_path):
            try:
                with open(project_json_path, "r", encoding="utf-8") as f:
                    old_data = json.load(f)
                    created_time = old_data.get("created", created_time)
            except Exception:
                pass

        data = {
            "project_name": os.path.basename(d),
            "project_format_version": 2,
            "software_version": "2.0.0",
            "minimum_supported_version": "1.0.0",
            "created": created_time,
            "modified": datetime.utcnow().isoformat() + "Z",
            "import_workflow": self.import_workflow_var.get(),
            "scan_density_pct": self.get_scan_density_pct(),
            "optimization_priority": self.get_optimization_priority_code(),
            "pano_resolution": list(self.get_pano_resolution_tuple()),
            "input_cloud": make_relative(self.input_file.get()),
            "pose_csv": make_relative(self.pose_csv.get()),
            "panorama_folder": make_relative(self.panos_dir.get()),
            "alignment_file": make_relative(self.alignment_file.get()),
            "output_e57": make_relative(self.output_file.get()),
            "scan_spacing": self.spacing_var.get(),
            "max_range": self.max_range_var.get(),
            "station_generation": self.mode_var.get(),
            "reduce_scans": True,
            "processing_complete": False,
        }
        try:
            with open(project_json_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            self.append_log(f"[ERROR] Failed to save project.json: {e}")

    def browse_input(self):
        file = filedialog.askopenfilename(
            title="Select Point Cloud",
            filetypes=(
                ("Point Cloud files", "*.e57;*.las;*.laz;*.ply;*.pts;*.xyz;*.xyzrgb;*.pcd"),
                ("All files", "*.*"),
            ),
        )
        if file:
            self.input_file.set(file)
            self.save_project_settings()

    def browse_output(self):
        file = filedialog.asksaveasfilename(
            title="Save Structured Output",
            defaultextension=".e57",
            filetypes=(("E57 files", "*.e57"),),
        )
        if file:
            self.output_file.set(file)
            self.save_project_settings()

    def browse_pose(self):
        file = filedialog.askopenfilename(
            title="Select Pose CSV",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if file:
            self.pose_csv.set(file)
            self.save_project_settings()

    def browse_panos(self):
        d = filedialog.askdirectory(title="Select Panoramas Directory")
        if d:
            self.panos_dir.set(d)
            self.save_project_settings()

    def browse_alignment(self):
        file = filedialog.askopenfilename(
            title="Select Alignment Bundle",
            filetypes=(
                ("NavVis Bundle", "*.bundle;*.zip"),
                ("JSON/XML", "*.json;*.xml"),
                ("All files", "*.*"),
            ),
        )
        if file:
            self.alignment_file.set(file)
            self.save_project_settings()

    # -------------------------------------------------------------------------
    # Console Logging & Pipeline Execution
    # -------------------------------------------------------------------------

    def append_log(self, text):
        if not hasattr(self, "textbox"):
            print(text)
            return
        self.textbox.configure(state="normal")
        self.textbox.insert("end", text + "\n")
        self.textbox.see("end")
        self.textbox.configure(state="disabled")

        # Update live telemetry stage bar when stage messages appear
        if hasattr(self, "lbl_stage_status") and hasattr(self, "pipeline_progress"):
            if "Stage 1:" in text:
                self.lbl_stage_status.configure(text="STAGE 1  ·  Loading Point Cloud & Sensor Metadata...")
                self.pipeline_progress.set(0.20)
            elif "Stage 2:" in text:
                self.lbl_stage_status.configure(text="STAGE 2  ·  Generating & Optimizing Virtual Scan Stations...")
                self.pipeline_progress.set(0.45)
            elif "Stage 3:" in text:
                self.lbl_stage_status.configure(text="STAGE 3  ·  Spherical Ray Partitioning & 2D Grid Indexing...")
                self.pipeline_progress.set(0.70)
            elif "Stage 4:" in text:
                self.lbl_stage_status.configure(text="STAGE 4  ·  Exporting Structured E57 & 360° Panospheres...")
                self.pipeline_progress.set(0.90)
            elif "PIPELINE COMPLETED SUCCESSFULLY" in text:
                self.lbl_stage_status.configure(text="COMPLETE  ·  Structured E57 Export & Reimport Verification Passed")
                self.pipeline_progress.set(1.0)

    def clear_log(self):
        if not hasattr(self, "textbox"):
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

    def cancel_pipeline(self):
        self.cancel_event.set()
        self.append_log("[WARN] Cancellation requested — stopping pipeline after current step...")
        self.cancel_btn.configure(state="disabled")
        if hasattr(self, "pill_engine"):
            self._set_pill_state(
                self.pill_engine,
                "● ENGINE: CANCELLING",
                text_color=AV_THEME["status_warn"],
                fg_color=COLORS["accent_surface"],
                border_color=AV_THEME["gold_primary"],
            )

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
            self.append_log(
                "[ERROR] Workflow 'E57 + Panoramas + Pose CSV' requires both Pose CSV and Panoramas Directory."
            )
            return

        self.cancel_event.clear()
        self.save_project_settings()
        self._pipeline_running = True
        self._pipeline_start_epoch = time.time()
        self.start_btn.configure(state="disabled", text="● PIPELINE ACTIVE")
        self.cancel_btn.configure(state="normal")
        if hasattr(self, "pipeline_progress"):
            self.pipeline_progress.set(0.08)
        if hasattr(self, "pill_engine"):
            self._set_pill_state(
                self.pill_engine,
                "● ENGINE: ACTIVE",
                text_color=AV_THEME["gold_bright"],
                fg_color=COLORS["accent_surface"],
                border_color=AV_THEME["gold_primary"],
            )
        if hasattr(self, "footer_status_lbl"):
            self.footer_status_lbl.configure(
                text=f"PROCESSING  |  Input: {os.path.basename(input_cloud)}  →  Output: {os.path.basename(output_path)}"
            )
        threading.Thread(target=self.run_pipeline_thread, daemon=True).start()

    def run_pipeline_thread(self):
        proj_dir = self.project_dir.get()
        job_id = None
        log_file_path = None
        output_e57 = self.output_file.get()

        # Setup Job and logging if project dir exists
        if proj_dir and os.path.exists(proj_dir):
            try:
                out_dir_base = os.path.join(proj_dir, "output")
                os.makedirs(out_dir_base, exist_ok=True)
                existing_jobs = [
                    d
                    for d in os.listdir(out_dir_base)
                    if os.path.isdir(os.path.join(out_dir_base, d)) and d.startswith("Job_")
                ]
                job_nums = []
                for job in existing_jobs:
                    try:
                        num = int(job.split("_")[1])
                        job_nums.append(num)
                    except ValueError:
                        pass
                next_num = max(job_nums) + 1 if job_nums else 1
                job_id = f"Job_{next_num:03d}"

                job_out_dir = os.path.join(out_dir_base, job_id)
                os.makedirs(job_out_dir, exist_ok=True)

                orig_filename = os.path.basename(output_e57) or "structured.e57"
                output_e57 = os.path.join(job_out_dir, orig_filename)

                log_dir = os.path.join(proj_dir, "logs")
                os.makedirs(log_dir, exist_ok=True)
                log_file_path = os.path.join(log_dir, f"{job_id}_run.log")

                self.file_handler = logging.FileHandler(log_file_path, encoding="utf-8")
                self.file_handler.setFormatter(
                    logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
                )
                self.logger.addHandler(self.file_handler)
                for m in ["io_modules", "stations", "simulation", "virtual_scanner"]:
                    logging.getLogger(m).addHandler(self.file_handler)

                self.logger.info(f"Initialized processing run: {job_id}")
                self.logger.info(f"Job Output Directory: {job_out_dir}")
            except Exception as e:
                self.logger.error(f"Failed to initialize Job directory structure: {e}")
                self._pipeline_running = False
                self.after(0, lambda: self.start_btn.configure(state="normal", text="▶ INITIALIZE PIPELINE"))
                self.after(0, lambda: self.cancel_btn.configure(state="disabled"))
                return

        wf = self.import_workflow_var.get()
        pose_csv_path = self.pose_csv.get() or None
        panos_dir_path = self.panos_dir.get() or None
        alignment_path = self.alignment_file.get() or None
        mode_pref = self.mode_var.get()

        if wf == "Structured E57 (Embedded Panoramas + Poses)":
            pose_csv_path = None
            panos_dir_path = None
        elif wf == "E57 + Bundle / Reconstruction File":
            pose_csv_path = None
            panos_dir_path = None
        elif wf == "E57 + Panoramas Directory + Pose CSV":
            alignment_path = None
        elif wf == "Generic Colorized Point Cloud (No Panoramas)":
            pose_csv_path = None
            panos_dir_path = None
            alignment_path = None
            if mode_pref == "auto":
                mode_pref = "geometry"

        bundle_dir = None
        if self.export_bundle_var.get():
            out_base = os.path.dirname(os.path.abspath(output_e57))
            bundle_dir = os.path.join(out_base, "walkthrough_bundle")

        succeeded = False
        try:
            run_pipeline(
                input_cloud_path=self.input_file.get(),
                output_e57_path=output_e57,
                pose_csv_path=pose_csv_path,
                navvis_images_dir=panos_dir_path,
                mode_pref=mode_pref,
                logger_instance=self.logger,
                alignment_file=alignment_path,
                spacing=self.spacing_var.get(),
                overlap=self.overlap_var.get(),
                coverage=self.coverage_var.get(),
                max_range=self.max_range_var.get(),
                project_dir=proj_dir,
                job_id=job_id,
                dry_run=self.dry_run_var.get(),
                scan_density_pct=self.get_scan_density_pct(),
                angular_res_deg=self.angular_res_var.get(),
                pano_resolution=self.get_pano_resolution_tuple(),
                optimization_priority=self.get_optimization_priority_code(),
                cancel_event=self.cancel_event,
                export_bundle_dir=bundle_dir,
            )
            succeeded = True
            self.logger.info("PIPELINE COMPLETED SUCCESSFULLY.")
        except Exception as e:
            self.logger.error(f"PIPELINE FAILURE: {str(e)}")
        finally:
            self._pipeline_running = False
            if log_file_path and hasattr(self, "file_handler") and self.file_handler:
                self.logger.removeHandler(self.file_handler)
                for m in ["io_modules", "stations", "simulation", "virtual_scanner"]:
                    logging.getLogger(m).removeHandler(self.file_handler)
                self.file_handler.close()
                self.file_handler = None

            def _restore_ui():
                self.start_btn.configure(state="normal", text="▶ INITIALIZE PIPELINE")
                self.cancel_btn.configure(state="disabled")
                if hasattr(self, "pill_engine"):
                    if succeeded:
                        self._set_pill_state(
                            self.pill_engine,
                            "● ENGINE: READY",
                            text_color=AV_THEME["status_ok"],
                            fg_color=COLORS["status_ok_surface"],
                            border_color=COLORS["status_ok_border"],
                        )
                    else:
                        self._set_pill_state(
                            self.pill_engine,
                            "● ENGINE: STANDBY",
                            text_color=AV_THEME["status_err"],
                            fg_color=COLORS["destructive_surface"],
                            border_color=COLORS["destructive_border"],
                        )
                if hasattr(self, "footer_status_lbl"):
                    status_prefix = "COMPLETE" if succeeded else "READY"
                    self.footer_status_lbl.configure(
                        text=f"{status_prefix}  |  VERIFY · DIRECT · CLARIFY  |  AV Structure 2.0 Apple Glass Spatial OS"
                    )

            self.after(0, _restore_ui)


if __name__ == "__main__":
    app = VirtualScannerGUI()
    app.mainloop()
