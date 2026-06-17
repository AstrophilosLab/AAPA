import customtkinter as ctk
import serial
import serial.tools.list_ports
import socket
import time
import threading
import os
import glob
import json
import math
from pathlib import Path

# --- CONFIGURAZIONE TEMA ---
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("dark-blue")

class AstroController(ctk.CTk):
    def __init__(self):
        super().__init__()

        # --- SETUP FINESTRA ---
        self.title("Astro Commander · AAPA")
        self.geometry("1180x980")
        self.configure(fg_color="#0e0e12")

        # --- PALETTE ---
        self.C_BG      = "#0e0e12"
        self.C_CARD    = "#1a1a22"
        self.C_INNER   = "#23232c"
        self.C_BORDER  = "#2a2a35"
        self.C_TEXT    = "#ececf2"
        self.C_DIM     = "#8a8a96"
        self.C_MUTED   = "#56565f"
        self.C_BLUE    = "#4a9eff"
        self.C_TEAL    = "#22d3aa"
        self.C_AMBER   = "#ff8a3d"
        self.C_PURPLE  = "#b27aff"
        self.C_GREEN   = "#22c55e"
        self.C_RED     = "#ef4444"
        self.C_NEG     = "#b94a4a"
        self.C_POS     = "#3da671"
        
        # --- VARIABILI DI STATO ---
        self.ser = None          # transport USB (pyserial)
        self.sock = None         # transport WiFi (socket TCP)
        self.is_connected = False
        self.active_mode = "USB"  # modalita' realmente in uso quando connesso
        self.polar_monitoring = False
        self.auto_aligning = False

        # Connessione: USB (seriale) oppure WiFi (TCP verso la board, porta 23).
        self.conn_mode = ctk.StringVar(value="USB")
        self.wifi_host = ctk.StringVar(value="192.168.4.1")
        self.wifi_port = ctk.IntVar(value=23)
        
        # Variabili Motori (Default Nema 17)
        self.steps_per_rev = ctk.IntVar(value=200)
        
        # ASSE X (Azimuth)
        self.x_ms = ctk.IntVar(value=16)
        self.x_run = ctk.IntVar(value=600)
        self.x_hold = ctk.IntVar(value=50)
        self.x_ratio = ctk.DoubleVar(value=1.0)
        self.x_reverse = ctk.BooleanVar(value=False)
        
        # ASSE Y (Altitude)
        self.y_ms = ctk.IntVar(value=16)
        self.y_run = ctk.IntVar(value=600)
        self.y_hold = ctk.IntVar(value=50)
        self.y_ratio = ctk.DoubleVar(value=1.0)
        self.y_reverse = ctk.BooleanVar(value=False)

        # StallGuard / Home / Limiti Y (sincronizzati con NVS della board)
        self.y_home_offset = ctk.IntVar(value=200)
        self.y_min_limit   = ctk.IntVar(value=-100000)
        self.y_max_limit   = ctk.IntVar(value=100000)
        self.y_sgthrs      = ctk.IntVar(value=80)
        self.y_homing_dir  = ctk.IntVar(value=-1)
        self.y_homing_speed = ctk.IntVar(value=800)
        self.y_position    = ctk.StringVar(value="?")
        self.y_homed_var   = ctk.StringVar(value="NOT HOMED")
        # Profilo di moto (max speed e accelerazione)
        self.x_max_speed = ctk.IntVar(value=1500)
        self.x_accel     = ctk.IntVar(value=500)
        self.y_max_speed = ctk.IntVar(value=1500)
        self.y_accel     = ctk.IntVar(value=500)
        # Flag: i parametri scrivibili vengono sincronizzati dalla board UNA volta
        # dopo la connessione. Poi il polling non li sovrascrive piu', cosi'
        # l'utente puo' digitare senza che gli venga cancellato il valore.
        self._writable_params_synced = False

        # Variabili N.I.N.A
        self.alt_error_var = ctk.StringVar(value="--° --' --\"")
        self.az_error_var = ctk.StringVar(value="--° --' --\"")
        self.last_update_var = ctk.StringVar(value="Status: Waiting for N.I.N.A...")
        
        # Dati Grezzi
        self.raw_alt = None
        self.raw_az = None
        self.last_log_timestamp = 0

        # Variabili Calibrazione & Pilot
        self.calib_status = ctk.StringVar(value="Ready")
        self.calib_result = ctk.StringVar(value="---")
        self.pilot_status = ctk.StringVar(value="Auto-Pilot Standby")

        # --- ROOT SCROLLABILE ---
        # Tutta la UI vive dentro questo frame: quando un asse si espande,
        # gli altri pannelli scendono naturalmente e si scrolla con la rotellina.
        self.scroll = ctk.CTkScrollableFrame(self, fg_color="transparent",
                                              scrollbar_button_color=self.C_BORDER,
                                              scrollbar_button_hover_color=self.C_DIM,
                                              corner_radius=0)
        self.scroll.pack(fill="both", expand=True)
        self.scroll.grid_columnconfigure(0, weight=1)
        self.scroll.grid_columnconfigure(1, weight=1)
        # Nessuna riga con weight: ogni pannello prende la sua altezza naturale
        # e gli altri vengono spinti giu' (scrollabili).

        # --- COSTRUZIONE UI ---
        self.setup_header()
        self.setup_axis_panels()
        self.setup_polar_panel()
        self.setup_calibration_panel()
        self.setup_homing_panel()
        self.setup_log_panel()

        self.refresh_ports()

        # Avvia polling automatico dello status (pos Y live, ogni 1 secondo)
        self.after(1500, self._poll_status_loop)

    def _poll_status_loop(self):
        """Polling periodico di :STATUS finche' la app e' aperta.
        Aggiorna posizione Y / homed / parametri NVS in tempo reale."""
        if self.is_connected:
            try:
                self.send_cmd(":STATUS")
            except Exception:
                pass
        self.after(1000, self._poll_status_loop)

    # --- HELPER UTILITY ---
    def decimal_to_dms(self, decimal_deg):
        if decimal_deg is None: return "--"
        is_positive = decimal_deg >= 0
        decimal_deg = abs(decimal_deg)
        d = int(decimal_deg)
        m = int((decimal_deg - d) * 60)
        s = (decimal_deg - d - m/60) * 3600
        sign = "+" if is_positive else "-"
        return f"{sign}{d}° {m:02d}' {s:04.1f}\""

    # ──────────────────────────────────────────────────────────
    # HELPER: card con header pulito (titolo + accent dot)
    # ──────────────────────────────────────────────────────────
    def _card(self, row, columnspan=2, sticky="ew", pady=(5, 5)):
        f = ctk.CTkFrame(self.scroll, fg_color=self.C_CARD, corner_radius=14,
                         border_width=1, border_color=self.C_BORDER)
        f.grid(row=row, column=0, columnspan=columnspan, padx=12, pady=pady, sticky=sticky)
        return f

    def _card_header(self, parent, title, accent):
        h = ctk.CTkFrame(parent, fg_color="transparent")
        h.pack(fill="x", padx=16, pady=(12, 6))
        dot = ctk.CTkLabel(h, text="●", text_color=accent, font=("Segoe UI", 12))
        dot.pack(side="left", padx=(0, 8))
        ctk.CTkLabel(h, text=title, font=("Segoe UI Semibold", 13),
                     text_color=self.C_TEXT).pack(side="left")
        return h

    # ──────────────────────────────────────────────────────────
    # HEADER (barra connessione)
    # ──────────────────────────────────────────────────────────
    def setup_header(self):
        self.header_frame = ctk.CTkFrame(self.scroll, fg_color=self.C_CARD, corner_radius=14,
                                          border_width=1, border_color=self.C_BORDER, height=64)
        self.header_frame.grid(row=0, column=0, columnspan=2, padx=12, pady=(12, 6), sticky="ew")
        self.header_frame.pack_propagate(False)

        # Brand a sinistra
        brand = ctk.CTkFrame(self.header_frame, fg_color="transparent")
        brand.pack(side="left", padx=20)
        ctk.CTkLabel(brand, text="ASTRO COMMANDER", font=("Segoe UI Semibold", 14),
                     text_color=self.C_TEXT).pack(anchor="w")
        ctk.CTkLabel(brand, text="AAPA · dual-axis polar alignment",
                     font=("Segoe UI", 10), text_color=self.C_DIM).pack(anchor="w")

        # Steps/rev a destra
        sr = ctk.CTkFrame(self.header_frame, fg_color="transparent")
        sr.pack(side="right", padx=16)
        ctk.CTkLabel(sr, text="STEPS/REV", font=("Segoe UI", 9, "bold"),
                     text_color=self.C_DIM).pack(anchor="e")
        ctk.CTkEntry(sr, textvariable=self.steps_per_rev, width=70, height=26,
                     fg_color=self.C_INNER, border_color=self.C_BORDER,
                     justify="center").pack()

        # Connessione al centro-destra
        conn = ctk.CTkFrame(self.header_frame, fg_color="transparent")
        conn.pack(side="right", padx=16)

        # Selettore di trasporto: USB (seriale) o WiFi (TCP).
        self.mode_seg = ctk.CTkSegmentedButton(conn, values=["USB", "WiFi"],
                                               variable=self.conn_mode,
                                               command=self._on_mode_change,
                                               width=120, height=30,
                                               fg_color=self.C_INNER,
                                               selected_color=self.C_BLUE,
                                               selected_hover_color="#3a86e0",
                                               unselected_color=self.C_INNER,
                                               unselected_hover_color=self.C_BORDER,
                                               text_color=self.C_TEXT,
                                               font=("Segoe UI Semibold", 11))
        self.mode_seg.pack(side="left", padx=(0, 8))

        # Campi USB: lista porte + refresh
        self.usb_frame = ctk.CTkFrame(conn, fg_color="transparent")
        self.port_combo = ctk.CTkComboBox(self.usb_frame, width=130, height=30,
                                          fg_color=self.C_INNER,
                                          border_color=self.C_BORDER,
                                          button_color=self.C_BORDER,
                                          dropdown_fg_color=self.C_INNER)
        self.port_combo.pack(side="left", padx=4)
        ctk.CTkButton(self.usb_frame, text="↻", width=34, height=30,
                      fg_color=self.C_INNER, hover_color=self.C_BORDER,
                      text_color=self.C_TEXT, font=("Segoe UI", 14),
                      command=self.refresh_ports).pack(side="left", padx=2)

        # Campi WiFi: host (IP) + porta TCP
        self.wifi_frame = ctk.CTkFrame(conn, fg_color="transparent")
        ctk.CTkEntry(self.wifi_frame, textvariable=self.wifi_host, width=120, height=30,
                     fg_color=self.C_INNER, border_color=self.C_BORDER,
                     justify="center").pack(side="left", padx=4)
        ctk.CTkEntry(self.wifi_frame, textvariable=self.wifi_port, width=54, height=30,
                     fg_color=self.C_INNER, border_color=self.C_BORDER,
                     justify="center").pack(side="left", padx=2)

        self.btn_connect = ctk.CTkButton(conn, text="●  Connect", width=120, height=30,
                                          fg_color=self.C_GREEN, hover_color="#16a34a",
                                          font=("Segoe UI Semibold", 11),
                                          command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=6)

        # Mostra i campi giusti per la modalita' iniziale.
        self._on_mode_change()

    def _on_mode_change(self, _=None):
        """Mostra i campi USB o WiFi a seconda della modalita' scelta.
        Bloccato mentre si e' connessi (si cambia trasporto solo da disconnessi)."""
        if self.is_connected:
            self.conn_mode.set(self.active_mode)  # ripristina, niente switch a caldo
            return
        self.usb_frame.pack_forget()
        self.wifi_frame.pack_forget()
        if self.conn_mode.get() == "WiFi":
            self.wifi_frame.pack(side="left", before=self.btn_connect)
        else:
            self.usb_frame.pack(side="left", before=self.btn_connect)

    # ──────────────────────────────────────────────────────────
    # AXIS PANELS
    # ──────────────────────────────────────────────────────────
    def setup_axis_panels(self):
        self.frame_x = ctk.CTkFrame(self.scroll, fg_color=self.C_CARD, corner_radius=14,
                                     border_width=1, border_color=self.C_BORDER)
        self.frame_x.grid(row=1, column=0, padx=(12, 6), pady=6, sticky="new")
        self.create_axis_ui(self.frame_x, "X", "AZIMUTH", self.C_BLUE,
                            'X', self.x_ms, self.x_run, self.x_hold,
                            self.x_ratio, self.x_reverse)

        self.frame_y = ctk.CTkFrame(self.scroll, fg_color=self.C_CARD, corner_radius=14,
                                     border_width=1, border_color=self.C_BORDER)
        self.frame_y.grid(row=1, column=1, padx=(6, 12), pady=6, sticky="new")
        self.create_axis_ui(self.frame_y, "Y", "ALTITUDE", self.C_TEAL,
                            'Y', self.y_ms, self.y_run, self.y_hold,
                            self.y_ratio, self.y_reverse)

    def create_axis_ui(self, parent, letter, subtitle, accent,
                       axis_char, v_ms, v_run, v_hold, v_ratio, v_rev):
        # Stato apri/chiudi (default chiuso)
        state = {"open": False}

        # ── HEADER cliccabile ──
        head = ctk.CTkFrame(parent, fg_color="transparent", cursor="hand2")
        head.pack(fill="x", padx=18, pady=(14, 14))

        big = ctk.CTkLabel(head, text=letter, font=("Segoe UI Black", 32),
                           text_color=accent, width=44, cursor="hand2")
        big.pack(side="left")

        col = ctk.CTkFrame(head, fg_color="transparent", cursor="hand2")
        col.pack(side="left", padx=4)
        lab1 = ctk.CTkLabel(col, text=f"AXIS {letter}", font=("Segoe UI Semibold", 11),
                            text_color=self.C_DIM, cursor="hand2")
        lab1.pack(anchor="w")
        lab2 = ctk.CTkLabel(col, text=subtitle, font=("Segoe UI Semibold", 16),
                            text_color=self.C_TEXT, cursor="hand2")
        lab2.pack(anchor="w")

        hint = ctk.CTkLabel(head, text="click to expand", font=("Segoe UI", 9),
                            text_color=self.C_MUTED, cursor="hand2")
        hint.pack(side="right", padx=(0, 4))
        chev = ctk.CTkLabel(head, text="▸", font=("Segoe UI", 16),
                            text_color=accent, cursor="hand2", width=20)
        chev.pack(side="right")

        # ── CONTENT (mostrato/nascosto al click) ──
        content = ctk.CTkFrame(parent, fg_color="transparent")
        # NON viene packato adesso: pack avviene in toggle()

        # Separatore sottile interno
        ctk.CTkFrame(content, fg_color=self.C_BORDER, height=1).pack(
            fill="x", padx=0, pady=(0, 12))

        # SETTINGS card
        sett = ctk.CTkFrame(content, fg_color=self.C_INNER, corner_radius=10, height=120)
        sett.pack(fill="x", padx=0, pady=(0, 12))
        sett.pack_propagate(False)

        # Riga 1 — Ratio + Reverse
        r1 = ctk.CTkFrame(sett, fg_color="transparent")
        r1.pack(fill="x", padx=14, pady=(12, 4))
        ctk.CTkLabel(r1, text="Ratio", width=80, anchor="w",
                     font=("Segoe UI", 11), text_color=self.C_DIM).pack(side="left")
        ctk.CTkEntry(r1, textvariable=v_ratio, width=90, height=28,
                     fg_color=self.C_CARD, border_color=self.C_BORDER,
                     justify="center").pack(side="left", padx=(0, 12))
        ctk.CTkSwitch(r1, text="Reverse", variable=v_rev,
                      progress_color=accent, font=("Segoe UI", 11),
                      text_color=self.C_TEXT).pack(side="left")

        # Riga 2 — Microstep + APPLY
        r2 = ctk.CTkFrame(sett, fg_color="transparent")
        r2.pack(fill="x", padx=14, pady=(4, 12))
        ctk.CTkLabel(r2, text="Microstep", width=80, anchor="w",
                     font=("Segoe UI", 11), text_color=self.C_DIM).pack(side="left")
        ctk.CTkComboBox(r2, variable=v_ms, values=["16", "32", "64"],
                        width=90, height=28,
                        fg_color=self.C_CARD, border_color=self.C_BORDER,
                        button_color=self.C_BORDER,
                        dropdown_fg_color=self.C_INNER,
                        justify="center").pack(side="left", padx=(0, 12))
        ctk.CTkButton(r2, text="APPLY", width=72, height=28,
                      fg_color=accent, hover_color=self.C_BORDER,
                      text_color="#0e0e12", font=("Segoe UI Semibold", 11),
                      command=lambda: self.send_cmd(f"S{axis_char}{v_ms.get()}")
                      ).pack(side="left")

        # QUICK MOVE
        qhead = ctk.CTkFrame(content, fg_color="transparent")
        qhead.pack(fill="x", padx=0, pady=(2, 6))
        ctk.CTkLabel(qhead, text="QUICK MOVE", font=("Segoe UI Semibold", 9),
                     text_color=self.C_DIM).pack(side="left")

        btn_grid = ctk.CTkFrame(content, fg_color="transparent")
        btn_grid.pack(pady=(0, 8))
        btns = [-360, -90, -10, -1, 1, 10, 90, 360]
        for i, val in enumerate(btns):
            color = self.C_NEG if val < 0 else self.C_POS
            hover = "#d65555" if val < 0 else "#2f8f5d"
            ctk.CTkButton(btn_grid, text=f"{val:+d}°".replace("+", "+ "),
                          width=66, height=36,
                          fg_color=color, hover_color=hover,
                          text_color="#fff",
                          font=("Segoe UI Semibold", 11),
                          corner_radius=8,
                          command=lambda v=val: self.move(axis_char, v)
                          ).grid(row=0 if i < 4 else 1,
                                 column=i if i < 4 else i - 4,
                                 padx=4, pady=4)

        # ── Toggle handler ──
        def toggle(event=None):
            if state["open"]:
                content.pack_forget()
                chev.configure(text="▸")
                hint.configure(text="click to expand")
                state["open"] = False
            else:
                content.pack(fill="x", padx=18, pady=(0, 14))
                chev.configure(text="▾")
                hint.configure(text="click to collapse")
                state["open"] = True

        # Bind click su tutti gli elementi dell'header
        for w in (head, big, col, lab1, lab2, hint, chev):
            w.bind("<Button-1>", toggle)

    # ──────────────────────────────────────────────────────────
    # AUTO-PILOT PANEL (N.I.N.A)
    # ──────────────────────────────────────────────────────────
    def setup_polar_panel(self):
        self.polar_frame = self._card(row=2)

        # Header: titolo + monitor switch
        h = ctk.CTkFrame(self.polar_frame, fg_color="transparent")
        h.pack(fill="x", padx=18, pady=(14, 4))
        ctk.CTkLabel(h, text="●", text_color=self.C_PURPLE,
                     font=("Segoe UI", 12)).pack(side="left", padx=(0, 8))
        ctk.CTkLabel(h, text="N.I.N.A AUTO-PILOT",
                     font=("Segoe UI Semibold", 13),
                     text_color=self.C_TEXT).pack(side="left")
        ctk.CTkLabel(h, text="Plate-solve driven alignment",
                     font=("Segoe UI", 10), text_color=self.C_DIM).pack(side="left", padx=10)
        self.btn_polar = ctk.CTkSwitch(h, text="Monitor",
                                        command=self.toggle_polar_monitor,
                                        progress_color=self.C_PURPLE,
                                        font=("Segoe UI", 11),
                                        text_color=self.C_TEXT)
        self.btn_polar.pack(side="right")

        # Box errori grandi
        errors = ctk.CTkFrame(self.polar_frame, fg_color="transparent")
        errors.pack(fill="x", padx=18, pady=8)
        errors.grid_columnconfigure(0, weight=1)
        errors.grid_columnconfigure(1, weight=1)

        self.create_error_box(errors, 0, "ALTITUDE ERROR", self.alt_error_var, self.C_TEAL)
        self.create_error_box(errors, 1, "AZIMUTH ERROR",  self.az_error_var,  self.C_AMBER)

        # Pulsanti azione
        ctrl = ctk.CTkFrame(self.polar_frame, fg_color="transparent")
        ctrl.pack(pady=(8, 6))
        ctk.CTkButton(ctrl, text="Align ALT", width=120, height=36,
                      fg_color=self.C_INNER, hover_color=self.C_BORDER,
                      text_color=self.C_TEXT, font=("Segoe UI", 11),
                      command=lambda: self.start_auto_pilot('Y')).pack(side="left", padx=6)

        self.btn_dual_pilot = ctk.CTkButton(ctrl, text="▶  AUTO ALIGN DUAL",
                                             width=240, height=42,
                                             fg_color=self.C_PURPLE,
                                             hover_color="#9a5fee",
                                             text_color="#0e0e12",
                                             font=("Segoe UI Semibold", 13),
                                             corner_radius=10,
                                             command=self.start_dual_pilot)
        self.btn_dual_pilot.pack(side="left", padx=10)

        ctk.CTkButton(ctrl, text="Align AZ", width=120, height=36,
                      fg_color=self.C_INNER, hover_color=self.C_BORDER,
                      text_color=self.C_TEXT, font=("Segoe UI", 11),
                      command=lambda: self.start_auto_pilot('X')).pack(side="left", padx=6)

        # Status + Stop
        foot = ctk.CTkFrame(self.polar_frame, fg_color="transparent")
        foot.pack(fill="x", padx=18, pady=(2, 14))
        ctk.CTkLabel(foot, textvariable=self.pilot_status,
                     font=("Consolas", 11),
                     text_color=self.C_AMBER, anchor="w").pack(side="left")
        ctk.CTkButton(foot, text="■  STOP", width=90, height=28,
                      fg_color=self.C_RED, hover_color="#dc2626",
                      text_color="#fff", font=("Segoe UI Semibold", 10),
                      command=self.stop_auto_pilot).pack(side="right")

    def create_error_box(self, parent, col, title, variable, accent):
        f = ctk.CTkFrame(parent, fg_color=self.C_INNER, corner_radius=10)
        f.grid(row=0, column=col, padx=6, sticky="ew")
        ctk.CTkLabel(f, text=title, font=("Segoe UI Semibold", 9),
                     text_color=self.C_DIM).pack(pady=(10, 2))
        ctk.CTkLabel(f, textvariable=variable, font=("Consolas", 22, "bold"),
                     text_color=accent).pack(pady=(0, 10))

    # ──────────────────────────────────────────────────────────
    # CALIBRATION PANEL
    # ──────────────────────────────────────────────────────────
    def setup_calibration_panel(self):
        self.calib_frame = self._card(row=3)

        # Header
        h = ctk.CTkFrame(self.calib_frame, fg_color="transparent")
        h.pack(fill="x", padx=18, pady=(14, 8))
        ctk.CTkLabel(h, text="●", text_color="#facc15",
                     font=("Segoe UI", 12)).pack(side="left", padx=(0, 8))
        ctk.CTkLabel(h, text="RATIO CALIBRATION",
                     font=("Segoe UI Semibold", 13),
                     text_color=self.C_TEXT).pack(side="left")
        ctk.CTkLabel(h, text="rota 360° motore · misura su cielo",
                     font=("Segoe UI", 10), text_color=self.C_DIM).pack(side="left", padx=10)

        # Riga unica: axis selector · start · status · result · apply
        row = ctk.CTkFrame(self.calib_frame, fg_color="transparent")
        row.pack(fill="x", padx=18, pady=(0, 14))

        self.calib_axis_combo = ctk.CTkComboBox(row,
                                                 values=["X (Azimuth)", "Y (Altitude)"],
                                                 width=140, height=30,
                                                 fg_color=self.C_INNER,
                                                 border_color=self.C_BORDER,
                                                 button_color=self.C_BORDER,
                                                 dropdown_fg_color=self.C_INNER)
        self.calib_axis_combo.pack(side="left", padx=(0, 8))

        ctk.CTkButton(row, text="START 360°", width=120, height=30,
                      fg_color="#facc15", hover_color="#eab308",
                      text_color="#0e0e12", font=("Segoe UI Semibold", 11),
                      command=self.start_calibration).pack(side="left", padx=4)

        ctk.CTkLabel(row, textvariable=self.calib_status,
                     font=("Consolas", 11), text_color=self.C_BLUE,
                     width=160, anchor="w").pack(side="left", padx=10)

        ctk.CTkLabel(row, text="RESULT", font=("Segoe UI Semibold", 9),
                     text_color=self.C_DIM).pack(side="left", padx=(10, 6))
        ctk.CTkLabel(row, textvariable=self.calib_result,
                     font=("Consolas", 14, "bold"),
                     text_color=self.C_GREEN, width=80).pack(side="left")

        ctk.CTkButton(row, text="APPLY", width=80, height=30,
                      fg_color=self.C_INNER, hover_color=self.C_BORDER,
                      text_color=self.C_TEXT, font=("Segoe UI", 11),
                      command=self.apply_calibration).pack(side="right")

    # ──────────────────────────────────────────────────────────
    # Y STALLGUARD / HOMING / SOFT LIMITS PANEL
    # ──────────────────────────────────────────────────────────
    def setup_homing_panel(self):
        self.homing_frame = self._card(row=4)

        # Header: dot + titolo + posizione live + status + HOME/Refresh
        h = ctk.CTkFrame(self.homing_frame, fg_color="transparent")
        h.pack(fill="x", padx=18, pady=(14, 6))
        ctk.CTkLabel(h, text="●", text_color=self.C_AMBER,
                     font=("Segoe UI", 12)).pack(side="left", padx=(0, 8))
        ctk.CTkLabel(h, text="Y · STALLGUARD & SOFT LIMITS",
                     font=("Segoe UI Semibold", 13),
                     text_color=self.C_TEXT).pack(side="left")

        # Posizione live al centro
        live = ctk.CTkFrame(h, fg_color="transparent")
        live.pack(side="left", padx=20)
        ctk.CTkLabel(live, text="POS", font=("Segoe UI Semibold", 9),
                     text_color=self.C_DIM).pack(side="left", padx=(0, 4))
        ctk.CTkLabel(live, textvariable=self.y_position,
                     font=("Consolas", 13, "bold"), text_color=self.C_BLUE,
                     width=70).pack(side="left")
        ctk.CTkLabel(live, textvariable=self.y_homed_var,
                     font=("Consolas", 10, "bold"),
                     text_color=self.C_AMBER, width=90).pack(side="left", padx=8)

        # Pulsanti header
        ctk.CTkButton(h, text="↻", width=34, height=30,
                      fg_color=self.C_INNER, hover_color=self.C_BORDER,
                      text_color=self.C_TEXT, font=("Segoe UI", 14),
                      command=self.query_status).pack(side="right", padx=3)
        ctk.CTkButton(h, text="HOME Y", width=110, height=30,
                      fg_color=self.C_AMBER, hover_color="#e67c2e",
                      text_color="#0e0e12", font=("Segoe UI Semibold", 11),
                      command=self.home_y).pack(side="right", padx=3)

        # Separatore
        sep = ctk.CTkFrame(self.homing_frame, fg_color=self.C_BORDER, height=1)
        sep.pack(fill="x", padx=18, pady=(2, 10))

        # Griglia 3 colonne x 2 righe di parametri
        grid = ctk.CTkFrame(self.homing_frame, fg_color="transparent")
        grid.pack(fill="x", padx=14, pady=4)
        for c in range(3):
            grid.grid_columnconfigure(c, weight=1)

        def cell(row, col, label, var, cmd_prefix):
            f = ctk.CTkFrame(grid, fg_color=self.C_INNER, corner_radius=10)
            f.grid(row=row, column=col, padx=4, pady=4, sticky="ew")
            ctk.CTkLabel(f, text=label, font=("Segoe UI Semibold", 9),
                         text_color=self.C_DIM).pack(anchor="w", padx=12, pady=(8, 0))
            inner = ctk.CTkFrame(f, fg_color="transparent")
            inner.pack(fill="x", padx=8, pady=(2, 8))
            ctk.CTkEntry(inner, textvariable=var, width=100, height=26,
                         fg_color=self.C_CARD, border_color=self.C_BORDER,
                         justify="center").pack(side="left", padx=2)
            ctk.CTkButton(inner, text="SET", width=50, height=26,
                          fg_color=self.C_BORDER, hover_color=self.C_AMBER,
                          text_color=self.C_TEXT,
                          font=("Segoe UI Semibold", 10),
                          command=lambda: self.send_cmd(f"{cmd_prefix} {var.get()}")
                          ).pack(side="left", padx=4)

        cell(0, 0, "HOME OFFSET (step)",      self.y_home_offset,  ":OFFY")
        cell(0, 1, "MIN LIMIT (step)",        self.y_min_limit,    ":MINY")
        cell(0, 2, "MAX LIMIT (step)",        self.y_max_limit,    ":MAXY")
        cell(1, 0, "SGTHRS (0-255)",          self.y_sgthrs,       ":SGY")
        cell(1, 1, "HOMING DIR (-1 / 1)",     self.y_homing_dir,   ":DIRY")
        cell(1, 2, "HOMING SPEED (step/s)",   self.y_homing_speed, ":HSPY")
        # Profilo di moto motori (movimenti normali). Il motore vero sente questi.
        cell(2, 0, "Y MAX SPEED (step/s)",    self.y_max_speed,    ":SPDY")
        cell(2, 1, "Y ACCEL (step/s²)",       self.y_accel,        ":ACCY")
        cell(2, 2, "X MAX SPEED (step/s)",    self.x_max_speed,    ":SPDX")
        cell(3, 0, "X ACCEL (step/s²)",       self.x_accel,        ":ACCX")

        # Azioni in basso
        actions = ctk.CTkFrame(self.homing_frame, fg_color="transparent")
        actions.pack(fill="x", padx=18, pady=(8, 14))

        ctk.CTkButton(actions, text="Save to board",  width=120, height=30,
                      fg_color=self.C_GREEN, hover_color="#16a34a",
                      text_color="#0e0e12", font=("Segoe UI Semibold", 11),
                      command=lambda: self.send_cmd(":SAVE")).pack(side="left", padx=3)
        ctk.CTkButton(actions, text="Set Y=0",  width=90, height=30,
                      fg_color=self.C_INNER, hover_color=self.C_BORDER,
                      text_color=self.C_TEXT, font=("Segoe UI", 11),
                      command=lambda: self.send_cmd(":RESETY")).pack(side="left", padx=3)
        ctk.CTkButton(actions, text="Diagnose SG", width=120, height=30,
                      fg_color=self.C_INNER, hover_color=self.C_PURPLE,
                      text_color=self.C_TEXT, font=("Segoe UI", 11),
                      command=lambda: self.send_cmd(":SGTEST")).pack(side="left", padx=3)
        ctk.CTkButton(actions, text="Read DIAG", width=100, height=30,
                      fg_color=self.C_INNER, hover_color=self.C_BORDER,
                      text_color=self.C_TEXT, font=("Segoe UI", 11),
                      command=lambda: self.send_cmd(":DREAD")).pack(side="left", padx=3)
        ctk.CTkButton(actions, text="■ STOP", width=90, height=30,
                      fg_color=self.C_RED, hover_color="#dc2626",
                      text_color="#fff", font=("Segoe UI Semibold", 11),
                      command=lambda: self.send_cmd(":STOP")).pack(side="right", padx=3)

    def home_y(self):
        if not self.is_connected:
            self.log("Not connected. Cannot home.")
            return
        self.log("Starting Y homing (StallGuard)...")
        self.send_cmd(":HOMEY")

    def query_status(self):
        """Forza una sync da NVS della board (sovrascrive i campi UI)."""
        if not self.is_connected: return
        self._writable_params_synced = False  # consente al prossimo STATUS di aggiornare i writable
        self.send_cmd(":STATUS")

    # ──────────────────────────────────────────────────────────
    # LOG PANEL
    # ──────────────────────────────────────────────────────────
    def setup_log_panel(self):
        self.log_frame = ctk.CTkFrame(self.scroll, fg_color=self.C_CARD, corner_radius=14,
                                       border_width=1, border_color=self.C_BORDER,
                                       height=160)
        self.log_frame.grid(row=5, column=0, columnspan=2, padx=12, pady=(6, 12), sticky="ew")
        self.log_frame.pack_propagate(False)

        h = ctk.CTkFrame(self.log_frame, fg_color="transparent")
        h.pack(fill="x", padx=18, pady=(10, 4))
        ctk.CTkLabel(h, text="●", text_color=self.C_DIM,
                     font=("Segoe UI", 12)).pack(side="left", padx=(0, 8))
        ctk.CTkLabel(h, text="LOG", font=("Segoe UI Semibold", 11),
                     text_color=self.C_DIM).pack(side="left")

        self.log_text = ctk.CTkTextbox(self.log_frame, font=("Consolas", 11),
                                        text_color=self.C_GREEN,
                                        fg_color=self.C_INNER,
                                        border_width=0, corner_radius=10)
        self.log_text.pack(fill="both", expand=True, padx=14, pady=(0, 12))
        self.log_text.configure(state="disabled")

    # ================= LOGICA PILOTA =================

    def stop_auto_pilot(self):
        self.auto_aligning = False
        self.pilot_status.set("PILOT ABORTED BY USER")
        self.log("AUTO PILOT STOPPED.")

    # --- START DUAL PILOT (NUOVO) ---
    def start_dual_pilot(self):
        if not self.is_connected or not self.polar_monitoring:
            self.pilot_status.set("Error: Connect & Enable Monitor First")
            return
        if self.raw_alt is None or self.raw_az is None: 
            self.pilot_status.set("Error: No NINA Data yet")
            return
            
        self.auto_aligning = True
        self.pilot_status.set(f"DUAL PILOT ENGAGED (ALT + AZ)")
        threading.Thread(target=self.dual_pilot_loop, daemon=True).start()

    # --- LOOP DUAL PILOT (NUOVO ALGORITMO) ---
    def dual_pilot_loop(self):
        target_error_deg = 0.005 # ~18 arcsec (Soglia di successo)
        gain = 0.8 
        max_attempts = 20
        attempt = 0
        
        self.log(f"--- STARTING DUAL ALIGNMENT ---")

        while self.auto_aligning and attempt < max_attempts:
            # 1. Lettura errori correnti
            curr_az = self.raw_az
            curr_alt = self.raw_alt
            
            # 2. Check Successo Completo
            az_ok = abs(curr_az) < target_error_deg
            alt_ok = abs(curr_alt) < target_error_deg
            
            if az_ok and alt_ok:
                self.pilot_status.set(f"SUCCESS! BOTH AXES ALIGNED!")
                self.log(f"FINAL: AZ={curr_az:.5f}° | ALT={curr_alt:.5f}°")
                break

            self.pilot_status.set(f"Iter {attempt+1}: Moving motors...")
            
            # 3. Calcolo Correzioni (Indipendenti)
            # Muoviamo l'asse solo se è fuori target
            if not az_ok:
                corr_az = -(curr_az * gain)
                self.log(f"Iter {attempt+1} [AZ]: Err {curr_az:.4f} -> Move {corr_az:.4f}")
                self.move('X', corr_az)
            
            if not alt_ok:
                corr_alt = -(curr_alt * gain)
                self.log(f"Iter {attempt+1} [ALT]: Err {curr_alt:.4f} -> Move {corr_alt:.4f}")
                self.move('Y', corr_alt)

            # 4. SETTLING TIME CONDIVISO (15s)
            # Aspettiamo una volta sola per entrambi i motori
            for i in range(15, 0, -1):
                if not self.auto_aligning: return 
                self.pilot_status.set(f"Settling... {i}s")
                time.sleep(1)

            # 5. Attesa Log NINA (Singola attesa per nuova foto)
            self.pilot_status.set(f"Waiting N.I.N.A image analysis...")
            start_ts = self.last_log_timestamp
            wait_timer = 0
            while self.last_log_timestamp == start_ts:
                time.sleep(1)
                wait_timer += 1
                if not self.auto_aligning: return 
                if wait_timer > 120: # Timeout aumentato per platesolving
                    self.pilot_status.set("Error: N.I.N.A Timeout")
                    return
            
            # 6. Check Direzione (Logic Reversal Check)
            # Qui è complesso fare check direzione su due assi, lo facciamo basilare
            new_az = self.raw_az
            new_alt = self.raw_alt
            
            # Se l'errore è aumentato drasticamente su un asse che abbiamo mosso, invertiamo
            if not az_ok and abs(new_az) > abs(curr_az) + 0.01:
                self.log("WARNING: AZ Error Increased! Reversing X logic...")
                self.x_reverse.set(not self.x_reverse.get())

            if not alt_ok and abs(new_alt) > abs(curr_alt) + 0.01:
                self.log("WARNING: ALT Error Increased! Reversing Y logic...")
                self.y_reverse.set(not self.y_reverse.get())
            
            attempt += 1

        self.auto_aligning = False
        if attempt >= max_attempts:
             self.pilot_status.set("Failed: Max attempts reached")

    # --- LOOP PILOTA SINGOLO (LEGACY/BACKUP) ---
    def start_auto_pilot(self, axis):
        if not self.is_connected or not self.polar_monitoring:
            self.pilot_status.set("Error: Connect & Enable Monitor First")
            return
        if self.raw_alt is None: 
            self.pilot_status.set("Error: No NINA Data yet")
            return
            
        self.auto_aligning = True
        self.pilot_status.set(f"PILOT ENGAGED: Aligning {axis}...")
        threading.Thread(target=self.auto_pilot_loop, args=(axis,), daemon=True).start()

    def auto_pilot_loop(self, axis):
        target_error_deg = 0.005 
        gain = 0.8 
        max_attempts = 15
        attempt = 0
        
        self.log(f"--- STARTING SINGLE ALIGN {axis} ---")

        while self.auto_aligning and attempt < max_attempts:
            current_error = self.raw_az if axis == 'X' else self.raw_alt
            
            if abs(current_error) < target_error_deg:
                self.pilot_status.set(f"SUCCESS! Error < {target_error_deg*60:.1f}'")
                break
            
            correction_deg = -(current_error * gain)
            self.pilot_status.set(f"Iter {attempt+1}: Err={current_error:.4f}°. Moving...")
            self.log(f"Pilot Iter {attempt+1}: Error {current_error:.4f}. Fix {correction_deg:.4f}")
            
            self.move(axis, correction_deg)
            
            for i in range(15, 0, -1):
                if not self.auto_aligning: return 
                self.pilot_status.set(f"Settling... {i}s")
                time.sleep(1)

            self.pilot_status.set(f"Waiting N.I.N.A update...")
            start_ts = self.last_log_timestamp
            wait_timer = 0
            while self.last_log_timestamp == start_ts:
                time.sleep(1)
                wait_timer += 1
                if not self.auto_aligning: return 
                if wait_timer > 90:
                    self.pilot_status.set("Error: N.I.N.A Timeout")
                    return
            
            new_error = self.raw_az if axis == 'X' else self.raw_alt
            if abs(new_error) > abs(current_error):
                self.log("WARNING: Error Increased! Reversing Logic...")
                if axis == 'X': self.x_reverse.set(not self.x_reverse.get())
                else: self.y_reverse.set(not self.y_reverse.get())
            
            attempt += 1
        self.auto_aligning = False

    # --- CALIBRATION LOGIC ---
    def start_calibration(self):
        if not self.is_connected or not self.polar_monitoring:
            self.calib_status.set("Error: Connect & Enable NINA first")
            return
        sel = self.calib_axis_combo.get()
        axis = 'X' if "X" in sel else 'Y'
        threading.Thread(target=self.calib_thread, args=(axis,), daemon=True).start()

    def calib_thread(self, axis):
        self.calib_status.set(f"1. Reading Start {axis}...")
        start_val = self.raw_az if axis == 'X' else self.raw_alt
        if start_val is None:
            self.calib_status.set("Err: No NINA Data")
            return

        self.calib_status.set("2. Moving 360 Motor Deg...")
        spr = self.steps_per_rev.get()
        ms = self.x_ms.get() if axis == 'X' else self.y_ms.get()
        steps = spr * ms 
        self.send_cmd(f"{axis}{steps}")
        
        for i in range(20, 0, -1):
            self.calib_status.set(f"Settling... {i}s")
            time.sleep(1)

        self.calib_status.set("Waiting NINA Log...")
        ts = self.last_log_timestamp
        e = 0
        while self.last_log_timestamp == ts:
            time.sleep(1); e+=1
            if e>90: 
                self.calib_status.set("Timeout NINA"); 
                return
            
        end_val = self.raw_az if axis == 'X' else self.raw_alt
        delta = abs(end_val - start_val)
        
        if delta < 0.01:
            self.calib_status.set("Err: Move < 0.01 (Noise)")
            self.log("CALIB: Sky didn't move. Check Clutch/Current.")
            return

        ratio = 360.0 / delta
        self.calib_result.set(f"{ratio:.4f}")
        self.calib_status.set("Success!")
        self.log(f"Calib: Delta={delta:.4f}°. Ratio={ratio:.4f}")

    def apply_calibration(self):
        try:
            r = float(self.calib_result.get())
            if "X" in self.calib_axis_combo.get(): self.x_ratio.set(r)
            else: self.y_ratio.set(r)
            self.log(f"Ratio {r:.4f} applied.")
        except Exception as e:
            self.log(f"Error during applying calibration: {e}")

    # --- MOVIMENTO ---
    def move(self, axis, degrees):
        try:
            spr = self.steps_per_rev.get()
            ms = self.x_ms.get() if axis == 'X' else self.y_ms.get()
            ratio = self.x_ratio.get() if axis == 'X' else self.y_ratio.get()
            rev = self.x_reverse.get() if axis == 'X' else self.y_reverse.get()
            
            if rev: degrees = -degrees
            steps = int((degrees / 360.0) * spr * ms * ratio)
            
            self.send_cmd(f"{axis}{steps}")
            self.log(f"Move {axis} {degrees}° ({steps} steps)")
        except Exception as e:
            self.log(f"Error during move: {e}")

    # --- NINA MONITOR ---
    def toggle_polar_monitor(self):
        if self.btn_polar.get() == 1:
            self.polar_monitoring = True
            threading.Thread(target=self.nina_monitor_loop, daemon=True).start()
        else:
            self.polar_monitoring = False

    def nina_monitor_loop(self):
        path = Path(os.path.expanduser("~")) / "Documents" / "N.I.N.A" / "PolarAlignment"
        while self.polar_monitoring:
            try:
                if not path.exists(): time.sleep(5); continue
                files = [f for f in path.glob('*') if f.is_file()]
                if not files: time.sleep(5); continue
                latest = max(files, key=os.path.getctime)
                ts = os.path.getmtime(latest)
                if ts != self.last_log_timestamp:
                    self.parse_nina(latest); self.last_log_timestamp = ts
            except Exception as e:
                self.log(f"Error in NINA monitor loop: {e}")
            time.sleep(1)

    def parse_nina(self, fpath):
        try:
            with open(fpath, 'r', encoding='utf-8') as f: lines = f.readlines()
            for line in reversed(lines):
                if "AltitudeError" in line and "{" in line:
                    data = json.loads(line.split(" - ", 1)[1].strip())
                    self.raw_alt = data.get("AltitudeError", 0)
                    self.raw_az = data.get("AzimuthError", 0)
                    self.alt_error_var.set(self.decimal_to_dms(self.raw_alt))
                    self.az_error_var.set(self.decimal_to_dms(self.raw_az))
                    self.last_update_var.set(f"Upd: {time.strftime('%H:%M:%S')}")
                    return
        except Exception as e: 
            self.log(f"Error during parsing NINA log: {e}")
            pass

    # --- UTILS ---
    def log(self, msg):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"> {msg}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def refresh_ports(self):
        self.port_combo.configure(values=[p.device for p in serial.tools.list_ports.comports()])
        
    def toggle_connection(self):
        if not self.is_connected:
            mode = self.conn_mode.get()
            try:
                if mode == "WiFi":
                    host = self.wifi_host.get().strip()
                    port = int(self.wifi_port.get())
                    # connect con timeout, poi timeout breve per il loop di lettura
                    self.sock = socket.create_connection((host, port), timeout=5)
                    self.sock.settimeout(0.2)
                    self.log(f"Connected via WiFi ({host}:{port}).")
                else:
                    self.ser = serial.Serial(self.port_combo.get(), 115200, timeout=1)
                    self.log(f"Connected via USB ({self.port_combo.get()}).")
                self.is_connected = True
                self.active_mode = mode
                self.btn_connect.configure(fg_color=self.C_RED, hover_color="#dc2626",
                                            text="●  Disconnect")
                threading.Thread(target=self.read_loop, daemon=True).start()
                # Forza una sincronizzazione iniziale dei parametri dalla board
                self._writable_params_synced = False
                self.after(500, self.query_status)
            except Exception as e:
                self.log(f"Error during connection: {e}")
                self._close_transport()
                self.is_connected = False
        else:
            self.is_connected = False
            self._close_transport()
            self._writable_params_synced = False  # alla prossima connessione, risync
            self.btn_connect.configure(fg_color=self.C_GREEN, hover_color="#16a34a",
                                        text="●  Connect")

    def _close_transport(self):
        """Chiude e azzera entrambi i trasporti, ignorando errori."""
        try:
            if self.ser: self.ser.close()
        except Exception:
            pass
        try:
            if self.sock: self.sock.close()
        except Exception:
            pass
        self.ser = None
        self.sock = None

    def read_loop(self):
        """Loop di lettura unico per USB e WiFi: estrae righe terminate da '\\n'
        e le inoltra a handle_rx() sul thread della UI."""
        buf = ""
        while self.is_connected:
            try:
                if self.sock is not None:
                    try:
                        data = self.sock.recv(256)
                    except socket.timeout:
                        continue
                    if not data:           # peer ha chiuso la connessione
                        self.after(0, lambda: self.log("WiFi: connection closed by board."))
                        break
                    buf += data.decode(errors="ignore")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self.after(0, lambda m=line: self.handle_rx(m))
                elif self.ser is not None:
                    if self.ser.in_waiting:
                        l = self.ser.readline().decode(errors="ignore").strip()
                        if l: self.after(0, lambda m=l: self.handle_rx(m))
                    else:
                        time.sleep(0.02)
                else:
                    break
            except Exception as e:
                self.after(0, lambda m=str(e): self.log(f"Error during reading: {m}"))
                break

    def handle_rx(self, line):
        self.log(f"RX: {line}")
        # Parsing risposta :STATUS (es. "POSX:0 POSY:1234 BUSYY:0 HOMED:1 OFFY:200 MINY:-1000 MAXY:1000 SGY:80 ...")
        if "POSY:" in line and "OFFY:" in line:
            try:
                for tok in line.split():
                    if ":" not in tok: continue
                    k, v = tok.split(":", 1)
                    if   k == "POSY":  self.y_position.set(v)
                    elif k == "HOMED": self.y_homed_var.set("HOMED" if v == "1" else "NOT HOMED")
                    # Parametri scrivibili: aggiornati SOLO al primo poll dopo la connessione,
                    # poi lasciati intatti per non cancellare quello che l'utente digita.
                    elif not self._writable_params_synced:
                        if   k == "OFFY":  self.y_home_offset.set(int(v))
                        elif k == "MINY":  self.y_min_limit.set(int(v))
                        elif k == "MAXY":  self.y_max_limit.set(int(v))
                        elif k == "SGY":   self.y_sgthrs.set(int(v))
                        elif k == "DIRY":  self.y_homing_dir.set(int(v))
                        elif k == "HSPY":  self.y_homing_speed.set(int(v))
                        elif k == "SPDX":  self.x_max_speed.set(int(v))
                        elif k == "ACCX":  self.x_accel.set(int(v))
                        elif k == "SPDY":  self.y_max_speed.set(int(v))
                        elif k == "ACCY":  self.y_accel.set(int(v))
                        # MICROSTEPS: critico! Senza questo Python e firmware sono
                        # disallineati e i comandi di movimento producono distanze
                        # diverse da quelle attese.
                        elif k == "MSX":   self.x_ms.set(int(v))
                        elif k == "MSY":   self.y_ms.set(int(v))
                # Dopo aver processato un'intera risposta STATUS, marca come sincronizzato
                self._writable_params_synced = True
            except Exception as e:
                self.log(f"Status parse error: {e}")
            
    def send_cmd(self, c):
        if not self.is_connected:
            self.log("Not connected. Cmd not sent.")
            return
        try:
            payload = (c + "\n").encode()
            if self.sock is not None:
                self.sock.sendall(payload)
            elif self.ser is not None:
                self.ser.write(payload)
        except Exception as e:
            self.log(f"Send error: {e}")

if __name__ == "__main__":
    app = AstroController()
    app.mainloop()
