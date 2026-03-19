import math
import tkinter as tk
from tkinter import ttk, messagebox


APP_TITLE = "Cinemática directa e inversa"
ROBOT = {
    "L1": 24.0,
    "a1": 15.0,
    "a2": 15.0,
    "a3": 15.0,
}
DEFAULTS = {
    "theta1": -10.01,
    "theta2": -77.10,
    "theta3": 99.00,
    "x": 31.0,
    "y": -5.61,
    "z": 14.0,
}


class KinematicsError(Exception):
    pass


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def clamp_unit(value: float) -> float:
    return clamp(value, -1.0, 1.0)


def robot_constants():
    return ROBOT["L1"], ROBOT["a1"], ROBOT["a2"], ROBOT["a3"]


def max_reach() -> float:
    _, a1, a2, a3 = robot_constants()
    return a1 + a2 + a3


def forward_kinematics(theta1_deg: float, theta2_deg: float, theta3_deg: float):
    L1, a1, a2, a3 = robot_constants()

    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    t3 = math.radians(theta3_deg)

    x = math.cos(t1) * (a1 + a2 * math.cos(t2) + a3 * math.cos(t2 + t3))
    y = math.sin(t1) * (a1 + a2 * math.cos(t2) + a3 * math.cos(t2 + t3))
    z = L1 + a2 * math.sin(t2) + a3 * math.sin(t2 + t3)

    return x, y, z


def inverse_kinematics(px: float, py: float, pz: float):
    L1, a1, a2, a3 = robot_constants()

    theta1 = math.atan2(py, px)
    px_prima = math.hypot(px, py)
    R = math.hypot(pz - L1, px_prima - a1)

    if a2 <= 0 or a3 <= 0:
        raise KinematicsError("a2 y a3 deben ser mayores que cero.")

    if R < 1e-9:
        raise KinematicsError("El punto cae en una singularidad para esta formulación.")

    if R > (a2 + a3) + 1e-9 or R < abs(a2 - a3) - 1e-9:
        raise KinematicsError("El punto ingresado no es alcanzable para el robot.")

    alpha = math.atan2((pz - L1), (px_prima - a1))
    cos_beta = ((R ** 2) + (a2 ** 2) - (a3 ** 2)) / (2 * a2 * R)
    cos_gamma = ((a2 ** 2) + (a3 ** 2) - (R ** 2)) / (2 * a2 * a3)

    beta = math.acos(clamp_unit(cos_beta))
    gamma = math.acos(clamp_unit(cos_gamma))

    theta2 = alpha - beta
    theta3 = math.pi - gamma

    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)


def robot_points(theta1_deg: float, theta2_deg: float, theta3_deg: float):
    L1, a1, a2, a3 = robot_constants()

    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    t3 = math.radians(theta3_deg)

    base = (0.0, 0.0, 0.0)
    p0 = (0.0, 0.0, L1)
    p1 = (a1 * math.cos(t1), a1 * math.sin(t1), L1)
    p2 = (
        math.cos(t1) * (a1 + a2 * math.cos(t2)),
        math.sin(t1) * (a1 + a2 * math.cos(t2)),
        L1 + a2 * math.sin(t2),
    )
    p3 = (
        math.cos(t1) * (a1 + a2 * math.cos(t2) + a3 * math.cos(t2 + t3)),
        math.sin(t1) * (a1 + a2 * math.cos(t2) + a3 * math.cos(t2 + t3)),
        L1 + a2 * math.sin(t2) + a3 * math.sin(t2 + t3),
    )
    return [base, p0, p1, p2, p3]


def point_is_reachable(x: float, y: float, z: float) -> bool:
    L1, a1, a2, a3 = robot_constants()
    rho = math.hypot(x, y)
    R = math.hypot(z - L1, rho - a1)
    return abs(a2 - a3) <= R <= (a2 + a3)


def clamp_target_top(x: float, y: float, z: float, fallback_phi: float = 0.0):
    """Mantiene Z fijo y ajusta solo el radio XY a la zona alcanzable."""
    L1, a1, a2, a3 = robot_constants()
    rho = math.hypot(x, y)
    phi = math.atan2(y, x) if rho > 1e-9 else fallback_phi

    vertical = z - L1
    if abs(vertical) >= (a2 + a3):
        vertical = clamp(vertical, -(a2 + a3) + 1e-6, (a2 + a3) - 1e-6)
        z = L1 + vertical

    span = math.sqrt(max(0.0, (a2 + a3) ** 2 - vertical ** 2))
    rho_min = max(0.0, a1 - span)
    rho_max = a1 + span

    rho = clamp(rho, rho_min, rho_max)
    return rho * math.cos(phi), rho * math.sin(phi), z


def clamp_target_side(rho: float, z: float):
    """Ajusta el punto R-Z a la circunferencia alcanzable del brazo."""
    L1, a1, a2, a3 = robot_constants()

    rho = max(0.0, rho)
    dx = rho - a1
    dz = z - L1
    radius = math.hypot(dx, dz)
    max_radius = a2 + a3
    min_radius = abs(a2 - a3)

    if radius < 1e-6:
        angle = -math.pi / 2
        radius = max(min_radius, 1e-6)
        dx = radius * math.cos(angle)
        dz = radius * math.sin(angle)
    else:
        angle = math.atan2(dz, dx)
        radius = clamp(radius, max(min_radius, 1e-6), max_radius)
        dx = radius * math.cos(angle)
        dz = radius * math.sin(angle)

    rho = max(0.0, a1 + dx)
    z = L1 + dz
    return rho, z


class StylishPlot(ttk.LabelFrame):
    def __init__(self, parent, title: str, interactive: bool = False,
                 on_top_drag=None, on_side_drag=None):
        super().__init__(parent, text=title, padding=10)
        self.on_top_drag = on_top_drag
        self.on_side_drag = on_side_drag
        self.interactive = interactive

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        self.top_canvas = tk.Canvas(
            self, width=460, height=340, bg="#fbfcff",
            highlightthickness=1, highlightbackground="#d9e2f2"
        )
        self.side_canvas = tk.Canvas(
            self, width=460, height=340, bg="#fbfcff",
            highlightthickness=1, highlightbackground="#d9e2f2"
        )

        self.top_canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.side_canvas.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        self.status = ttk.Label(
            self,
            text="",
            foreground="#47617f",
            justify="left",
        )
        self.status.grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))

        self.top_to_world = None
        self.side_to_world = None
        self.top_target_screen = None
        self.side_target_screen = None
        self._drag_mode = None

        if interactive:
            self.top_canvas.bind("<ButtonPress-1>", self._on_press_top)
            self.top_canvas.bind("<B1-Motion>", self._on_drag_top)
            self.top_canvas.bind("<ButtonRelease-1>", self._on_release)
            self.side_canvas.bind("<ButtonPress-1>", self._on_press_side)
            self.side_canvas.bind("<B1-Motion>", self._on_drag_side)
            self.side_canvas.bind("<ButtonRelease-1>", self._on_release)

    @staticmethod
    def _near_target(screen_target, x, y, radius=14):
        if screen_target is None:
            return False
        tx, ty = screen_target
        return (tx - x) ** 2 + (ty - y) ** 2 <= radius ** 2

    def _on_press_top(self, event):
        if self._near_target(self.top_target_screen, event.x, event.y):
            self._drag_mode = "top"

    def _on_press_side(self, event):
        if self._near_target(self.side_target_screen, event.x, event.y):
            self._drag_mode = "side"

    def _on_drag_top(self, event):
        if self._drag_mode != "top" or self.top_to_world is None or self.on_top_drag is None:
            return
        x, y = self.top_to_world(event.x, event.y)
        self.on_top_drag(x, y)

    def _on_drag_side(self, event):
        if self._drag_mode != "side" or self.side_to_world is None or self.on_side_drag is None:
            return
        rho, z = self.side_to_world(event.x, event.y)
        self.on_side_drag(rho, z)

    def _on_release(self, _event):
        self._drag_mode = None

    def draw(self, points, interactive_hint=False):
        self._draw_top(points, interactive_hint=interactive_hint)
        self._draw_side(points, interactive_hint=interactive_hint)

    def _make_transform(self, canvas, bounds):
        width = int(canvas["width"])
        height = int(canvas["height"])
        margin = 42

        min_x, max_x, min_y, max_y = bounds
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

        def to_screen(x, y):
            sx = margin + (x - min_x) * scale
            sy = height - margin - (y - min_y) * scale
            return sx, sy

        def to_world(sx, sy):
            x = (sx - margin) / scale + min_x
            y = (height - margin - sy) / scale + min_y
            return x, y

        return to_screen, to_world, width, height, margin

    def _draw_grid(self, canvas, bounds, to_screen, x_step=5, y_step=5):
        min_x, max_x, min_y, max_y = bounds

        start_x = int(math.floor(min_x / x_step) * x_step)
        end_x = int(math.ceil(max_x / x_step) * x_step)
        start_y = int(math.floor(min_y / y_step) * y_step)
        end_y = int(math.ceil(max_y / y_step) * y_step)

        for x in range(start_x, end_x + 1, x_step):
            sx1, sy1 = to_screen(x, min_y)
            sx2, sy2 = to_screen(x, max_y)
            canvas.create_line(sx1, sy1, sx2, sy2, fill="#edf1f7")
        for y in range(start_y, end_y + 1, y_step):
            sx1, sy1 = to_screen(min_x, y)
            sx2, sy2 = to_screen(max_x, y)
            canvas.create_line(sx1, sy1, sx2, sy2, fill="#edf1f7")

        if min_x <= 0 <= max_x:
            sx1, sy1 = to_screen(0, min_y)
            sx2, sy2 = to_screen(0, max_y)
            canvas.create_line(sx1, sy1, sx2, sy2, fill="#b5bfd0", width=2)
        if min_y <= 0 <= max_y:
            sx1, sy1 = to_screen(min_x, 0)
            sx2, sy2 = to_screen(max_x, 0)
            canvas.create_line(sx1, sy1, sx2, sy2, fill="#b5bfd0", width=2)

    def _draw_workspace_top(self, canvas, to_screen):
        radius = max_reach()
        x1, y1 = to_screen(-radius, radius)
        x2, y2 = to_screen(radius, -radius)
        canvas.create_oval(x1, y1, x2, y2, outline="#b9d6f2", dash=(6, 4), width=2)

    def _draw_workspace_side(self, canvas, to_screen):
        L1, a1, a2, a3 = robot_constants()
        radius = a2 + a3
        x1, y1 = to_screen(a1 - radius, L1 + radius)
        x2, y2 = to_screen(a1 + radius, L1 - radius)
        canvas.create_oval(x1, y1, x2, y2, outline="#b9d6f2", dash=(6, 4), width=2)

    def _draw_links(self, canvas, screen_points):
        shadow_points = []
        for sx, sy in screen_points:
            shadow_points.extend([sx + 2, sy + 2])
        canvas.create_line(*shadow_points, fill="#d5deea", width=8, smooth=True, capstyle=tk.ROUND)
        flat_points = []
        for sx, sy in screen_points:
            flat_points.extend([sx, sy])
        canvas.create_line(*flat_points, fill="#4d7cfe", width=6, smooth=True, capstyle=tk.ROUND)

    def _draw_joints(self, canvas, screen_points, names):
        for idx, ((sx, sy), name) in enumerate(zip(screen_points, names)):
            radius = 7 if idx < len(screen_points) - 1 else 9
            fill = "#ffffff" if idx < len(screen_points) - 1 else "#ffb347"
            outline = "#3b5da8" if idx < len(screen_points) - 1 else "#cc7a00"
            canvas.create_oval(
                sx - radius, sy - radius, sx + radius, sy + radius,
                fill=fill, outline=outline, width=2
            )
            canvas.create_text(
                sx + 12, sy - 12,
                text=name,
                anchor="w",
                fill="#2b3952",
                font=("Segoe UI", 9, "bold")
            )

    def _draw_label_box(self, canvas, width, title, subtitle):
        canvas.create_rectangle(14, 12, 210, 48, fill="#ffffff", outline="#e0e6f0")
        canvas.create_text(24, 18, text=title, anchor="nw", fill="#22324d", font=("Segoe UI", 10, "bold"))
        canvas.create_text(24, 34, text=subtitle, anchor="nw", fill="#5c6d89", font=("Segoe UI", 9))

    def _draw_top(self, points, interactive_hint=False):
        canvas = self.top_canvas
        canvas.delete("all")
        R = max_reach() + 6
        bounds = (-R, R, -R, R)
        to_screen, to_world, width, height, margin = self._make_transform(canvas, bounds)
        self.top_to_world = to_world

        self._draw_grid(canvas, bounds, to_screen, x_step=5, y_step=5)
        self._draw_workspace_top(canvas, to_screen)

        projected = [(x, y) for x, y, _ in points]
        screen_points = [to_screen(x, y) for x, y in projected]

        self._draw_links(canvas, screen_points)
        self._draw_joints(canvas, screen_points, ["B", "J0", "J1", "J2", "EF"])

        self.top_target_screen = screen_points[-1]
        sx, sy = screen_points[-1]
        canvas.create_oval(sx - 14, sy - 14, sx + 14, sy + 14, outline="#ffb347", width=2, dash=(4, 2))

        canvas.create_text(width - margin, height - 22, text="X (cm)", anchor="e", fill="#5c6d89")
        canvas.create_text(margin + 4, 16, text="Y (cm)", anchor="nw", fill="#5c6d89")

        subtitle = "Arrastra EF para mover X-Y" if interactive_hint else "Proyección del robot"
        self._draw_label_box(canvas, width, "Vista superior", subtitle)

    def _draw_side(self, points, interactive_hint=False):
        canvas = self.side_canvas
        canvas.delete("all")
        L1, _, a2, a3 = robot_constants()
        x_max = max_reach() + 6
        y_min = L1 - (a2 + a3) - 8
        y_max = L1 + (a2 + a3) + 8
        bounds = (0.0, x_max, y_min, y_max)
        to_screen, to_world, width, height, margin = self._make_transform(canvas, bounds)
        self.side_to_world = to_world

        self._draw_grid(canvas, bounds, to_screen, x_step=5, y_step=5)
        self._draw_workspace_side(canvas, to_screen)

        projected = [(math.hypot(x, y), z) for x, y, z in points]
        screen_points = [to_screen(rho, z) for rho, z in projected]

        self._draw_links(canvas, screen_points)
        self._draw_joints(canvas, screen_points, ["B", "J0", "J1", "J2", "EF"])

        self.side_target_screen = screen_points[-1]
        sx, sy = screen_points[-1]
        canvas.create_oval(sx - 14, sy - 14, sx + 14, sy + 14, outline="#ffb347", width=2, dash=(4, 2))

        canvas.create_text(width - margin, height - 22, text="R (cm)", anchor="e", fill="#5c6d89")
        canvas.create_text(margin + 4, 16, text="Z (cm)", anchor="nw", fill="#5c6d89")

        subtitle = "Arrastra EF para mover R-Z" if interactive_hint else "Proyección del robot"
        self._draw_label_box(canvas, width, "Vista lateral", subtitle)

    def set_status(self, text: str):
        self.status.configure(text=text)


class KinematicsApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1120x820")
        self.minsize(1020, 760)
        self.configure(bg="#f4f7fb", padx=14, pady=14)

        self._current_phi = math.atan2(DEFAULTS["y"], DEFAULTS["x"]) if DEFAULTS["x"] or DEFAULTS["y"] else 0.0
        self._syncing_direct = False

        self._setup_style()
        self._build_ui()

        self.load_direct_defaults()
        self.load_inverse_defaults()

    def _setup_style(self):
        style = ttk.Style(self)
        for theme_name in ("vista", "clam", "default"):
            if theme_name in style.theme_names():
                style.theme_use(theme_name)
                break

        style.configure("Title.TLabel", font=("Segoe UI", 16, "bold"), foreground="#22324d")
        style.configure("Sub.TLabel", font=("Segoe UI", 10), foreground="#5c6d89")
        style.configure("Card.TLabelframe", background="#f4f7fb")
        style.configure("Card.TLabelframe.Label", font=("Segoe UI", 10, "bold"), foreground="#22324d")
        style.configure("Value.TLabel", font=("Segoe UI", 12, "bold"), foreground="#1d3252")
        style.configure("Accent.TButton", font=("Segoe UI", 10, "bold"))

    @staticmethod
    def _read_float(var, field_name):
        value = var.get().strip().replace(",", ".")
        try:
            return float(value)
        except ValueError as exc:
            raise KinematicsError(f"Valor inválido en '{field_name}'.") from exc

    def _build_ui(self):
        header = ttk.Frame(self)
        header.pack(fill="x", pady=(0, 12))

        ttk.Label(header, text="Cinemática directa e inversa del manipulador", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            header,
            text=(
                "Medidas en centímetros y ángulos en grados. "
                "En directa puedes mover los ángulos con deslizadores. "
                "En inversa puedes arrastrar el efector final y el punto siempre se ajusta a una zona alcanzable."
            ),
            style="Sub.TLabel",
            wraplength=1050,
            justify="left",
        ).pack(anchor="w", pady=(4, 0))

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True)

        self.direct_tab = ttk.Frame(notebook, padding=8)
        self.inverse_tab = ttk.Frame(notebook, padding=8)

        notebook.add(self.direct_tab, text="Cinemática directa")
        notebook.add(self.inverse_tab, text="Cinemática inversa")

        self._build_direct_tab()
        self._build_inverse_tab()

    def _make_entry(self, parent, row, label, default_value):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=6)
        var = tk.StringVar(value=str(default_value))
        entry = ttk.Entry(parent, textvariable=var, width=12)
        entry.grid(row=row, column=1, sticky="w", pady=6)
        return var, entry

    def _build_direct_tab(self):
        controls = ttk.Frame(self.direct_tab)
        controls.pack(fill="x", pady=(0, 10))

        left = ttk.LabelFrame(controls, text="Ángulos", padding=12, style="Card.TLabelframe")
        left.pack(side="left", fill="y")

        self.dir_theta1_var, self.dir_theta1_entry = self._make_entry(left, 0, "θ1 (°):", DEFAULTS["theta1"])
        self.dir_theta2_var, self.dir_theta2_entry = self._make_entry(left, 2, "θ2 (°):", DEFAULTS["theta2"])
        self.dir_theta3_var, self.dir_theta3_entry = self._make_entry(left, 4, "θ3 (°):", DEFAULTS["theta3"])

        self.dir_scale1 = self._make_slider(left, 1, DEFAULTS["theta1"], self._on_direct_slider)
        self.dir_scale2 = self._make_slider(left, 3, DEFAULTS["theta2"], self._on_direct_slider)
        self.dir_scale3 = self._make_slider(left, 5, DEFAULTS["theta3"], self._on_direct_slider)

        buttons = ttk.Frame(left)
        buttons.grid(row=6, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Button(buttons, text="Calcular", style="Accent.TButton", command=self.calculate_direct).pack(side="left")
        ttk.Button(buttons, text="Restablecer", command=self.load_direct_defaults).pack(side="left", padx=(8, 0))

        for entry in (self.dir_theta1_entry, self.dir_theta2_entry, self.dir_theta3_entry):
            entry.bind("<Return>", lambda _e: self.calculate_direct())

        results = ttk.LabelFrame(controls, text="Posición encontrada", padding=12, style="Card.TLabelframe")
        results.pack(side="left", fill="both", expand=True, padx=(10, 0))

        self.out_x = tk.StringVar(value="X =")
        self.out_y = tk.StringVar(value="Y =")
        self.out_z = tk.StringVar(value="Z =")

        ttk.Label(results, textvariable=self.out_x, style="Value.TLabel").pack(anchor="w", pady=4)
        ttk.Label(results, textvariable=self.out_y, style="Value.TLabel").pack(anchor="w", pady=4)
        ttk.Label(results, textvariable=self.out_z, style="Value.TLabel").pack(anchor="w", pady=4)

        ttk.Label(
            results,
            text="Mueve los deslizadores para ver el brazo actualizarse en tiempo real.",
            style="Sub.TLabel",
            wraplength=420,
            justify="left",
        ).pack(anchor="w", pady=(8, 0))

        self.direct_plot = StylishPlot(self.direct_tab, "Visualización", interactive=False)
        self.direct_plot.pack(fill="both", expand=True)

    def _make_slider(self, parent, row, value, callback):
        scale = ttk.Scale(parent, from_=-180, to=180, orient="horizontal", length=280, command=callback)
        scale.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        scale.set(value)
        return scale

    def _build_inverse_tab(self):
        controls = ttk.Frame(self.inverse_tab)
        controls.pack(fill="x", pady=(0, 10))

        left = ttk.LabelFrame(controls, text="Coordenadas del efector final", padding=12, style="Card.TLabelframe")
        left.pack(side="left", fill="y")

        self.inv_x_var, self.inv_x_entry = self._make_entry(left, 0, "X (cm):", DEFAULTS["x"])
        self.inv_y_var, self.inv_y_entry = self._make_entry(left, 1, "Y (cm):", DEFAULTS["y"])
        self.inv_z_var, self.inv_z_entry = self._make_entry(left, 2, "Z (cm):", DEFAULTS["z"])

        actions = ttk.Frame(left)
        actions.grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Button(actions, text="Calcular", style="Accent.TButton", command=self.calculate_inverse).pack(side="left")
        ttk.Button(actions, text="Restablecer", command=self.load_inverse_defaults).pack(side="left", padx=(8, 0))

        for entry in (self.inv_x_entry, self.inv_y_entry, self.inv_z_entry):
            entry.bind("<Return>", lambda _e: self.calculate_inverse())

        results = ttk.LabelFrame(controls, text="Ángulos encontrados", padding=12, style="Card.TLabelframe")
        results.pack(side="left", fill="both", expand=True, padx=(10, 0))

        self.out_t1 = tk.StringVar(value="θ1 =")
        self.out_t2 = tk.StringVar(value="θ2 =")
        self.out_t3 = tk.StringVar(value="θ3 =")

        ttk.Label(results, textvariable=self.out_t1, style="Value.TLabel").pack(anchor="w", pady=4)
        ttk.Label(results, textvariable=self.out_t2, style="Value.TLabel").pack(anchor="w", pady=4)
        ttk.Label(results, textvariable=self.out_t3, style="Value.TLabel").pack(anchor="w", pady=4)

        ttk.Label(
            results,
            text=(
                "Arrastra el efector final EF en cualquiera de las dos vistas. "
                "El punto se corrige automáticamente para permanecer en una zona alcanzable."
            ),
            style="Sub.TLabel",
            wraplength=440,
            justify="left",
        ).pack(anchor="w", pady=(8, 0))

        self.inverse_plot = StylishPlot(
            self.inverse_tab,
            "Visualización interactiva",
            interactive=True,
            on_top_drag=self._handle_inverse_drag_top,
            on_side_drag=self._handle_inverse_drag_side,
        )
        self.inverse_plot.pack(fill="both", expand=True)

    def _on_direct_slider(self, _value):
        if self._syncing_direct:
            return
        self._syncing_direct = True
        try:
            self.dir_theta1_var.set(f"{self.dir_scale1.get():.2f}")
            self.dir_theta2_var.set(f"{self.dir_scale2.get():.2f}")
            self.dir_theta3_var.set(f"{self.dir_scale3.get():.2f}")
            self.calculate_direct(show_errors=False)
        finally:
            self._syncing_direct = False

    def load_direct_defaults(self):
        self.dir_theta1_var.set(str(DEFAULTS["theta1"]))
        self.dir_theta2_var.set(str(DEFAULTS["theta2"]))
        self.dir_theta3_var.set(str(DEFAULTS["theta3"]))
        self.dir_scale1.set(DEFAULTS["theta1"])
        self.dir_scale2.set(DEFAULTS["theta2"])
        self.dir_scale3.set(DEFAULTS["theta3"])
        self.calculate_direct(show_errors=False)

    def load_inverse_defaults(self):
        self.inv_x_var.set(str(DEFAULTS["x"]))
        self.inv_y_var.set(str(DEFAULTS["y"]))
        self.inv_z_var.set(str(DEFAULTS["z"]))
        if DEFAULTS["x"] or DEFAULTS["y"]:
            self._current_phi = math.atan2(DEFAULTS["y"], DEFAULTS["x"])
        self.calculate_inverse(show_errors=False)

    def calculate_direct(self, show_errors=True):
        try:
            theta1 = self._read_float(self.dir_theta1_var, "θ1")
            theta2 = self._read_float(self.dir_theta2_var, "θ2")
            theta3 = self._read_float(self.dir_theta3_var, "θ3")

            self.dir_scale1.set(theta1)
            self.dir_scale2.set(theta2)
            self.dir_scale3.set(theta3)

            x, y, z = forward_kinematics(theta1, theta2, theta3)
            points = robot_points(theta1, theta2, theta3)

            self.out_x.set(f"X = {x:.3f} cm")
            self.out_y.set(f"Y = {y:.3f} cm")
            self.out_z.set(f"Z = {z:.3f} cm")
            self.direct_plot.draw(points, interactive_hint=False)
            self.direct_plot.set_status(
                f"EF = ({x:.2f}, {y:.2f}, {z:.2f}) cm"
            )
        except KinematicsError as err:
            if show_errors:
                messagebox.showerror("Error", str(err))
        except Exception as err:
            if show_errors:
                messagebox.showerror("Error", f"No se pudo calcular la cinemática directa.\n{err}")

    def calculate_inverse(self, show_errors=True):
        try:
            x = self._read_float(self.inv_x_var, "X")
            y = self._read_float(self.inv_y_var, "Y")
            z = self._read_float(self.inv_z_var, "Z")

            t1, t2, t3 = inverse_kinematics(x, y, z)
            points = robot_points(t1, t2, t3)

            if x or y:
                self._current_phi = math.atan2(y, x)

            self.out_t1.set(f"θ1 = {t1:.3f}°")
            self.out_t2.set(f"θ2 = {t2:.3f}°")
            self.out_t3.set(f"θ3 = {t3:.3f}°")
            self.inverse_plot.draw(points, interactive_hint=True)
            self.inverse_plot.set_status(
                f"Arrastra EF. Punto actual: ({x:.2f}, {y:.2f}, {z:.2f}) cm"
            )
        except KinematicsError as err:
            if show_errors:
                messagebox.showerror("Error", str(err))
        except Exception as err:
            if show_errors:
                messagebox.showerror("Error", f"No se pudo calcular la cinemática inversa.\n{err}")

    def _update_inverse_from_xyz(self, x: float, y: float, z: float):
        self.inv_x_var.set(f"{x:.3f}")
        self.inv_y_var.set(f"{y:.3f}")
        self.inv_z_var.set(f"{z:.3f}")
        self.calculate_inverse(show_errors=False)

    def _handle_inverse_drag_top(self, x: float, y: float):
        current_z = self._read_float(self.inv_z_var, "Z")
        if x or y:
            self._current_phi = math.atan2(y, x)
        x, y, z = clamp_target_top(x, y, current_z, self._current_phi)
        self._update_inverse_from_xyz(x, y, z)

    def _handle_inverse_drag_side(self, rho: float, z: float):
        rho, z = clamp_target_side(rho, z)
        phi = self._current_phi
        x = rho * math.cos(phi)
        y = rho * math.sin(phi)
        self._update_inverse_from_xyz(x, y, z)


if __name__ == "__main__":
    app = KinematicsApp()
    app.mainloop()