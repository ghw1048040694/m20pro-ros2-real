import argparse
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except ImportError:
    class PackageNotFoundError(Exception):
        pass

    def get_package_share_directory(_package_name: str) -> str:
        raise PackageNotFoundError()


FREE_VALUE = 254
OBSTACLE_VALUE = 0


@dataclass
class MapMetadata:
    image: str
    resolution: float
    origin: str
    negate: str
    occupied_thresh: str
    free_thresh: str


def read_occ_yaml(yaml_path: str) -> MapMetadata:
    data: Dict[str, str] = {}
    with open(yaml_path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip()
    return MapMetadata(
        image=data["image"],
        resolution=float(data["resolution"]),
        origin=data["origin"],
        negate=data["negate"],
        occupied_thresh=data["occupied_thresh"],
        free_thresh=data["free_thresh"],
    )


def write_occ_yaml(yaml_path: str, metadata: MapMetadata, image_name: str) -> None:
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"image: {image_name}\n")
        f.write(f"resolution: {metadata.resolution}\n")
        f.write(f"origin: {metadata.origin}\n")
        f.write(f"negate: {metadata.negate}\n")
        f.write(f"occupied_thresh: {metadata.occupied_thresh}\n")
        f.write(f"free_thresh: {metadata.free_thresh}\n")


def resolve_image_path(yaml_path: str, image_value: str) -> str:
    image_value = image_value.strip().strip("\"'")
    yaml_dir = os.path.dirname(yaml_path)
    if os.path.isabs(image_value):
        if os.path.exists(image_value):
            return image_value
        local_image = os.path.join(yaml_dir, os.path.basename(image_value))
        if os.path.exists(local_image):
            return local_image
        return image_value
    return os.path.normpath(os.path.join(yaml_dir, image_value))


def read_pgm(path: str) -> Tuple[int, int, int, List[List[int]]]:
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError("Only binary P5 PGM files are supported")

        tokens: List[bytes] = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                break
            stripped = line.strip()
            if not stripped or stripped.startswith(b"#"):
                continue
            tokens.extend(stripped.split())
        if len(tokens) < 3:
            raise ValueError("Invalid PGM header")

        width = int(tokens[0])
        height = int(tokens[1])
        max_value = int(tokens[2])
        raw = f.read(width * height)
        if len(raw) != width * height:
            raise ValueError("PGM pixel data is incomplete")

    rows = [list(raw[y * width:(y + 1) * width]) for y in range(height)]
    return width, height, max_value, rows


def write_pgm(path: str, width: int, height: int, max_value: int, rows: List[List[int]]) -> None:
    with open(path, "wb") as f:
        f.write(f"P5\n{width} {height}\n{max_value}\n".encode("ascii"))
        for row in rows:
            f.write(bytes(row))


class MapEditorApp:
    def __init__(self, root: tk.Tk, yaml_path: str):
        self.root = root
        self.yaml_path = os.path.abspath(yaml_path)
        self.metadata = read_occ_yaml(self.yaml_path)
        self.image_path = resolve_image_path(self.yaml_path, self.metadata.image)
        self.width, self.height, self.max_value, self.grid = read_pgm(self.image_path)

        self.scale = 2
        self.brush_radius = tk.IntVar(value=2)
        self.mode = tk.StringVar(value="obstacle")
        self.status = tk.StringVar(value="")

        self.base_photo: Optional[tk.PhotoImage] = None
        self.photo: Optional[tk.PhotoImage] = None
        self.canvas_image_id: Optional[int] = None
        self._dragging = False

        self.root.title("M20 Pro Map Editor")
        self._build_ui()
        self._render()
        self._set_status(f"Loaded {os.path.basename(self.yaml_path)} ({self.width}x{self.height})")

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self.root, padding=8)
        toolbar.pack(fill="x")

        ttk.Button(toolbar, text="Open Map", command=self._open_map).pack(side="left", padx=4)
        ttk.Button(toolbar, text="Save As", command=self._save_as).pack(side="left", padx=4)

        ttk.Label(toolbar, text="Mode").pack(side="left", padx=(16, 4))
        for label, value in (("Black", "obstacle"), ("White", "free")):
            ttk.Radiobutton(toolbar, text=label, variable=self.mode, value=value).pack(side="left")

        ttk.Label(toolbar, text="Brush").pack(side="left", padx=(16, 4))
        ttk.Spinbox(toolbar, from_=1, to=20, textvariable=self.brush_radius, width=4).pack(side="left")

        ttk.Label(toolbar, text="Zoom").pack(side="left", padx=(16, 4))
        ttk.Button(toolbar, text="-", width=3, command=self._zoom_out).pack(side="left")
        ttk.Button(toolbar, text="+", width=3, command=self._zoom_in).pack(side="left", padx=(4, 0))
        self.zoom_label = ttk.Label(toolbar, text=self._zoom_text(), width=6, anchor="center")
        self.zoom_label.pack(side="left", padx=(6, 0))

        frame = ttk.Frame(self.root)
        frame.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(frame, background="#202020")
        hbar = ttk.Scrollbar(frame, orient="horizontal", command=self.canvas.xview)
        vbar = ttk.Scrollbar(frame, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)

        self.canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Button-4>", self._on_mousewheel)
        self.canvas.bind("<Button-5>", self._on_mousewheel)

        status_bar = ttk.Label(self.root, textvariable=self.status, anchor="w", padding=8)
        status_bar.pack(fill="x")

    def _render(self) -> None:
        scale = self.scale
        self.base_photo = tk.PhotoImage(width=self.width, height=self.height)
        for y, row in enumerate(self.grid):
            self.base_photo.put(self._row_colors(row), to=(0, y))
        if scale == 1:
            self.photo = self.base_photo
        else:
            self.photo = self.base_photo.zoom(scale, scale)

        if self.canvas_image_id is None:
            self.canvas_image_id = self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
        else:
            self.canvas.itemconfig(self.canvas_image_id, image=self.photo)
        self.canvas.configure(scrollregion=(0, 0, self.width * scale, self.height * scale))
        self.zoom_label.configure(text=self._zoom_text())

    def _open_map(self) -> None:
        selected = filedialog.askopenfilename(
            title="Select occ_grid.yaml",
            filetypes=[("Occupancy YAML", "*.yaml"), ("Occupancy YAML", "*.yml")],
            initialdir=os.path.dirname(self.yaml_path),
        )
        if not selected:
            return
        self.yaml_path = os.path.abspath(selected)
        self.metadata = read_occ_yaml(self.yaml_path)
        self.image_path = resolve_image_path(self.yaml_path, self.metadata.image)
        self.width, self.height, self.max_value, self.grid = read_pgm(self.image_path)
        self._render()
        self._set_status(f"Loaded {os.path.basename(self.yaml_path)} ({self.width}x{self.height})")

    def _save_as(self) -> None:
        default_name = os.path.basename(os.path.dirname(self.yaml_path)) + "_edited"
        target_dir = filedialog.askdirectory(
            title="Select output directory",
            initialdir=os.path.dirname(os.path.dirname(self.yaml_path)),
            mustexist=False,
        )
        if not target_dir:
            return
        save_dir = os.path.join(target_dir, default_name)
        os.makedirs(save_dir, exist_ok=True)
        image_name = "occ_grid.pgm"
        pgm_path = os.path.join(save_dir, image_name)
        yaml_path = os.path.join(save_dir, "occ_grid.yaml")
        write_pgm(pgm_path, self.width, self.height, self.max_value, self.grid)
        write_occ_yaml(yaml_path, self.metadata, image_name)
        self._set_status(f"Saved edited map to {save_dir}")
        messagebox.showinfo("Map saved", f"Edited map saved to:\n{save_dir}")

    def _on_press(self, event: tk.Event) -> None:
        self._dragging = True
        self._paint_at(event.x, event.y)

    def _on_drag(self, event: tk.Event) -> None:
        if self._dragging:
            self._paint_at(event.x, event.y)

    def _on_release(self, _event: tk.Event) -> None:
        self._dragging = False

    def _on_mousewheel(self, event: tk.Event) -> None:
        if getattr(event, "delta", 0) > 0 or getattr(event, "num", None) == 4:
            self._zoom_in()
        elif getattr(event, "delta", 0) < 0 or getattr(event, "num", None) == 5:
            self._zoom_out()

    def _paint_at(self, canvas_x: int, canvas_y: int) -> None:
        scale = self.scale
        x = int(self.canvas.canvasx(canvas_x) / scale)
        y = int(self.canvas.canvasy(canvas_y) / scale)
        if not (0 <= x < self.width and 0 <= y < self.height):
            return
        value = {
            "free": FREE_VALUE,
            "obstacle": OBSTACLE_VALUE,
        }[self.mode.get()]
        changed_cells: List[Tuple[int, int]] = []
        radius = self.brush_radius.get()
        for py in range(max(0, y - radius), min(self.height, y + radius + 1)):
            for px in range(max(0, x - radius), min(self.width, x + radius + 1)):
                if (px - x) * (px - x) + (py - y) * (py - y) > radius * radius:
                    continue
                if self.grid[py][px] != value:
                    self.grid[py][px] = value
                    changed_cells.append((px, py))
        if changed_cells:
            self._update_pixels(changed_cells, value)
            self._set_status(f"Edited cell around ({x}, {y}) in {self.mode.get()} mode")

    def _update_pixels(self, cells: List[Tuple[int, int]], value: int) -> None:
        if self.photo is None:
            return
        scale = self.scale
        color = self._pixel_color(value)
        cells_by_row: Dict[int, List[int]] = {}
        for x, y in cells:
            cells_by_row.setdefault(y, []).append(x)

        for y, xs in cells_by_row.items():
            xs.sort()
            run_start = xs[0]
            previous = xs[0]
            for x in xs[1:]:
                if x == previous + 1:
                    previous = x
                    continue
                self._fill_display_run(run_start, previous, y, scale, color)
                run_start = x
                previous = x
            self._fill_display_run(run_start, previous, y, scale, color)

    def _fill_display_run(self, x0: int, x1: int, y: int, scale: int, color: str) -> None:
        assert self.photo is not None
        self.photo.put(
            color,
            to=(x0 * scale, y * scale, (x1 + 1) * scale, (y + 1) * scale),
        )

    def _zoom_in(self) -> None:
        if self.scale < 8:
            self.scale += 1
            self._render()

    def _zoom_out(self) -> None:
        if self.scale > 1:
            self.scale -= 1
            self._render()

    def _zoom_text(self) -> str:
        return f"{self.scale}x"

    @staticmethod
    def _pixel_color(value: int) -> str:
        if value <= 50:
            return "#000000"
        if value >= 250:
            return "#ffffff"
        return "#999999"

    @classmethod
    def _row_colors(cls, row: List[int]) -> str:
        return "{" + " ".join(cls._pixel_color(value) for value in row) + "}"

    def _set_status(self, message: str) -> None:
        self.status.set(message)


def default_maps_dir() -> str:
    try:
        bringup_share = get_package_share_directory("m20pro_bringup")
    except PackageNotFoundError:
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        src_dir = os.path.dirname(package_dir)
        bringup_share = os.path.join(src_dir, "m20pro_bringup")

    maps_dir = os.path.join(bringup_share, "maps")
    return maps_dir if os.path.isdir(maps_dir) else os.getcwd()


def choose_yaml_path() -> str:
    return filedialog.askopenfilename(
        title="Select occ_grid.yaml",
        filetypes=[("Occupancy YAML", "*.yaml"), ("Occupancy YAML", "*.yml")],
        initialdir=default_maps_dir(),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Edit M20 Pro occupancy grid maps")
    parser.add_argument("yaml", nargs="?", default="", help="Path to occ_grid.yaml")
    args = parser.parse_args()

    root = tk.Tk()
    yaml_path = args.yaml.strip()
    if not yaml_path:
        root.withdraw()
        yaml_path = choose_yaml_path()
        if not yaml_path:
            root.destroy()
            return
        root.deiconify()

    app = MapEditorApp(root, yaml_path)
    root.minsize(1000, 720)
    root.mainloop()


if __name__ == "__main__":
    main()
