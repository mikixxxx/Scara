"""Small Tkinter editor of machine/probe JSON. Never connects to SR6."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

from rho4_probe import DEFAULT_SETTINGS, load_settings, atomic_write_json, ProbeError


FIELDS = (
    ("Host RHO4", "machine.host"),
    ("TCP port", "machine.port"),
    ("Sonda: offset X [mm]", "probe.tip_offset_mm.x"),
    ("Sonda: offset Y [mm]", "probe.tip_offset_mm.y"),
    ("Sonda: offset Z [mm]", "probe.tip_offset_mm.z"),
    ("Smer priblizeni Z (-1/+1)", "probe.approach_z_sign"),
    ("Max. draha sondy [mm]", "probe.max_travel_mm"),
    ("Rychlost sondy [mm/s]", "probe.speed_mm_s"),
    ("Timeout [s]", "probe.timeout_s"),
    ("Tolerance XY [mm]", "probe.xy_tolerance_mm"),
)


def get_nested(mapping, dotted):
    for key in dotted.split("."):
        mapping = mapping[key]
    return mapping


def set_nested(mapping, dotted, value):
    keys = dotted.split(".")
    for key in keys[:-1]:
        mapping = mapping[key]
    mapping[keys[-1]] = value


class MachineSettingsApp(tk.Tk):
    def __init__(self, path: Path):
        super().__init__()
        self.path = Path(path)
        self.settings = load_settings(self.path)
        self.title("SCARA SR6 - konfigurace sondy")
        self.resizable(False, False)
        pane = ttk.Frame(self, padding=14)
        pane.grid(row=0, column=0)
        ttk.Label(pane, text="Nastaveni stroje – pouze lokalni soubor JSON",
                  font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))
        self.fields = {}
        for i, (label, key) in enumerate(FIELDS, 1):
            ttk.Label(pane, text=label).grid(row=i, column=0, sticky="w", pady=3, padx=(0, 12))
            var = tk.StringVar(value=str(get_nested(self.settings, key)))
            ttk.Entry(pane, textvariable=var, width=24).grid(row=i, column=1, sticky="w")
            self.fields[key] = var
        row = len(FIELDS) + 1
        self.calibrated = tk.BooleanVar(value=self.settings["probe"]["calibrated"])
        ttk.Checkbutton(pane, text="Kalibrace sondy fyzicky overena", variable=self.calibrated).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(12, 0))
        ttk.Label(pane, text="POZOR: potvrzeni kalibrace nezmeri offset!",
                  foreground="red").grid(row=row+1, column=0, columnspan=2, sticky="w")
        ttk.Label(pane, text="Aktuator zatim neimplementovan. Probe jen v miste aktualniho XY.",
                  wraplength=440).grid(row=row+2, column=0, columnspan=2, sticky="w", pady=(8, 12))
        ttk.Button(pane, text="Ulozit rho4_machine.json", command=self.save).grid(
            row=row+3, column=0, columnspan=2, sticky="ew")

    def save(self):
        data = copy.deepcopy(self.settings)
        try:
            for key, var in self.fields.items():
                raw = var.get().strip()
                if key.endswith("host"):
                    value = raw
                elif key.endswith(".port") or key.endswith("_sign"):
                    value = int(raw)
                else:
                    value = float(raw)
                set_nested(data, key, value)
            if self.calibrated.get() and not self.settings["probe"]["calibrated"]:
                if not messagebox.askyesno(
                    "Potvrzeni kalibrace",
                    "Opravdu jsi FYZICKY zmeril offset sondy vuci prirube a overil smer Z?\n"
                    "Program kalibraci sam neprovadi."):
                    return
            data["probe"]["calibrated"] = bool(self.calibrated.get())
            # Run exactly the same validator before persisting the file.
            import tempfile
            from os import unlink
            with tempfile.NamedTemporaryFile(mode="w", suffix=".json", encoding="utf-8",
                                             delete=False) as tmp:
                json.dump(data, tmp)
                tmp_path = Path(tmp.name)
            try:
                load_settings(tmp_path)
            finally:
                unlink(tmp_path)
            atomic_write_json(self.path, data)
            self.settings = data
            messagebox.showinfo("Ulozeno", f"Konfigurace ulozena:\n{self.path}")
        except (ValueError, TypeError, ProbeError, OSError) as exc:
            messagebox.showerror("Chyba", str(exc))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_SETTINGS)
    args = parser.parse_args()
    MachineSettingsApp(args.config).mainloop()
