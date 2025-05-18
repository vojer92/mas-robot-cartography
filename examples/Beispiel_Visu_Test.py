"""Simulation von Robotern, die eine Gitterwelt erkunden.

Dieses Modul definiert Agenten (Roboter, Hindernis, Ladestation) und das
Simulationsmodell (ExplorationModel) unter Verwendung des Mesa-Frameworks (v3.2.0).
Es nutzt 'mesa.discrete_space' Komponenten inklusive PropertyLayer für die
räumliche Repräsentation und Datenhaltung.
Interaktive Visualisierungskomponenten werden mit Solara und Matplotlib erstellt.
"""

import mesa
import numpy as np
import solara
from typing import Set, Tuple, Optional, Dict, List, Any, Type

from matplotlib.figure import Figure
import matplotlib.colors as mcolors
import pandas as pd

# Mesa Visualisierungskomponenten
from mesa.visualization import SolaraViz, make_plot_component, make_space_component
from mesa.visualization.utils import update_counter

# Mesa discrete_space Komponenten
from mesa.discrete_space import OrthogonalMooreGrid, Cell, CellAgent, FixedAgent, PropertyLayer, CellCollection

# ======================================================================
#  Konstanten
# ======================================================================

MAP_UNEXPLORED: int = 0
MAP_EXPLORED: int = 1
MAP_OBSTACLE: int = -1
MAP_STATION: int = 2

MODE_EXPLORE: str = "explore"
MODE_RETURN: str = "return"

EXPLORATION_MILESTONES = {
    "Time_to_50_Percent": 50.0,
    "Time_to_75_Percent": 75.0,
    "Time_to_90_Percent": 90.0,
}


# ======================================================================
#  Agenten
# ======================================================================

class Robot(CellAgent):
    BATTERY_THRESHOLD: int = 15
    MAX_BATTERY: int = 100

    def __init__(self, model: mesa.Model, initial_cell: Cell) -> None:
        super().__init__(model)
        self.cell: Cell = initial_cell
        self.explored_cells: Set[Tuple[int, int]] = set()
        self.battery: int = self.MAX_BATTERY
        self.mode: str = MODE_EXPLORE
        self.home: Optional[Tuple[int, int]] = None

        if self.cell:
            self.explored_cells.add(self.cell.coordinate)

    def step(self) -> None:
        if not self.cell:
            return

        current_pos_coord: Tuple[int, int] = self.cell.coordinate

        if self.battery <= self.BATTERY_THRESHOLD and self.mode != MODE_RETURN:
            self.mode = MODE_RETURN

        # Persönliche Erkundung und globale Markierung bei Bewegung (in move_to)
        # Redundanzzählung geschieht in move_to

        self.battery -= 1
        if self.battery <= 0:
            self.battery = 0
            return

        if self.mode == MODE_EXPLORE:
            self.explore()
        else:
            self.return_to_base()

    def move_to(self, target_cell: Cell) -> None:
        if target_cell and self.cell and target_cell != self.cell:  # Nur bewegen, wenn Ziel gültig und anders
            current_global_status_target = self.model.exploration_layer.data[target_cell.coordinate]

            # Redundanz loggen: Wenn Zielzelle BEREITS global erkundet ist, BEVOR dieser Roboter sie betritt
            if current_global_status_target == MAP_EXPLORED:
                if hasattr(self.model, 'redundancy_layer'):
                    self.model.redundancy_layer.data[target_cell.coordinate] += 1

            super().move_to(target_cell)  # CellAgent.move_to() aktualisiert self.cell

            # Nach der Bewegung: Persönliche Erkundung der neuen Zelle
            new_pos_coord = self.cell.coordinate  # self.cell ist jetzt target_cell
            if new_pos_coord not in self.explored_cells:
                self.explored_cells.add(new_pos_coord)
                # Globale Markierung, falls diese Zelle durch diesen Schritt *neu* global erkundet wird
                if self.model.exploration_layer.data[new_pos_coord] == MAP_UNEXPLORED:
                    self.model.mark_cell_explored(new_pos_coord)
        elif target_cell == self.cell:
            # Roboter versucht sich auf die gleiche Zelle zu bewegen (z.B. festgefahren)
            # Hier könnte man auch Redundanz loggen, wenn die Zelle erkundet ist
            if self.model.exploration_layer.data[target_cell.coordinate] == MAP_EXPLORED:
                if hasattr(self.model, 'redundancy_layer'):
                    self.model.redundancy_layer.data[target_cell.coordinate] += 1

    def explore(self) -> None:
        if not self.cell: return
        neighborhood: CellCollection[Cell] = self.cell.get_neighborhood(radius=1, include_center=False)
        if not neighborhood.cells: return
        valid_neighbors_collection: CellCollection[Cell] = neighborhood.select(
            lambda c: not any(isinstance(agent, Obstacle) for agent in c.agents)
        )
        if not valid_neighbors_collection.cells: return
        new_neighbors_collection: CellCollection[Cell] = valid_neighbors_collection.select(
            lambda c: c.coordinate not in self.explored_cells
        )
        target_list: List[Cell] = new_neighbors_collection.cells or valid_neighbors_collection.cells
        if not target_list: return
        target_cell: Cell = self.random.choice(target_list)
        self.move_to(target_cell)

    def return_to_base(self) -> None:
        if not self.cell or self.home is None: return
        if self.cell.coordinate == self.home:
            self.battery = self.MAX_BATTERY
            self.mode = MODE_EXPLORE
            return
        neighborhood: CellCollection[Cell] = self.cell.get_neighborhood(radius=1, include_center=False)
        if not neighborhood.cells: return
        valid_neighbors_collection: CellCollection[Cell] = neighborhood.select(
            lambda c: not any(isinstance(a, Obstacle) for a in c.agents)
        )
        valid_neighbor_cells_list: List[Cell] = valid_neighbors_collection.cells
        if not valid_neighbor_cells_list: return
        home_x, home_y = self.home
        target_cell: Cell = min(
            valid_neighbor_cells_list,
            key=lambda c: (c.coordinate[0] - home_x) ** 2 + (c.coordinate[1] - home_y) ** 2
        )
        self.move_to(target_cell)


class Obstacle(FixedAgent):
    def __init__(self, model: mesa.Model, cell: Cell) -> None:
        super().__init__(model)
        self.cell: Cell = cell

    def step(self) -> None: pass


class ChargingStation(FixedAgent):
    def __init__(self, model: mesa.Model, cell: Cell) -> None:
        super().__init__(model)
        self.cell: Cell = cell

    def step(self) -> None: pass


# ======================================================================
#  Modell
# ======================================================================
class ExplorationModel(mesa.Model):
    grid: OrthogonalMooreGrid
    exploration_layer: PropertyLayer
    redundancy_layer: PropertyLayer
    station_pos: Tuple[int, int]
    datacollector: mesa.DataCollector
    _milestone_achieved_steps: Dict[str, Optional[int]]
    _previous_exploration_percentage_for_rate: float  # Für robustere Ratenberechnung

    def __init__(self, *,
                 width: int = 20,
                 height: int = 20,
                 num_robots: int = 5,
                 obstacle_density: float = 0.2,
                 seed: Optional[int] = None) -> None:
        super().__init__(seed=seed)
        self.width: int = width
        self.height: int = height
        self.grid: OrthogonalMooreGrid = OrthogonalMooreGrid(
            (self.width, self.height), torus=False, random=self.random
        )
        self.exploration_layer: PropertyLayer = PropertyLayer(
            "exploration_status", (self.width, self.height), MAP_UNEXPLORED, int
        )
        self.redundancy_layer: PropertyLayer = PropertyLayer(
            "redundancy_count", (self.width, self.height), 0, int
        )
        self._coord_to_cell: Dict[Tuple[int, int], Cell] = {
            c.coordinate: c for c in self.grid.all_cells
        }
        self.station_pos: Tuple[int, int] = (self.width // 2, self.height // 2)
        station_cell_obj: Optional[Cell] = self._coord_to_cell.get(self.station_pos)
        if station_cell_obj is None:
            raise ValueError(f"Konnte Zelle für Station bei {self.station_pos} nicht finden.")
        _station: ChargingStation = ChargingStation(self, station_cell_obj)
        self.exploration_layer.data[self.station_pos] = MAP_STATION

        self._create_obstacles(obstacle_density)
        self._create_robots(num_robots, station_cell_obj)

        # Initialisierung für Ratenberechnung
        # self._previous_exploration_percentage_for_rate wird NACH dem ersten Collect gesetzt.
        # Für den allerersten Aufruf von _get_exploration_rate (im ersten collect) wird es 0 sein.
        self._previous_exploration_percentage_for_rate = 0.0

        self._milestone_achieved_steps = {name: None for name in EXPLORATION_MILESTONES}

        model_reporters_dict = {
            "ExplorationPercentage": self._get_exploration_percentage,
            "ActiveRobots": self._count_active_robots,
            "ExplorationRate": self._get_exploration_rate,
            "AverageRedundancy": self._get_average_redundancy,
        }
        for ms_name in EXPLORATION_MILESTONES:  # ms_name statt name, um Konflikt mit lambda zu vermeiden
            model_reporters_dict[ms_name] = lambda m, milestone_key=ms_name: m._milestone_achieved_steps[milestone_key]

        self.datacollector: mesa.DataCollector = mesa.DataCollector(model_reporters=model_reporters_dict)

        # Ersten Datenpunkt sammeln (entspricht Zustand vor dem ersten Schritt)
        current_exploration_at_init = self._get_exploration_percentage()  # Holen für _previous_exploration_percentage_for_rate
        self._check_milestones()  # Meilensteine prüfen basierend auf dem Initialzustand
        self.datacollector.collect(self)

        # Setze den Wert für die Ratenberechnung des ERSTEN echten Schritts
        self._previous_exploration_percentage_for_rate = current_exploration_at_init

    def _create_obstacles(self, density: float) -> None:
        for x_coord in range(self.width):
            for y_coord in range(self.height):
                coord: Tuple[int, int] = (x_coord, y_coord)
                if coord == self.station_pos: continue
                if self.random.random() < density:
                    cell_obj: Optional[Cell] = self._coord_to_cell.get(coord)
                    if cell_obj:
                        _obstacle: Obstacle = Obstacle(self, cell_obj)
                        self.exploration_layer.data[coord] = MAP_OBSTACLE

    def _create_robots(self, number_of_robots: int, station_cell: Cell) -> None:
        for _ in range(number_of_robots):
            robot: Robot = Robot(self, station_cell)
            robot.home = self.station_pos

    def mark_cell_explored(self, pos: Tuple[int, int]) -> None:
        if self.exploration_layer.data[pos] == MAP_UNEXPLORED:
            self.exploration_layer.data[pos] = MAP_EXPLORED

    def _get_exploration_percentage(self) -> float:
        explorable_cells_numpy = (self.exploration_layer.data != MAP_OBSTACLE)
        explored_cells_numpy = (self.exploration_layer.data == MAP_EXPLORED)
        explorable_cells_count: int = np.sum(explorable_cells_numpy)
        explored_cells_count: int = np.sum(explored_cells_numpy)
        if explorable_cells_count == 0: return 100.0
        return 100.0 * float(explored_cells_count) / float(explorable_cells_count)

    def _get_exploration_rate(self) -> float:
        # Diese Methode wird vom DataCollector während self.datacollector.collect(self) aufgerufen.
        # self._previous_exploration_percentage_for_rate wurde am Ende des *vorherigen* Schritts
        # (oder in __init__ vor dem ersten Schritt) gesetzt.
        current_exploration = self._get_exploration_percentage()  # Aktueller Wert für diesen Sammelzyklus
        rate = current_exploration - self._previous_exploration_percentage_for_rate
        return rate

    def _get_average_redundancy(self) -> float:
        explored_mask = self.exploration_layer.data == MAP_EXPLORED
        if not np.any(explored_mask): return 0.0
        relevant_redundancy_values = self.redundancy_layer.data[explored_mask]
        return np.mean(relevant_redundancy_values) if relevant_redundancy_values.size > 0 else 0.0

    def _count_active_robots(self) -> int:
        if Robot not in self.agents_by_type: return 0
        return sum(1 for agent in self.agents_by_type[Robot] if agent.battery > 0)

    def _check_milestones(self) -> None:
        current_percentage = self._get_exploration_percentage()
        for name, threshold in EXPLORATION_MILESTONES.items():
            if self._milestone_achieved_steps[name] is None and current_percentage >= threshold:
                self._milestone_achieved_steps[name] = self.steps  # self.steps ist der aktuelle Schritt

    def step(self) -> None:
        # _previous_exploration_percentage_for_rate enthält den Wert vom Ende des letzten Schritts.
        # Dieser wird von _get_exploration_rate verwendet, das von collect() aufgerufen wird.

        if Robot in self.agents_by_type:
            self.agents_by_type[Robot].shuffle_do("step")

        current_exploration_after_step = self._get_exploration_percentage()
        self._check_milestones()  # Meilensteine basierend auf dem Zustand *nach* den Agentenaktionen prüfen

        # Datensammlung: Ruft _get_exploration_rate auf, welches den _previous_ Wert verwendet
        self.datacollector.collect(self)

        # Aktualisiere _previous_exploration_percentage_for_rate für den NÄCHSTEN Schritt
        # mit dem Wert, der gerade für diesen Schritt berechnet/gesammelt wurde.
        self._previous_exploration_percentage_for_rate = current_exploration_after_step


# ======================================================================
#  Darstellung / Visualisierung (Rest des Codes bleibt gleich, bis auf Page())
# ======================================================================
COLOR_ROBOT_EXPLORE: str = "tab:blue"
COLOR_ROBOT_RETURN_LOW_BATT: str = "tab:red"
COLOR_ROBOT_RETURN_OK_BATT: str = "tab:orange"
COLOR_OBSTACLE: str = "tab:gray"
COLOR_STATION: str = "tab:green"


def agent_portrayal(agent: mesa.Agent) -> Dict[str, Any]:
    portrayal: Dict[str, Any] = {}
    if isinstance(agent, Robot):
        if agent.battery <= 0:
            color = COLOR_ROBOT_RETURN_LOW_BATT
        elif agent.mode == MODE_RETURN:
            color = COLOR_ROBOT_RETURN_OK_BATT
        else:
            color = COLOR_ROBOT_EXPLORE
        portrayal = {"color": color, "size": 50, "zorder": 2}
    elif isinstance(agent, Obstacle):
        portrayal = {"color": COLOR_OBSTACLE, "size": 100, "marker": "s", "zorder": 1}
    elif isinstance(agent, ChargingStation):
        portrayal = {"color": COLOR_STATION, "size": 80, "marker": "s", "zorder": 1}
    return portrayal


MAP_COLORS: Dict[int, str] = {
    MAP_OBSTACLE: "dimgray", MAP_UNEXPLORED: "black",
    MAP_EXPLORED: "lightblue", MAP_STATION: "green"
}
MAP_LABELS: Dict[int, str] = {
    MAP_OBSTACLE: "Hindernis", MAP_UNEXPLORED: "Unerforscht",
    MAP_EXPLORED: "Erforscht", MAP_STATION: "Ladestation"
}
map_bounds_keys: List[int] = sorted(MAP_COLORS.keys())
map_norm_bounds: List[float] = [float(b) - 0.5 for b in map_bounds_keys] + [float(map_bounds_keys[-1]) + 0.5]
map_colormap: mcolors.ListedColormap = mcolors.ListedColormap([MAP_COLORS[k] for k in map_bounds_keys])
map_norm: mcolors.BoundaryNorm = mcolors.BoundaryNorm(map_norm_bounds, map_colormap.N)


@solara.component
def ExplorationMap(model: ExplorationModel) -> None:
    update_counter.get()
    fig: Figure = Figure(figsize=(6, 6))
    ax = fig.subplots()
    img = ax.imshow(model.exploration_layer.data.T, cmap=map_colormap, norm=map_norm, origin="lower")
    ax.set_title("Karte des erkundeten Raums")
    cbar = fig.colorbar(img, ax=ax, ticks=map_bounds_keys)
    cbar.set_ticklabels([MAP_LABELS[k] for k in map_bounds_keys])
    solara.FigureMatplotlib(fig)


@solara.component
def RedundancyMap(model: ExplorationModel) -> None:
    update_counter.get()  # Beibehalten, aber beobachten
    fig: Figure = Figure(figsize=(6, 6))
    ax = fig.subplots()
    redundancy_data = model.redundancy_layer.data.T
    img = ax.imshow(redundancy_data, cmap="viridis", origin="lower", aspect='auto', vmin=0)  # vmin=0 hinzugefügt
    ax.set_title("Redundanz der Zellbesuche")
    fig.colorbar(img, ax=ax, label="Anzahl redundanter Besuche")
    solara.FigureMatplotlib(fig)


@solara.component
def ExplorationPlot(model: ExplorationModel) -> None:
    update_counter.get()
    model_data_df = model.datacollector.get_model_vars_dataframe()
    if model_data_df.empty or "ExplorationPercentage" not in model_data_df.columns: return
    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    model_data_df["ExplorationPercentage"].plot(ax=ax, grid=True)
    ax.set_title("Erkundungsfortschritt")
    ax.set_xlabel("Zeitschritte")
    ax.set_ylabel("Erkundete Fläche (%)")
    ax.set_ylim(0, 100)
    solara.FigureMatplotlib(fig)


@solara.component
def ActiveRobotsPlot(model: ExplorationModel) -> None:
    update_counter.get()
    model_data_df = model.datacollector.get_model_vars_dataframe()
    if model_data_df.empty or "ActiveRobots" not in model_data_df.columns: return
    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    model_data_df["ActiveRobots"].plot(ax=ax, grid=True)
    ax.set_title("Aktive Roboter")
    ax.set_xlabel("Zeitschritte")
    ax.set_ylabel("Anzahl Roboter")
    initial_robot_count: int = 0
    if Robot in model.agents_by_type and model.agents_by_type[Robot]:
        initial_robot_count = len(model.agents_by_type.get(Robot, []))
    elif Robot in model.agents_by_type and not model.agents_by_type[Robot]:
        initial_robot_count = 0
    ax.set_ylim(0, initial_robot_count + 1 if initial_robot_count > 0 else 1)
    solara.FigureMatplotlib(fig)


@solara.component
def BatteryHistogram(model: ExplorationModel) -> None:
    update_counter.get()
    battery_levels_list: List[int] = []
    if Robot in model.agents_by_type and model.agents_by_type[Robot]:
        battery_levels_list = [agent.battery for agent in model.agents_by_type.get(Robot, [])]
    if not battery_levels_list and not (Robot in model.agents_by_type and not model.agents_by_type[Robot]): return
    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    if battery_levels_list:
        ax.hist(battery_levels_list, bins=10, range=(0, Robot.MAX_BATTERY), edgecolor="black")
    else:
        ax.hist([], bins=10, range=(0, Robot.MAX_BATTERY), edgecolor="black")
    ax.set_title("Batterieverteilung der Roboter")
    ax.set_xlabel("Batteriestand (%)")
    ax.set_ylabel("Anzahl Roboter")
    ax.set_xlim(0, Robot.MAX_BATTERY)
    solara.FigureMatplotlib(fig)


@solara.component
def ExplorationRatePlot(model: ExplorationModel) -> None:
    update_counter.get()
    # Fehlerbehandlung, falls DataFrame-Erstellung fehlschlägt
    try:
        model_data_df = model.datacollector.get_model_vars_dataframe()
    except ValueError as e:
        solara.Error(f"Fehler beim Erstellen des DataFrames für ExplorationRatePlot: {e}")
        # Optional: Ausgabe der Längen der Listen im model_vars Dictionary zur Diagnose
        if hasattr(model, 'datacollector') and hasattr(model.datacollector, 'model_vars'):
            for k, v_list in model.datacollector.model_vars.items():
                solara.Warning(f"Reporter '{k}' hat {len(v_list)} Einträge.")
        return

    if model_data_df.empty or "ExplorationRate" not in model_data_df.columns: return
    exploration_rate_series = model_data_df["ExplorationRate"]
    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    exploration_rate_series.plot(ax=ax, grid=True)
    ax.set_title("Explorationsrate")
    ax.set_xlabel("Zeitschritte")
    ax.set_ylabel("Änderung der erkundeten Fläche (% pro Schritt)")
    solara.FigureMatplotlib(fig)


@solara.component
def AverageRedundancyPlot(model: ExplorationModel) -> None:
    update_counter.get()  # Beibehalten, aber beobachten
    # Fehlerbehandlung, falls DataFrame-Erstellung fehlschlägt
    try:
        model_data_df = model.datacollector.get_model_vars_dataframe()
    except ValueError as e:
        solara.Error(f"Fehler beim Erstellen des DataFrames für AverageRedundancyPlot: {e}")
        if hasattr(model, 'datacollector') and hasattr(model.datacollector, 'model_vars'):
            for k, v_list in model.datacollector.model_vars.items():
                solara.Warning(f"Reporter '{k}' hat {len(v_list)} Einträge.")
        return

    if model_data_df.empty or "AverageRedundancy" not in model_data_df.columns: return
    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    model_data_df["AverageRedundancy"].plot(ax=ax, grid=True)
    ax.set_title("Durchschnittliche Redundanz")
    ax.set_xlabel("Zeitschritte")
    ax.set_ylabel("Ø Redundante Besuche pro erkundeter Zelle")
    solara.FigureMatplotlib(fig)


@solara.component
def MilestoneTable(model: ExplorationModel) -> None:
    # update_counter.get() # Vorerst auskommentieren, um Render-Loop zu testen
    data = []
    for name, threshold in EXPLORATION_MILESTONES.items():
        achieved_step = model._milestone_achieved_steps.get(name)
        data.append({
            "Meilenstein": f"{int(threshold)}% Erkundung",
            "Schritt erreicht": achieved_step if achieved_step is not None else "Nicht erreicht"
        })
    if not data: return
    df = pd.DataFrame(data)
    solara.DataFrame(df)  # Solara-Komponente zur Anzeige von DataFrames


model_params: Dict[str, Dict[str, Any]] = {
    "width": {"type": "SliderInt", "value": 20, "min": 10, "max": 50, "step": 1, "label": "Breite:"},
    "height": {"type": "SliderInt", "value": 20, "min": 10, "max": 50, "step": 1, "label": "Höhe:"},
    "num_robots": {"type": "SliderInt", "value": 5, "min": 1, "max": 20, "step": 1, "label": "Anzahl Roboter:"},
    "obstacle_density": {"type": "SliderFloat", "value": 0.2, "min": 0.0, "max": 0.5, "step": 0.01,
                         "label": "Hindernisdichte:"},
    "seed": {"type": "SliderInt", "value": 1, "min": 1, "max": 10000, "step": 1, "label": "Seed:"}
}


@solara.component
def Page() -> Any:
    initial_model_kwargs: Dict[str, Any] = {
        key: param_config["value"] for key, param_config in model_params.items()
    }
    valid_model_kwargs: Dict[str, Any] = {
        key: value for key, value in initial_model_kwargs.items() if value is not None
    }
    exploration_model: ExplorationModel = ExplorationModel(**valid_model_kwargs)
    space_visualization_component = make_space_component(agent_portrayal)

    visualization_app = SolaraViz(
        exploration_model,
        components=[
            space_visualization_component,
            ExplorationPlot,
            ExplorationRatePlot,
            AverageRedundancyPlot,
            ActiveRobotsPlot,
            ExplorationMap,
            RedundancyMap,
            BatteryHistogram,
            MilestoneTable
        ],
        model_params=model_params,
        name="Roboter‑Erkundungssimulation (Mesa 3)"
    )
    return visualization_app