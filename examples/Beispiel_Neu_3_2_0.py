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

# Mesa Visualisierungskomponenten
from mesa.visualization import SolaraViz, make_plot_component, make_space_component
from mesa.visualization.utils import update_counter

# Mesa discrete_space Komponenten
from mesa.discrete_space import OrthogonalMooreGrid, Cell, CellAgent, FixedAgent, PropertyLayer, CellCollection


# ======================================================================
#  Konstanten
# ======================================================================

MAP_UNEXPLORED: int = 0
"""Zellstatus: Noch von keinem Roboter besucht."""
MAP_EXPLORED: int = 1
"""Zellstatus: Von mindestens einem Roboter besucht."""
MAP_OBSTACLE: int = -1
"""Zellstatus: Enthält ein unpassierbares Hindernis."""
MAP_STATION: int = 2
"""Zellstatus: Standort der Ladestation."""

MODE_EXPLORE: str = "explore"
"""Robotermodus: Sucht aktiv nach unerforschten Zellen."""
MODE_RETURN: str = "return"
"""Robotermodus: Kehrt zur Ladestation zurück."""


# ======================================================================
#  Agenten
# ======================================================================

class Robot(CellAgent):
    """Ein Roboter-Agent, der eine Gitterumgebung erkundet.

    Der Roboter navigiert durch das Gitter, markiert besuchte Zellen als
    erforscht und kehrt zur Ladestation zurück, wenn seine Batterie
    einen kritischen Schwellenwert erreicht.

    Attribute:
        explored_cells (Set[Tuple[int, int]]): Ein Set der Koordinaten,
            die dieser spezifische Roboter bereits besucht hat.
        battery (int): Der aktuelle Batteriestand des Roboters (0-100).
        mode (str): Der aktuelle Betriebsmodus des Roboters, entweder
            MODE_EXPLORE oder MODE_RETURN.
        home (Optional[Tuple[int, int]]): Die Koordinaten der Ladestation.
            Wird vom Modell bei der Initialisierung gesetzt.
        BATTERY_THRESHOLD (int): Der Batteriestand, bei dem der Roboter
            in den Rückkehrmodus wechselt.
        MAX_BATTERY (int): Die maximale Batteriekapazität.
    """
    BATTERY_THRESHOLD: int = 15
    MAX_BATTERY: int = 100

    def __init__(self, model: mesa.Model, initial_cell: Cell) -> None:
        """Initialisiert einen Roboter-Agenten.

        Args:
            model: Die Mesa-Modellinstanz, zu der dieser Agent gehört.
            initial_cell: Das Cell-Objekt, in dem der Roboter startet.
        """
        super().__init__(model)
        self.cell: Cell = initial_cell # CellAgent erfordert das Setzen von self.cell
        self.explored_cells: Set[Tuple[int, int]] = set()
        self.battery: int = self.MAX_BATTERY
        self.mode: str = MODE_EXPLORE
        self.home: Optional[Tuple[int, int]] = None

        if self.cell:
            self.explored_cells.add(self.cell.coordinate)

    def step(self) -> None:
        """Führt einen Simulationsschritt für den Roboter aus.

        Der Roboter überprüft seinen Batteriestand, passt seinen Modus an,
        markiert die aktuelle Zelle als erkundet, verbraucht Batterie und
        führt dann eine Aktion basierend auf seinem Modus aus (erkunden oder
        zurückkehren). Wenn die Batterie leer ist, bleibt der Roboter stehen.
        """
        if not self.cell: # Sicherheitsprüfung, sollte nicht eintreten
            return

        current_pos_coord: Tuple[int, int] = self.cell.coordinate

        # 1. Modus basierend auf Batteriestand anpassen
        if self.battery <= self.BATTERY_THRESHOLD and self.mode != MODE_RETURN:
            self.mode = MODE_RETURN

        # 2. Aktuelle Position als erkundet markieren (persönlich und global)
        if current_pos_coord not in self.explored_cells:
            self.explored_cells.add(current_pos_coord)
            if self.model.exploration_layer.data[current_pos_coord] == MAP_UNEXPLORED:
                self.model.mark_cell_explored(current_pos_coord)

        # 3. Batterie verbrauchen und auf Entladung prüfen
        self.battery -= 1
        if self.battery <= 0:
            self.battery = 0  # Negative Werte verhindern
            return # Roboter ist entladen und bleibt stehen

        # 4. Aktion basierend auf Modus ausführen
        if self.mode == MODE_EXPLORE:
            self.explore()
        else:  # self.mode == MODE_RETURN
            self.return_to_base()

    def explore(self) -> None:
        """Bewegt den Roboter zu einer benachbarten Zelle im Erkundungsmodus.
        Priorisiert unbesuchte Nachbarzellen. Meidet Hindernisse.
        Wählt zufällig, wenn mehrere Optionen bestehen. Bleibt stehen,
        wenn keine gültigen Züge möglich sind.
        (Refaktoriertes Beispiel mit CellCollection.select() und Lambdas)
        """
        if not self.cell:
            return

        neighborhood: CellCollection[Cell] = self.cell.get_neighborhood(radius=1, include_center=False)

        if not neighborhood.cells: # Prüfen, ob überhaupt Nachbarn vorhanden sind
            return

        # Filtere gültige Nachbarzellen (keine Hindernisse)
        valid_neighbors_collection: CellCollection[Cell] = neighborhood.select(
            lambda c: not any(isinstance(agent, Obstacle) for agent in c.agents)
        )

        if not valid_neighbors_collection.cells:
            return # Keine gültigen Züge ohne Hindernisse

        # Filtere aus den gültigen Nachbarn diejenigen, die neu sind (unerforscht durch diesen Roboter)
        new_neighbors_collection: CellCollection[Cell] = valid_neighbors_collection.select(
            lambda c: c.coordinate not in self.explored_cells
        )

        # Wähle Ziel: Priorisiere neue Zellen, sonst irgendeine gültige
        # .cells gibt eine Liste der Cell-Objekte in der Collection zurück
        target_list: List[Cell] = new_neighbors_collection.cells or valid_neighbors_collection.cells

        if not target_list:
            return # Keine Zellen zur Auswahl (sollte durch Prüfungen oben abgedeckt sein)

        target_cell: Cell = self.random.choice(target_list)
        self.move_to(target_cell)


    def return_to_base(self) -> None:
        """Bewegt den Roboter einen Schritt näher zur Ladestation.

        Wenn an der Station, wird aufgeladen und in den Erkundungsmodus gewechselt.
        Andernfalls wird die Nachbarzelle ohne Hindernis gewählt, die der
        Heimatbasis am nächsten ist (Greedy-Ansatz). Bleibt stehen, wenn festgefahren.
        (Teilweise refaktoriert mit CellCollection.select())
        """
        if not self.cell or self.home is None:
            return

        if self.cell.coordinate == self.home:
            # An der Ladestation angekommen
            self.battery = self.MAX_BATTERY
            self.mode = MODE_EXPLORE
            return

        neighborhood: CellCollection[Cell] = self.cell.get_neighborhood(radius=1, include_center=False)

        if not neighborhood.cells: # Prüfen, ob überhaupt Nachbarn vorhanden sind
            return

        # Filtere gültige Nachbarzellen (keine Hindernisse)
        valid_neighbors_collection: CellCollection[Cell] = neighborhood.select(
            lambda c: not any(isinstance(a, Obstacle) for a in c.agents)
        )

        valid_neighbor_cells_list: List[Cell] = valid_neighbors_collection.cells

        if not valid_neighbor_cells_list:
            return # Festgefahren

        # Wähle den Nachbarn, der der Basis am nächsten ist
        home_x, home_y = self.home
        target_cell: Cell = min(
            valid_neighbor_cells_list, # Verwende die gefilterte Liste
            key=lambda c: (c.coordinate[0] - home_x) ** 2 + (c.coordinate[1] - home_y) ** 2
        )
        self.move_to(target_cell)


class Obstacle(FixedAgent):
    """Ein statischer Hindernis-Agent, der die Bewegung blockiert."""

    def __init__(self, model: mesa.Model, cell: Cell) -> None:
        """Initialisiert ein Hindernis.

        Args:
            model: Die Mesa-Modellinstanz.
            cell: Das Cell-Objekt, in dem sich das Hindernis befindet.
        """
        super().__init__(model)
        self.cell: Cell = cell # FixedAgent erfordert das Setzen von self.cell

    def step(self) -> None:
        """Hindernisse sind passiv und führen keine Aktionen aus."""
        pass


class ChargingStation(FixedAgent):
    """Der Ladestations-Agent, zu dem Roboter zum Aufladen zurückkehren."""

    def __init__(self, model: mesa.Model, cell: Cell) -> None:
        """Initialisiert die Ladestation.

        Args:
            model: Die Mesa-Modellinstanz.
            cell: Das Cell-Objekt, an dem sich die Station befindet.
        """
        super().__init__(model)
        self.cell: Cell = cell # FixedAgent erfordert das Setzen von self.cell

    def step(self) -> None:
        """Ladestationen sind passiv und führen keine Aktionen aus."""
        pass


# ======================================================================
#  Modell
# ======================================================================
class ExplorationModel(mesa.Model):
    """Das Hauptsimulationsmodell für die Robotererkundung.

    Verwaltet das Gitter, die Agenten und den globalen Erkundungszustand
    mittels eines PropertyLayers.

    Attribute:
        grid (OrthogonalMooreGrid): Die räumliche Umgebung der Simulation.
        exploration_layer (PropertyLayer): Speichert den Erkundungsstatus
            jeder Zelle (unerforscht, erforscht, Hindernis, Station).
        station_pos (Tuple[int, int]): Die Koordinaten der Ladestation.
        datacollector (mesa.DataCollector): Sammelt Modelldaten über die Zeit.
        width (int): Die Breite des Gitters.
        height (int): Die Höhe des Gitters.
    """
    grid: OrthogonalMooreGrid
    exploration_layer: PropertyLayer
    station_pos: Tuple[int, int]
    datacollector: mesa.DataCollector

    def __init__(self, *,
                 width: int = 20,
                 height: int = 20,
                 num_robots: int = 5,
                 obstacle_density: float = 0.2,
                 seed: Optional[int] = None) -> None:
        """Initialisiert das Erkundungsmodell.

        Args:
            width: Die Breite des Simulationsgitters.
            height: Die Höhe des Simulationsgitters.
            num_robots: Die Anzahl der zu erstellenden Roboter.
            obstacle_density: Die Wahrscheinlichkeit (0.0-1.0), dass eine
                Zelle (außer der Station) ein Hindernis enthält.
            seed: Ein optionaler Seed für den Zufallszahlengenerator.
        """
        super().__init__(seed=seed)
        self.width: int = width
        self.height: int = height
        self.grid: OrthogonalMooreGrid = OrthogonalMooreGrid(
            (self.width, self.height), torus=False, random=self.random
        )

        self.exploration_layer: PropertyLayer = PropertyLayer(
            name="exploration_status",
            dimensions=(self.width, self.height),
            default_value=MAP_UNEXPLORED,
            dtype=int
        )

        # Hilfsdictionary für schnellen Zugriff auf Cell-Objekte via Koordinate
        self._coord_to_cell: Dict[Tuple[int, int], Cell] = {
            c.coordinate: c for c in self.grid.all_cells
        }

        self.station_pos: Tuple[int, int] = (self.width // 2, self.height // 2)
        station_cell_obj: Optional[Cell] = self._coord_to_cell.get(self.station_pos)
        if station_cell_obj is None: # Sollte nicht passieren
            raise ValueError(f"Konnte Zelle für Station bei {self.station_pos} nicht finden.")

        _station: ChargingStation = ChargingStation(self, station_cell_obj)
        self.exploration_layer.data[self.station_pos] = MAP_STATION

        self._create_obstacles(obstacle_density)
        self._create_robots(num_robots, station_cell_obj)

        self.datacollector: mesa.DataCollector = mesa.DataCollector(
            model_reporters={
                "ExplorationPercentage": self._get_exploration_percentage,
                "ActiveRobots": self._count_active_robots,
            }
        )
        self.datacollector.collect(self)

    def _create_obstacles(self, density: float) -> None:
        """Erstellt und platziert Hindernisse zufällig im Gitter.

        Args:
            density: Die Dichte der Hindernisse.
        """
        for x_coord in range(self.width):
            for y_coord in range(self.height):
                coord: Tuple[int, int] = (x_coord, y_coord)
                if coord == self.station_pos:
                    continue
                if self.random.random() < density:
                    cell_obj: Optional[Cell] = self._coord_to_cell.get(coord)
                    if cell_obj:
                        _obstacle: Obstacle = Obstacle(self, cell_obj)
                        self.exploration_layer.data[coord] = MAP_OBSTACLE

    def _create_robots(self, number_of_robots: int, station_cell: Cell) -> None:
        """Erstellt Roboter und platziert sie an der Ladestation.

        Args:
            number_of_robots: Die Anzahl der zu erstellenden Roboter.
            station_cell: Das Cell-Objekt der Ladestation.
        """
        for _ in range(number_of_robots):
            robot: Robot = Robot(self, station_cell)
            robot.home = self.station_pos

    def mark_cell_explored(self, pos: Tuple[int, int]) -> None:
        """Markiert eine Zelle als erkundet im globalen PropertyLayer.

        Args:
            pos: Die (x,y)-Koordinaten der zu markierenden Zelle.
        """
        if self.exploration_layer.data[pos] == MAP_UNEXPLORED:
            self.exploration_layer.data[pos] = MAP_EXPLORED

    def _get_exploration_percentage(self) -> float:
        """Berechnet den Prozentsatz der erkundeten, nicht-Hindernis Zellen.

        Returns:
            Der Prozentsatz (0.0-100.0) der erkundeten Fläche.
        """
        explorable_cells_count: int = (self.exploration_layer.data != MAP_OBSTACLE).sum()
        explored_cells_count: int = (self.exploration_layer.data == MAP_EXPLORED).sum()

        if explorable_cells_count == 0:
            return 100.0
        return 100.0 * float(explored_cells_count) / float(explorable_cells_count)

    def _count_active_robots(self) -> int:
        """Zählt die Anzahl der Roboter mit Batterieleistung > 0.

        Returns:
            Die Anzahl der aktiven Roboter.
        """
        if Robot not in self.agents_by_type:
            return 0
        return sum(1 for agent in self.agents_by_type[Robot] if agent.battery > 0)

    def step(self) -> None:
        """Führt einen Simulationsschritt aus.

        Aktiviert alle Roboter in zufälliger Reihenfolge und sammelt Daten.
        """
        if Robot in self.agents_by_type: # Nur ausführen, wenn Roboter existieren
            self.agents_by_type[Robot].shuffle_do("step")
        self.datacollector.collect(self)

# ======================================================================
#  Darstellung / Visualisierung
# ======================================================================
COLOR_ROBOT_EXPLORE: str = "tab:blue"
COLOR_ROBOT_RETURN_LOW_BATT: str = "tab:red"
COLOR_ROBOT_RETURN_OK_BATT: str = "tab:orange"
COLOR_OBSTACLE: str = "tab:gray"
COLOR_STATION: str = "tab:green"

def agent_portrayal(agent: mesa.Agent) -> Dict[str, Any]:
    """Erzeugt ein Portrayal-Wörterbuch für die Visualisierung eines Agenten.

    Args:
        agent: Die zu darstellende Mesa-Agenteninstanz.

    Returns:
        Ein Wörterbuch mit visuellen Eigenschaften für den Agenten.
    """
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

# Farbdefinitionen und Normalisierung für die Erkundungskarte
MAP_COLORS: Dict[int, str] = {
    MAP_OBSTACLE: "dimgray",
    MAP_UNEXPLORED: "black",
    MAP_EXPLORED: "lightblue",
    MAP_STATION: "green"
}
MAP_LABELS: Dict[int, str] = {
    MAP_OBSTACLE: "Hindernis",
    MAP_UNEXPLORED: "Unerforscht",
    MAP_EXPLORED: "Erforscht",
    MAP_STATION: "Ladestation"
}
# Sortierte Schlüssel für konsistente Reihenfolge der Farbkarte
map_bounds_keys: List[int] = sorted(MAP_COLORS.keys())
# Erzeuge Grenzen für die diskrete Farbkarte
map_norm_bounds: List[float] = [float(b) - 0.5 for b in map_bounds_keys] + [float(map_bounds_keys[-1]) + 0.5]
map_colormap: mcolors.ListedColormap = mcolors.ListedColormap([MAP_COLORS[k] for k in map_bounds_keys])
map_norm: mcolors.BoundaryNorm = mcolors.BoundaryNorm(map_norm_bounds, map_colormap.N)


@solara.component
def ExplorationMap(model: ExplorationModel) -> None:
    """Zeigt die globale Erkundungskarte als Heatmap an."""
    update_counter.get() # Stellt Aktualisierung bei Modelländerungen sicher

    fig: Figure = Figure(figsize=(6, 6))
    ax = fig.subplots()

    img = ax.imshow(model.exploration_layer.data.T, cmap=map_colormap, norm=map_norm, origin="lower")
    ax.set_title("Karte des erkundeten Raums")

    cbar = fig.colorbar(img, ax=ax, ticks=map_bounds_keys)
    cbar.set_ticklabels([MAP_LABELS[k] for k in map_bounds_keys])

    solara.FigureMatplotlib(fig)


@solara.component
def ExplorationPlot(model: ExplorationModel) -> None:
    """Zeigt ein Liniendiagramm des Erkundungsfortschritts."""
    update_counter.get()
    model_data_df = model.datacollector.get_model_vars_dataframe()
    if model_data_df.empty or "ExplorationPercentage" not in model_data_df.columns:
        return # Nichts rendern, wenn keine Daten vorhanden sind

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
    """Zeigt ein Liniendiagramm der Anzahl aktiver Roboter."""
    update_counter.get()
    model_data_df = model.datacollector.get_model_vars_dataframe()
    if model_data_df.empty or "ActiveRobots" not in model_data_df.columns:
        return

    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    model_data_df["ActiveRobots"].plot(ax=ax, grid=True)
    ax.set_title("Aktive Roboter")
    ax.set_xlabel("Zeitschritte")
    ax.set_ylabel("Anzahl Roboter")

    initial_robot_count: int = 0
    # Sicherer Zugriff, falls keine Roboter dieses Typs existieren oder die Liste leer ist
    if Robot in model.agents_by_type and model.agents_by_type[Robot]:
        initial_robot_count = len(model.agents_by_type.get(Robot, []))
    elif Robot in model.agents_by_type and not model.agents_by_type[Robot]: # Explizit 0, falls Typ existiert aber leer ist
        initial_robot_count = 0


    ax.set_ylim(0, initial_robot_count + 1 if initial_robot_count > 0 else 1) # Verhindert ylim(0,0)
    solara.FigureMatplotlib(fig)


@solara.component
def BatteryHistogram(model: ExplorationModel) -> None:
    """Zeigt ein Histogramm der Roboter-Batteriestände."""
    update_counter.get()
    battery_levels_list: List[int] = []
    # Sicherer Zugriff, falls keine Roboter dieses Typs existieren oder die Liste leer ist
    if Robot in model.agents_by_type and model.agents_by_type[Robot]:
        battery_levels_list = [agent.battery for agent in model.agents_by_type.get(Robot, [])]

    # Nur rendern, wenn Roboter existieren und Daten haben, ODER wenn explizit 0 Roboter vorhanden sind
    # (um einen leeren Plot zu vermeiden, wenn keine Roboter da sind, aber der Typ bekannt ist)
    if not battery_levels_list and not (Robot in model.agents_by_type and not model.agents_by_type[Robot]):
        return


    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    if battery_levels_list : # Nur plotten, wenn Daten vorhanden sind
        ax.hist(battery_levels_list, bins=10, range=(0, Robot.MAX_BATTERY), edgecolor="black")
    else: # Fallback für den Fall, dass es zwar Roboter vom Typ gibt, aber die Liste leer ist (z.B. 0 Roboter)
        ax.hist([], bins=10, range=(0, Robot.MAX_BATTERY), edgecolor="black")


    ax.set_title("Batterieverteilung der Roboter")
    ax.set_xlabel("Batteriestand (%)")
    ax.set_ylabel("Anzahl Roboter")
    ax.set_xlim(0, Robot.MAX_BATTERY)
    solara.FigureMatplotlib(fig)


# Parameter für die interaktive Solara-Oberfläche
# (snake_case für Schlüssel hier beibehalten, da sie von Solara intern so erwartet werden könnten)
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
    """Erstellt die Hauptseite der Solara-Visualisierung."""
    initial_model_kwargs: Dict[str, Any] = {
        key: param_config["value"] for key, param_config in model_params.items()
    }
    # Stelle sicher, dass nur gültige Parameter übergeben werden (obwohl hier alle Defaults haben)
    valid_model_kwargs: Dict[str, Any] = {
        key: value for key, value in initial_model_kwargs.items() if value is not None
    }

    # Erstelle die Modell-Instanz
    # Das `seed`-Argument wird direkt an `super().__init__(seed=seed)` in `ExplorationModel` weitergegeben.
    exploration_model: ExplorationModel = ExplorationModel(**valid_model_kwargs)

    # Erstelle die Visualisierungskomponente für den Raum
    space_visualization_component = make_space_component(agent_portrayal)

    # Erstelle die SolaraViz-Instanz
    visualization_app = SolaraViz(
        exploration_model,
        components=[
            space_visualization_component,
            ExplorationPlot,
            ActiveRobotsPlot,
            ExplorationMap,
            BatteryHistogram
        ],
        model_params=model_params,
        name="Roboter‑Erkundungssimulation (Mesa 3)"
    )
    return visualization_app