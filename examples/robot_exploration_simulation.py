"""
 Simulation von Robotern, die eine Gitterwelt mit Hindernissen und einer Ladestation erkunden.

 Dieses Modul definiert die Agenten (Roboter, Hindernis, Ladestation) und das
 Simulationsmodell (ExplorationModel) unter Verwendung des Mesa-Frameworks (v3.x).
 Es enthält Visualisierungskomponenten, die mit Solara und Matplotlib erstellt wurden,
 für interaktive Simulationsläufe.
 """

import mesa
import numpy as np
import solara
from typing import Set, Tuple, Optional, Dict, List, Any, Type

from matplotlib.figure import Figure
import matplotlib.colors as mcolors

# Verwendung von SolaraViz und Komponenten direkt aus mesa.visualization
from mesa.visualization import SolaraViz, make_plot_component, make_space_component
from mesa.visualization.utils import update_counter

# ======================================================================
#  Konstanten
# ======================================================================

# Zustände der Erkundungskarte
# MAP_UNEXPLORED (0): Zelle unbesucht.
# MAP_EXPLORED (1): Zelle besucht.
# MAP_OBSTACLE (-1): Zelle enthält ein Hindernis.
# MAP_STATION (2): Zelle enthält die Ladestation.
MAP_UNEXPLORED: int = 0  # Zellstatus: Noch von keinem Roboter besucht
MAP_EXPLORED: int = 1  # Zellstatus: Von mindestens einem Roboter besucht
MAP_OBSTACLE: int = -1  # Zellstatus: Enthält ein unpassierbares Hindernis
MAP_STATION: int = 2  # Zellstatus: Standort der Ladestation

# Roboter-Modi
MODE_EXPLORE: str = "explore"  # Robotermodus: Sucht aktiv nach unerforschten Zellen
MODE_RETURN: str = "return"  # Robotermodus: Kehrt zur Ladestation zurück


# ======================================================================
#  Agenten
# ======================================================================

class Robot(mesa.Agent):
    """
    Ein Roboter-Agent, der eine Gitterumgebung erkundet und zur Basis zurückkehrt, um sich aufzuladen.

    Attribute:
        unique_id (int): Eine eindeutige Kennung für den Agenten, zugewiesen von Mesa.
        model (mesa.Model): Die Modellinstanz, zu der der Agent gehört.
        explored_cells (Set[Tuple[int, int]]): Koordinaten, die dieser Roboter besucht hat.
        battery (int): Aktueller Batteriestand (0-100). Beginnt bei MAX_BATTERY.
        mode (str): Der aktuelle Betriebsmodus ('explore' oder 'return').
        home (Optional[Tuple[int, int]]): Koordinaten der Ladestation.
                                         Wird vom Modell während der Initialisierung gesetzt.
    """

    # Klassenkonstanten für Batteriewerte
    BATTERY_THRESHOLD: int = 15
    MAX_BATTERY: int = 100

    def __init__(self, model: mesa.Model) -> None:
        """
        Initialisiert einen neuen Roboter-Agenten.

        Parameter:
            model (mesa.Model): Die Simulationsmodellinstanz.
        """
        super().__init__(model)
        self.explored_cells: Set[Tuple[int, int]] = set()
        self.battery: int = self.MAX_BATTERY
        self.mode: str = MODE_EXPLORE
        self.home: Optional[Tuple[int, int]] = None  # Wird vom Model gesetzt

    def step(self) -> None:
        """
        Führt einen Schritt des Roboterverhaltens aus.

        Der Roboter prüft seinen Batteriestand, aktualisiert ggf. seinen Modus,
        markiert die aktuelle Zelle als erkundet, verbraucht Batterie und führt dann
        eine Aktion (erkunden oder zurückkehren) basierend auf seinem aktuellen Modus aus.
        Wenn die Batterie Null erreicht, bleibt der Roboter stehen.
        """
        # 1) Modus umschalten, falls Batterie knapp
        if self.battery <= self.BATTERY_THRESHOLD and self.mode != MODE_RETURN:
            self.mode = MODE_RETURN
            # print(f"Roboter {self.unique_id} wechselt in RETURN-Modus bei {self.pos} mit {self.battery}% Batterie.")

        # 2) Aktuelle Position als erkundet markieren (persönlich & globale Karte)
        if self.pos not in self.explored_cells:
            self.explored_cells.add(self.pos)
            # Informiere das Modell, die globale Erkundungskarte zu aktualisieren,
            # aber nur, wenn sie nicht bereits als Hindernis oder Station bekannt ist.
            if self.model.explored_map[self.pos] == MAP_UNEXPLORED:
                self.model.mark_cell_explored(self.pos)

        # 3) Batterieverbrauch & Prüfung auf Entladung
        self.battery -= 1
        if self.battery <= 0:
            # print(f"Roboter {self.unique_id} hat keine Batterie mehr bei {self.pos}.")
            self.battery = 0  # Negative Werte verhindern
            # Roboter bleibt stehen und tut in diesem Schritt nichts mehr
            return

        # 4) Aktion entsprechend Modus ausführen
        if self.mode == MODE_EXPLORE:
            self.explore()
        else:  # self.mode == MODE_RETURN
            self.return_to_base()

    def explore(self) -> None:
        """
        Bewegt den Roboter während der Erkundung zu einer Nachbarzelle.

        Der Roboter priorisiert die Bewegung zu benachbarten Zellen, die er persönlich
        noch nicht besucht hat ('new_neighbors'). Er meidet Zellen, die Hindernisse
        enthalten. Wenn mehrere neue Zellen verfügbar sind, wählt er zufällig eine aus.
        Wenn keine neuen Zellen verfügbar sind, wählt er zufällig aus allen gültigen
        (nicht-Hindernis) Nachbarn. Wenn keine gültigen Nachbarn existieren (z.B. gefangen),
        bleibt er stehen.
        """
        neighbors: List[Tuple[int, int]] = self.model.grid.get_neighborhood(
            self.pos, moore=True, include_center=False
        )

        valid_neighbors: List[Tuple[int, int]] = []
        new_neighbors: List[Tuple[int, int]] = []

        for pos in neighbors:
            # Prüfe, ob die Zelle ein Hindernis enthält
            cell_contents = self.model.grid.get_cell_list_contents(pos)
            if not any(isinstance(agent, Obstacle) for agent in cell_contents):
                valid_neighbors.append(pos)
                # Prüfe, ob *dieser* Roboter die Zelle schon kennt
                if pos not in self.explored_cells:
                    new_neighbors.append(pos)

        if not valid_neighbors:
            # Keine gültigen Züge verfügbar (eingeschlossen oder von Hindernissen umgeben)
            return

        # Wähle Ziel: Priorisiere neue Zellen, sonst nimm irgendeine gültige Zelle
        target: Tuple[int, int] = self.random.choice(new_neighbors or valid_neighbors)
        self.model.grid.move_agent(self, target)

    def return_to_base(self) -> None:
        """
        Bewegt den Roboter einen Schritt näher zur Ladestation.

        Wenn der Roboter bereits an der Station ist, lädt er sich vollständig auf und wechselt
        zurück in den 'explore'-Modus. Andernfalls identifiziert er gültige Nachbarzellen
        (keine Hindernisse) und bewegt sich zu derjenigen mit dem geringsten quadrierten
        euklidischen Abstand zu den Heimatkoordinaten. Dies ist ein einfacher Greedy-
        Pfadfindungsansatz. Wenn er feststeckt, bleibt er stehen.
        """
        if self.home is None:
            # Sollte idealerweise nicht passieren, wenn das Modell 'home' korrekt setzt.
            # print(f"Warnung: Roboter {self.unique_id} hat kein home gesetzt.")
            return

        if self.pos == self.home:
            # An der Ladestation angekommen
            self.battery = self.MAX_BATTERY
            self.mode = MODE_EXPLORE
            # print(f"Roboter {self.unique_id} hat die Basis erreicht und aufgeladen.")
            return

        # Finde Nachbarn ohne Hindernisse
        neighbors: List[Tuple[int, int]] = self.model.grid.get_neighborhood(
            self.pos, moore=True, include_center=False
        )
        valid_neighbors: List[Tuple[int, int]] = [
            p for p in neighbors
            if not any(isinstance(a, Obstacle) for a in self.model.grid.get_cell_list_contents(p))
        ]

        if not valid_neighbors:
            # In einer Sackgasse auf dem Rückweg feststeckend
            # print(f"Roboter {self.unique_id} steckt in Sackgasse bei Rückkehr fest.")
            return

        # Wähle den Nachbarn, der der Basis am nächsten ist (Greedy-Strategie)
        hx, hy = self.home
        target: Tuple[int, int] = min(
            valid_neighbors,
            key=lambda p: (p[0] - hx) ** 2 + (p[1] - hy) ** 2
        )
        self.model.grid.move_agent(self, target)


class Obstacle(mesa.Agent):
    """
    Ein statischer Hindernis-Agent, der Bewegung blockiert.

    Attribute:
        unique_id (int): Eine eindeutige Kennung für den Agenten, zugewiesen von Mesa.
        model (mesa.Model): Die Modellinstanz, zu der der Agent gehört.
    """

    def __init__(self, model: mesa.Model) -> None:
        """
        Initialisiert einen Hindernis-Agenten.

        Parameter:
            model (mesa.Model): Die Simulationsmodellinstanz.
        """
        super().__init__(model)

    def step(self) -> None:
        """Hindernisse sind passiv und führen keine Aktionen aus."""
        pass


class ChargingStation(mesa.Agent):
    """
    Der Ladestations-Agent, zu dem Roboter zum Aufladen zurückkehren.

    Attribute:
        unique_id (int): Eine eindeutige Kennung für den Agenten, zugewiesen von Mesa.
        model (mesa.Model): Die Modellinstanz, zu der der Agent gehört.
    """

    def __init__(self, model: mesa.Model) -> None:
        """
        Initialisiert den Ladestations-Agenten.

        Parameter:
            model (mesa.Model): Die Simulationsmodellinstanz.
        """
        super().__init__(model)

    def step(self) -> None:
        """Ladestationen sind passiv und führen keine Aktionen aus."""
        pass


# ======================================================================
#  Modell
# ======================================================================
class ExplorationModel(mesa.Model):
    """
    Das Hauptsimulationsmodell, das das Gitter, Agenten und den Erkundungszustand verwaltet.

    Verwendet Mesa 3.x APIs (AgentSet-Aktivierung, automatische unique IDs).

    Attribute:
        grid (mesa.space.MultiGrid): Die räumliche Umgebung für die Agenten.
        width (int): Die Breite des Gitters.
        height (int): Die Höhe des Gitters.
        explored_map (np.ndarray): Ein NumPy-Array, das den globalen Kartenzustand darstellt
                                   (unerforscht, erforscht, Hindernis, Station).
        station_pos (Tuple[int, int]): Koordinaten der einzelnen Ladestation.
        datacollector (mesa.DataCollector): Sammelt Modelldaten über die Zeit.
        agents (mesa.AgentSet): Container für alle Agenten im Modell (Mesa 3.x).
        agents_by_type (Dict[Type[mesa.Agent], mesa.AgentSet]): Agenten nach Klasse gruppiert.
    """

    grid: mesa.space.MultiGrid
    explored_map: np.ndarray
    station_pos: Tuple[int, int]
    datacollector: mesa.DataCollector

    def __init__(self, *,
                 width: int = 20,
                 height: int = 20,
                 num_robots: int = 5,
                 obstacle_density: float = 0.2,
                 seed: Optional[int] = None) -> None:
        """
        Initialisiert die Erkundungsmodell-Umgebung.

        Richtet das Gitter ein, platziert die Ladestation, füllt Hindernisse
        basierend auf der Dichte, erstellt und platziert Roboter-Agenten an der Station
        und initialisiert den Datensammler.

        Parameter:
            width (int): Die Breite des Simulationsgitters.
            height (int): Die Höhe des Simulationsgitters.
            num_robots (int): Die Anzahl der zu erstellenden Roboter-Agenten.
            obstacle_density (float): Die Wahrscheinlichkeit (0.0 bis 1.0), dass eine gegebene Zelle
                              (außer der Station) ein Hindernis enthält.
            seed (Optional[int]): Ein optionaler Seed für den Zufallszahlengenerator zur
                  Reproduzierbarkeit. Wird an mesa.Model übergeben.
        """
        super().__init__(seed=seed)
        self.grid = mesa.space.MultiGrid(width, height, torus=False)
        self.width: int = width
        self.height: int = height

        # Erkundungskarte initialisieren (alle Zellen anfangs unerforscht)
        self.explored_map = np.full((width, height), MAP_UNEXPLORED)

        # Ladestation in der Mitte platzieren
        self.station_pos = (width // 2, height // 2)
        station: ChargingStation = ChargingStation(self)
        self.grid.place_agent(station, self.station_pos)
        self.explored_map[self.station_pos] = MAP_STATION  # Station auf Karte markieren

        # Hindernisse erstellen und platzieren, auf Karte markieren
        self._create_obstacles(obstacle_density)

        # Roboter erstellen und an der Station platzieren
        self._create_robots(num_robots)

        # DataCollector für Metriken einrichten
        self.datacollector = mesa.DataCollector(
            model_reporters={
                "ExplorationPercentage": self.get_exploration_percentage,
                "ActiveRobots": self.count_active_robots,
            }
            # Beispiel für Agenten-Reporter, falls benötigt:
            # agent_reporters={"Battery": "battery", "Mode": "mode"}
        )
        # Initiale Zustandsdaten sammeln
        self.datacollector.collect(self)

    def _create_obstacles(self, density: float) -> None:
        """
        Erstellt und platziert Hindernis-Agenten zufällig im Gitter.

        Parameter:
            density (float): Die Wahrscheinlichkeit für das Platzieren eines Hindernisses in einer Zelle.
        """
        for x in range(self.width):
            for y in range(self.height):
                # Überspringe die Position der Ladestation
                if (x, y) == self.station_pos:
                    continue
                # Erstelle Hindernis basierend auf Dichtewahrscheinlichkeit
                # Verwendet self.random (initialisiert durch den Seed in super().__init__)
                if self.random.random() < density:
                    obstacle: Obstacle = Obstacle(self)
                    self.grid.place_agent(obstacle, (x, y))
                    self.explored_map[x, y] = MAP_OBSTACLE  # Hindernis auf Karte markieren

    def _create_robots(self, n: int) -> None:
        """
        Erstellt n Roboter-Agenten und platziert sie an der Ladestation.

        Parameter:
            n (int): Die Anzahl der zu erstellenden Roboter.
        """
        for _ in range(n):
            robot: Robot = Robot(self)
            robot.home = self.station_pos  # Heimatbasis für den Roboter setzen
            self.grid.place_agent(robot, self.station_pos)
            # Die Startzelle (Station) gilt als vom Roboter besucht
            robot.explored_cells.add(self.station_pos)

    def mark_cell_explored(self, pos: Tuple[int, int]) -> None:
        """
        Aktualisiert die globale Erkundungskarte, um eine Zelle als erkundet zu markieren.

        Ändert den Zustand nur, wenn die Zelle derzeit als unerforscht markiert ist.
        Hindernis- oder Stationsmarkierungen werden nicht überschrieben.

        Parameter:
            pos (Tuple[int, int]): Die (x, y)-Koordinaten der zu markierenden Zelle.
        """
        if self.explored_map[pos] == MAP_UNEXPLORED:
            self.explored_map[pos] = MAP_EXPLORED

    def get_exploration_percentage(self) -> float:
        """
        Berechnet den Prozentsatz des Gitters, der erkundet wurde.

        Die Berechnung berücksichtigt nur Zellen, die keine Hindernisse sind. Die Station
        zählt zum erkundbaren Bereich.

        Rückgabe:
            float: Der Prozentsatz (0.0 bis 100.0) der Nicht-Hindernis-Zellen, die als erkundet markiert sind.
        """
        explorable_cells: int = (self.explored_map != MAP_OBSTACLE).sum()
        # Station zählt als erkundbar, aber nicht als "erforscht" im Sinne des Fortschritts
        explored_cells: int = (self.explored_map == MAP_EXPLORED).sum()

        if explorable_cells == 0:
            return 100.0  # Wenn keine Zellen erkundbar sind, ist die Erkundung abgeschlossen
        else:
            # Berechne den Prozentsatz
            return 100.0 * float(explored_cells) / float(explorable_cells) # Explizite Float-Konvertierung für Klarheit

    def count_active_robots(self) -> int:
        """
        Zählt die Anzahl der Roboter, die derzeit Batterieleistung (> 0) haben.

        Rückgabe:
            int: Die Anzahl der Roboter mit Batterie > 0.
        """
        # Geht von der Mesa 3.x-Struktur aus, bei der Agenten über self.agents_by_type zugänglich sind
        return sum(1 for agent in self.agents_by_type[Robot] if agent.battery > 0)

    def step(self) -> None:
        """
        Führt einen Zeitschritt im Modell aus.

        Dies beinhaltet die Aktivierung der 'step'-Methode aller Roboter-Agenten in
        zufälliger Reihenfolge (unter Verwendung der Mesa 3.x AgentSet-Aktivierung, die
        self.random verwendet) und anschließendes Sammeln von Daten mit dem Datensammler.
        """
        # Aktiviere die 'step'-Methode für alle Roboter-Agenten in zufälliger Reihenfolge
        # Mesa's shuffle_do verwendet intern self.random, das durch den Seed beeinflusst wird
        self.agents_by_type[Robot].shuffle_do("step") # 'step' ist ein Methodenname

        # Sammle Daten, nachdem alle Agenten gehandelt haben
        self.datacollector.collect(self)


# ======================================================================
#  Darstellung / Visualization
# ======================================================================

# Farbdefinitionen
COLOR_ROBOT_EXPLORE: str = "tab:blue"
COLOR_ROBOT_RETURN_LOW_BATT: str = "tab:red"  # Batterie <= Schwellenwert
COLOR_ROBOT_RETURN_OK_BATT: str = "tab:orange"  # Batterie > Schwellenwert, aber im Rückkehrmodus
COLOR_OBSTACLE: str = "tab:gray"
COLOR_STATION: str = "tab:green"


def agent_portrayal(agent: mesa.Agent) -> Dict[str, Any]:
    """
    Erzeugt ein Portrayal-Wörterbuch zur Visualisierung eines Agenten in Solara/Mesa.

    Definiert die Farbe, Größe, Form, Ebene und Textattribute für verschiedene
    Agententypen (Roboter, Hindernis, Ladestation) basierend auf ihrem Zustand
    (z.B. Robotermodus, Batteriestand).

    Parameter:
        agent (mesa.Agent): Die zu darstellende Agenteninstanz.

    Rückgabe:
        Dict[str, Any]: Ein Wörterbuch, das die visuellen Eigenschaften für den Agenten definiert.
    """
    if isinstance(agent, Robot):
        color: str = COLOR_ROBOT_EXPLORE
        # Unterscheide Rückkehr-Farbe nach Dringlichkeit (Batteriestand)
        if agent.mode == MODE_RETURN:
            color = COLOR_ROBOT_RETURN_LOW_BATT if agent.battery <= Robot.BATTERY_THRESHOLD else COLOR_ROBOT_RETURN_OK_BATT
        elif agent.battery <= Robot.BATTERY_THRESHOLD:
            # Noch im Explore-Modus, aber Batterie wird knapp (visuelle Warnung)
            color = COLOR_ROBOT_RETURN_OK_BATT # Gleiche Farbe wie nicht-kritische Rückkehr

        return {
            "color": color,
            "size": 50,
            "layer": 2,  # Roboter auf oberster Ebene
            "text": f"{agent.battery}%",
            "text_color": "white" if color != COLOR_STATION else "black", # Textkontrast sicherstellen
        }

    if isinstance(agent, Obstacle):
        return {
            "color": COLOR_OBSTACLE,
            "size": 100,  # Füllt die Zelle fast aus
            "shape": "rect",
            "layer": 1,  # Unter Robotern
        }

    if isinstance(agent, ChargingStation):
        return {
            "color": COLOR_STATION,
            "size": 80,  # Etwas kleiner als Hindernis
            "shape": "rect",
            "layer": 1,  # Unter Robotern
            "text": "⚡",  # Blitz-Symbol
            "text_color": "black",
        }
    # Fallback für unerwartete Agententypen
    return {}


# ------------------------------------------------------------------
#  Solara‑Komponenten
# ------------------------------------------------------------------

# Eigene Farbkarte und Normalisierung für die Visualisierung der Erkundungskarte
MAP_COLORS: Dict[int, str] = {
    MAP_OBSTACLE: "dimgray",  # Dunkelgrau für Hindernis
    MAP_UNEXPLORED: "black",  # Schwarz für Unerforscht
    MAP_EXPLORED: "lightblue",  # Hellblau für Erforscht
    MAP_STATION: "green"  # Grün für Station
}
MAP_LABELS: Dict[int, str] = {
    MAP_OBSTACLE: "Hindernis",
    MAP_UNEXPLORED: "Unerforscht",
    MAP_EXPLORED: "Erforscht",
    MAP_STATION: "Ladestation"
}
# Sortierte Schlüssel für konsistente Reihenfolge der Farbkarte
map_bounds: List[int] = sorted(MAP_COLORS.keys())
# Erzeuge Grenzen für die diskrete Farbkarte
norm_bounds: List[float] = [float(b) - 0.5 for b in map_bounds] + [float(map_bounds[-1]) + 0.5]
cmap: mcolors.ListedColormap = mcolors.ListedColormap([MAP_COLORS[k] for k in map_bounds])
norm: mcolors.BoundaryNorm = mcolors.BoundaryNorm(norm_bounds, cmap.N)


@solara.component
def ExplorationMap(model: ExplorationModel) -> None:
    """
    Zeigt die globale Erkundungskarte als Heatmap mit Matplotlib an.

    Verwendet eine benutzerdefinierte diskrete Farbkarte, um die verschiedenen Zellzustände
    (unerforscht, erforscht, Hindernis, Station) darzustellen. Reagiert auf Modellaktualisierungen.

    Parameter:
        model (ExplorationModel): Die ExplorationModel-Instanz, die die Kartendaten enthält.
    """
    update_counter.get()  # Stellt sicher, dass die Komponente bei Modelländerungen aktualisiert wird

    fig: Figure = Figure(figsize=(6, 6))
    ax = fig.subplots()

    # Transponiere Karte für korrekte imshow-Ausrichtung (y-Achsenursprung unten)
    img = ax.imshow(model.explored_map.T, cmap=cmap, norm=norm, origin="lower")
    ax.set_title("Karte des erkundeten Raums")

    # Füge eine Farbleiste mit passenden Ticks und Beschriftungen für Kartenzustände hinzu
    cbar = fig.colorbar(img, ax=ax, ticks=map_bounds)
    cbar.set_ticklabels([MAP_LABELS[k] for k in map_bounds])

    solara.FigureMatplotlib(fig)


@solara.component
def ExplorationPlot(model: ExplorationModel) -> None:
    """
    Zeigt ein Liniendiagramm des prozentualen Erkundungsfortschritts über die Zeit an.

    Ruft Daten vom Datensammler des Modells ab. Reagiert auf Modellaktualisierungen.

    Parameter:
        model (ExplorationModel): Die ExplorationModel-Instanz, die den Datensammler enthält.
    """
    update_counter.get()
    df = model.datacollector.get_model_vars_dataframe()
    if df.empty or "ExplorationPercentage" not in df.columns:
        # Nichts rendern, wenn keine Daten vorhanden sind
        return

    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    df["ExplorationPercentage"].plot(ax=ax, grid=True)
    ax.set_title("Erkundungsfortschritt")
    ax.set_xlabel("Zeitschritte")
    ax.set_ylabel("Erkundete Fläche (%)")
    ax.set_ylim(0, 100)  # Y-Achsenskala von 0% bis 100% festlegen
    solara.FigureMatplotlib(fig)


@solara.component
def ActiveRobotsPlot(model: ExplorationModel) -> None:
    """
    Zeigt ein Liniendiagramm der Anzahl aktiver Roboter (Batterie > 0) über die Zeit an.

    Ruft Daten vom Datensammler des Modells ab. Reagiert auf Modellaktualisierungen.

    Parameter:
        model (ExplorationModel): Die ExplorationModel-Instanz, die den Datensammler enthält.
    """
    update_counter.get()
    df = model.datacollector.get_model_vars_dataframe()
    if df.empty or "ActiveRobots" not in df.columns:
        # Nichts rendern, wenn keine Daten vorhanden sind
        return

    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    df["ActiveRobots"].plot(ax=ax, grid=True)
    ax.set_title("Aktive Roboter")
    ax.set_xlabel("Zeitschritte")
    ax.set_ylabel("Anzahl Roboter")
    # Setze Y-Achsenlimit basierend auf der anfänglichen Anzahl von Robotern plus Puffer
    ax.set_ylim(0, len(model.agents_by_type.get(Robot, [])) + 1)
    solara.FigureMatplotlib(fig)


@solara.component
def BatteryHistogram(model: ExplorationModel) -> None:
    """
    Zeigt ein Histogramm der Verteilung der aktuellen Batteriestände der Roboter an.

    Reagiert auf Modellaktualisierungen.

    Parameter:
        model (ExplorationModel): Die ExplorationModel-Instanz, die die Roboter-Agenten enthält.
    """
    update_counter.get()
    # Hole Batteriestände, behandle den Fall, dass noch keine Roboter existieren könnten
    battery_levels: List[int] = [a.battery for a in model.agents_by_type.get(Robot, [])]
    if not battery_levels:
        # Nichts rendern, wenn keine Roboter vorhanden sind
        return

    fig: Figure = Figure(figsize=(6, 4))
    ax = fig.subplots()
    ax.hist(battery_levels, bins=10, range=(0, Robot.MAX_BATTERY), edgecolor="black")
    ax.set_title("Batterieverteilung der Roboter")
    ax.set_xlabel("Batteriestand (%)")
    ax.set_ylabel("Anzahl Roboter")
    ax.set_xlim(0, Robot.MAX_BATTERY)  # Fixiere X-Achse von 0 bis maximale Batterie
    solara.FigureMatplotlib(fig)


# ------------------------------------------------------------------
#  Interaktive Visualisierung Setup
# ------------------------------------------------------------------

# Parameter für die interaktive Solara-Oberfläche, mit Typ-Hinweisen und Beschriftungen
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
    """
    Erstellt die Hauptseite der Solara-Visualisierung.

    Initialisiert das Modell und richtet die Visualisierungskomponenten ein.

    Rückgabe:
        Any: Das SolaraViz-Objekt für die Darstellung.
    """
    # Hole alle initialen Werte, einschließlich des Seeds
    initial_kwargs: Dict[str, Any] = {k: v["value"] for k, v in model_params.items()}

    # In Mesa 3.x kann seed=None übergeben werden. Aber da wir einen Default-Wert (1)
    # im Slider haben, ist das Filtern von None nicht mehr zwingend notwendig,
    # schadet aber auch nicht.
    valid_kwargs: Dict[str, Any] = {k: v for k, v in initial_kwargs.items() if v is not None}

    # Erstelle die Modell-Instanz und übergebe ALLE Parameter aus valid_kwargs
    model: ExplorationModel = ExplorationModel(**valid_kwargs)
    space_viz = make_space_component(agent_portrayal)

    viz = SolaraViz(
        model,
        components=[
            space_viz,
            ExplorationPlot,
            ActiveRobotsPlot,
            ExplorationMap,
            BatteryHistogram
        ],
        model_params=model_params,
        name="Roboter‑Erkundungssimulation (Mesa 3)"
    )
    return viz