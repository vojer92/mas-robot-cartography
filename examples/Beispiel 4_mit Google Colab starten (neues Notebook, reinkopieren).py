# Notwendige Bibliotheken installieren (falls nicht schon vorhanden)
# !pip install numpy matplotlib pillow # In .py auskommentieren/entfernen

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
from matplotlib.lines import Line2D # Für Legende
from IPython.display import HTML, display # Nur relevant in Colab/Jupyter
import base64 # Nur relevant für Colab/Jupyter Anzeige
import random
from collections import deque, defaultdict
import math
from typing import Tuple, Dict, Any, Optional, Set, List, Deque # Type Hinting

# --- Konstanten ---
# Positionstyp
Position = Tuple[int, int]

# Grid-Typen (Echte Welt & Wissen)
UNKNOWN = -1
EMPTY = 0
OBSTACLE = 1
RESOURCE = 2
DEPOT = 3
GridType = int

# Agenten-Zustände (werden jetzt impliziter durch die Aufgabe gesteuert)
# Behalten zur Visualisierung bei
STATE_EXPLORING = "EXPLORING"
STATE_GOING_TO_RESOURCE = "GOING_TO_RESOURCE"
STATE_GATHERING = "GATHERING"
STATE_TRANSPORTING = "TRANSPORTING"
STATE_IDLE = "IDLE"

# Aufgabentypen
TASK_EXPLORE = 'EXPLORE'
TASK_GATHER = 'GATHER'
TASK_DEPOSIT = 'DEPOSIT'
Task = Dict[str, Any] # z.B. {'action': TASK_GATHER, 'target': (x,y)}

# Nachrichten-Typen (Protokoll)
MSG_MAP_UPDATE = "MAP_UPDATE"           # data: {'pos': Position, 'type': GridType}
MSG_RESOURCE_FOUND = "RESOURCE_FOUND"     # data: {'pos': Position, 'amount': int}
MSG_RESOURCE_DEPLETED = "RESOURCE_DEPLETED" # data: {'pos': Position}
MSG_TARGETING_RESOURCE = "TARGETING_RESOURCE" # data: {'pos': Position, 'agent_id': int}
MSG_RELEASING_RESOURCE = "RELEASING_RESOURCE" # data: {'pos': Position, 'agent_id': int}
Message = Dict[str, Any] # z.B. {'type': MSG_MAP_UPDATE, 'sender_id': int, 'data': {...}}

# Visualisierungs-Konstanten
AGENT_COLOR = 'blue'
DEPOT_COLOR = 'darkgrey'
RESOURCE_COLOR = 'gold'
OBSTACLE_COLOR = 'black'
KNOWN_MAP_COLORS = {
    UNKNOWN: 'lightgrey', EMPTY: 'white', OBSTACLE: 'dimgray',
    RESOURCE: 'palegoldenrod', DEPOT: 'lightblue'
}
AGENT_STATE_COLORS = {
    STATE_EXPLORING: 'blue', STATE_GOING_TO_RESOURCE: 'cyan',
    STATE_GATHERING: 'orange', STATE_TRANSPORTING: 'red', STATE_IDLE: 'purple'
}
SEPARATION_DISTANCE = 1.5
SEPARATION_WEIGHT = 0.3

# --- Klassen ---

class ResourcePatch:
    """Repräsentiert ein Rohstoffvorkommen in der echten Welt."""
    def __init__(self, pos: Position, amount: int):
        self.pos = pos
        self.amount = amount

class SharedKnowledge:
    """Zentrale Wissensbasis, aktualisiert durch Nachrichten."""
    def __init__(self, width: int, height: int, depot_locations: List[Position]):
        self.width = width
        self.height = height
        self.map: np.ndarray = np.full((height, width), UNKNOWN, dtype=int)
        self.known_resources: Dict[Position, int] = {} # pos -> amount
        self.known_depots: Set[Position] = set(depot_locations)
        self.total_resources_collected: int = 0
        # NEU: Wer zielt auf welche Ressource?
        self.resource_targets: Dict[Position, int] = {} # res_pos -> agent_id

        for dx, dy in depot_locations:
             if 0 <= dx < width and 0 <= dy < height:
                 self.map[dy, dx] = DEPOT

    def is_resource_targeted(self, pos: Position) -> bool:
        """Prüft, ob eine Ressource bereits von einem Agenten anvisiert wird."""
        return pos in self.resource_targets

    def update_from_message(self, message: Message):
        """Aktualisiert die Wissensbasis basierend auf einer Nachricht."""
        msg_type = message['type']
        data = message['data']
        pos = data.get('pos')

        if not pos or not (0 <= pos[0] < self.width and 0 <= pos[1] < self.height):
            return

        y, x = pos[1], pos[0] # map ist (y,x) indiziert!

        if msg_type == MSG_MAP_UPDATE:
            map_type = data['type']
            # Nur updaten, wenn Zelle unbekannt war oder Info präziser wird
            if self.map[y, x] == UNKNOWN:
                self.map[y, x] = map_type
            # Wenn Ressource bekannt war, aber jetzt als leer/Hindernis gemeldet wird
            elif self.map[y, x] == RESOURCE and map_type != RESOURCE:
                 self.map[y, x] = map_type
                 if pos in self.known_resources: del self.known_resources[pos]
                 if pos in self.resource_targets: del self.resource_targets[pos] # Auch Ziel freigeben

        elif msg_type == MSG_RESOURCE_FOUND:
            # Nur hinzufügen/updaten, wenn Zelle nicht als Hindernis/Depot bekannt ist
             if self.map[y, x] != OBSTACLE and self.map[y, x] != DEPOT:
                self.map[y, x] = RESOURCE
                current_amount = self.known_resources.get(pos, -1) # -1 um Neuentdeckung zu signalisieren
                new_amount = data.get('amount', 0)
                # Füge hinzu, wenn neu, oder aktualisiere Menge (optional: nur wenn größer?)
                if current_amount == -1 or new_amount > 0 :
                    self.known_resources[pos] = new_amount
                    # print(f"Knowledge: Resource at {pos} updated/found, amount {new_amount}") # Debug

        elif msg_type == MSG_RESOURCE_DEPLETED:
            if pos in self.known_resources: del self.known_resources[pos]
            if pos in self.resource_targets: del self.resource_targets[pos] # Ziel freigeben
            if self.map[y, x] == RESOURCE: self.map[y, x] = EMPTY

        elif msg_type == MSG_TARGETING_RESOURCE:
            agent_id = data.get('agent_id')
            # Füge Ziel hinzu, wenn noch nicht vergeben ODER dieser Agent es schon hatte
            if pos in self.known_resources and agent_id is not None:
                 if pos not in self.resource_targets or self.resource_targets[pos] == agent_id:
                      self.resource_targets[pos] = agent_id
                 # else: print(f"Knowledge: Conflict - Agent {agent_id} wants {pos}, but Agent {self.resource_targets[pos]} targets it.") # Optional: Konflikt loggen

        elif msg_type == MSG_RELEASING_RESOURCE:
            agent_id = data.get('agent_id')
            # Entferne Ziel nur, wenn dieser Agent es tatsächlich beansprucht hat
            if pos in self.resource_targets and self.resource_targets[pos] == agent_id:
                del self.resource_targets[pos]

    def get_closest_available_resource(self, agent_pos: Position) -> Optional[Position]:
        """Findet die nächstgelegene bekannte, verfügbare (>0) und nicht anvisierte Ressource."""
        available_resources = {
            pos: amount for pos, amount in self.known_resources.items()
            if amount > 0 and pos not in self.resource_targets
        }
        if not available_resources: return None

        min_dist = float('inf')
        closest_pos = None
        ax, ay = agent_pos
        # Iteriere nur über verfügbare und nicht anvisierte Ressourcen
        for pos, amount in available_resources.items():
             dist = abs(ax - pos[0]) + abs(ay - pos[1])
             if dist < min_dist:
                 min_dist = dist
                 closest_pos = pos
        return closest_pos

    # get_random_unknown_cell und get_closest_depot bleiben wie zuvor
    def get_random_unknown_cell(self) -> Optional[Position]:
        unknown_indices = np.argwhere(self.map == UNKNOWN)
        if len(unknown_indices) > 0:
             idx = random.choice(range(len(unknown_indices)))
             y, x = unknown_indices[idx]
             return (x, y)
        return None
    def get_closest_depot(self, agent_pos: Position) -> Optional[Position]:
        if not self.known_depots: return None
        min_dist = float('inf'); closest_pos = None; ax, ay = agent_pos
        for pos in self.known_depots:
            dist = abs(ax - pos[0]) + abs(ay - pos[1])
            if dist < min_dist: min_dist = dist; closest_pos = pos
        return closest_pos


class RobotAgent:
    """Repräsentiert einen autonomen Roboter mit einfacher Intention."""
    def __init__(self, agent_id: int, start_pos: Position, env: 'SimulationEnvironment'):
        self.id = agent_id
        self.pos = start_pos
        self.env = env
        self.inventory: int = 0
        self.max_carry: int = 5
        self.current_task: Optional[Task] = None # Aktuelle Aufgabe/Intention
        self.stuck_counter: int = 0
        self.visual_state: str = STATE_IDLE # Nur für Visualisierung

    def set_task(self, task: Optional[Task]):
        """Setzt eine neue Aufgabe und gibt ggf. altes Ressourcenziel frei."""
        # Wenn alte Aufgabe ein Ressourcenziel war, Nachricht zum Freigeben senden
        if (self.current_task and
            self.current_task['action'] == TASK_GATHER and
            self.current_task.get('target')):
            old_target = self.current_task['target']
            # print(f"Agent {self.id} releasing target {old_target} for new task {task}") # Debug
            self.env.post_message({
                'type': MSG_RELEASING_RESOURCE, 'sender_id': self.id, 'data': {'pos': old_target, 'agent_id': self.id}
            })

        self.current_task = task
        self.stuck_counter = 0 # Zähler zurücksetzen bei neuer Aufgabe

    def sense_and_post(self):
        """Nimmt lokale Umgebung wahr und postet Nachrichten."""
        knowledge = self.env.shared_knowledge
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                check_pos = (self.pos[0] + dx, self.pos[1] + dy)
                if not self.env.is_within_bounds(check_pos): continue

                y, x = check_pos[1], check_pos[0]
                known_type = knowledge.map[y, x]

                # Sende Update nur für unbekannte Zellen oder eigene Zelle (um Änderungen zu sehen)
                if known_type == UNKNOWN or (dx == 0 and dy == 0):
                    actual_type = self.env.get_cell_type(check_pos)
                    resource = self.env.get_resource_at(check_pos)

                    if known_type == UNKNOWN or known_type != actual_type or \
                       (actual_type == RESOURCE and check_pos not in knowledge.known_resources):

                        self.env.post_message({'type': MSG_MAP_UPDATE, 'sender_id': self.id,
                                               'data': {'pos': check_pos, 'type': actual_type}})

                        if actual_type == RESOURCE and resource:
                            self.env.post_message({'type': MSG_RESOURCE_FOUND, 'sender_id': self.id,
                                                   'data': {'pos': check_pos, 'amount': resource.amount}})


    def decide_and_act(self):
        """Wählt Aufgabe, führt Schritt aus."""
        knowledge = self.env.shared_knowledge

        # 1. Aufgabe wählen, wenn keine vorhanden ist
        if self.current_task is None:
            if self.inventory >= self.max_carry: # Wenn voll, zum Depot
                depot_target = knowledge.get_closest_depot(self.pos)
                if depot_target:
                    self.set_task({'action': TASK_DEPOSIT, 'target': depot_target})
                else: self.visual_state = STATE_IDLE # Kein Depot bekannt
            else: # Wenn nicht voll, suche Ressource oder erkunde
                resource_target = knowledge.get_closest_available_resource(self.pos)
                if resource_target:
                    self.set_task({'action': TASK_GATHER, 'target': resource_target})
                    #print(f"Agent {self.id} targeting resource {resource_target}") # Debug
                    # Absicht kommunizieren
                    self.env.post_message({
                        'type': MSG_TARGETING_RESOURCE, 'sender_id': self.id,
                        'data': {'pos': resource_target, 'agent_id': self.id}
                    })
                else: # Keine verfügbaren Ressourcen bekannt -> Erkunden
                    explore_target = knowledge.get_random_unknown_cell()
                    if explore_target:
                         self.set_task({'action': TASK_EXPLORE, 'target': explore_target})
                    else: # Alles erkundet, nichts zu tun
                         self.set_task({'action': TASK_EXPLORE, 'target': None}) # Wandern/Idle
                         self.visual_state = STATE_IDLE

        # 2. Aktuelle Aufgabe ausführen
        if self.current_task:
            action = self.current_task['action']
            target = self.current_task.get('target')

            # Stuck Check
            if self.stuck_counter > 5:
                # print(f"Agent {self.id} stuck on task {action}, choosing new task.") # Debug
                self.set_task(None) # Gibt Ziel frei, wählt nächste Runde neu
                self.visual_state = STATE_EXPLORING # Gehe sicherheitshalber erkunden
                self.stuck_counter = 0
                # Optional: Random move, um aus Sackgasse zu kommen
                self.move_random()
                return

            # Ziel erreichen
            if target and self.pos != target:
                if action == TASK_GATHER: self.visual_state = STATE_GOING_TO_RESOURCE
                elif action == TASK_DEPOSIT: self.visual_state = STATE_TRANSPORTING
                else: self.visual_state = STATE_EXPLORING

                moved = self.move_towards(target)
                if not moved: self.stuck_counter += 1
                else: self.stuck_counter = 0

            # Am Ziel angekommen oder kein spezifisches Ziel (z.B. Idle-Exploration)
            else:
                if action == TASK_GATHER:
                    self.visual_state = STATE_GATHERING
                    current_resource = self.env.get_resource_at(self.pos)
                    if current_resource and current_resource.amount > 0 and self.inventory < self.max_carry:
                        amount_to_take = min(self.max_carry - self.inventory, current_resource.amount)
                        mined_amount = self.env.mine_resource(self.pos, amount_to_take)
                        if mined_amount > 0:
                            self.inventory += mined_amount
                            # Wenn jetzt voll, oder Ressource leer -> neue Aufgabe suchen
                            if self.inventory >= self.max_carry or self.env.get_resource_at(self.pos) is None or self.env.get_resource_at(self.pos).amount <= 0:
                                if self.env.get_resource_at(self.pos) is None or self.env.get_resource_at(self.pos).amount <= 0:
                                     self.env.post_message({'type': MSG_RESOURCE_DEPLETED, 'sender_id': self.id, 'data': {'pos': self.pos}})
                                self.set_task(None) # Fertig hier, nächste Runde neue Aufgabe
                        else: # Konnte nichts abbauen (Fehler?)
                           self.set_task(None) # Neue Aufgabe
                    else: # Ressource nicht (mehr) da oder schon voll
                        if current_resource is None or current_resource.amount <= 0 :
                             self.env.post_message({'type': MSG_RESOURCE_DEPLETED, 'sender_id': self.id, 'data': {'pos': self.pos}})
                        self.set_task(None) # Neue Aufgabe

                elif action == TASK_DEPOSIT:
                    if self.inventory > 0:
                        self.env.notify_deposit(self.inventory)
                        self.inventory = 0
                    self.set_task(None) # Abgeladen, neue Aufgabe
                    self.visual_state = STATE_EXPLORING

                elif action == TASK_EXPLORE:
                    self.visual_state = STATE_EXPLORING
                    # Wenn Ziel erreicht (war unbekannte Zelle) oder kein Ziel -> neue Aufgabe
                    self.set_task(None)
                    # Optional: Wenn IDLE (kein Target bei EXPLORE), mache Random Walk
                    if not target:
                        self.move_random()

    # --- Bewegungslogik mit Separation (unverändert zum letzten Mal) ---
    def calculate_separation_vector(self, neighbors_pos: List[Position]) -> np.ndarray:
        steer = np.array([0.0, 0.0]); count = 0
        for neighbor_pos in neighbors_pos:
            diff = np.array(self.pos) - np.array(neighbor_pos); dist_sq = diff[0]**2 + diff[1]**2
            if 0 < dist_sq < SEPARATION_DISTANCE**2: steer += diff / max(dist_sq, 0.1); count += 1 # max vermeidet Division durch 0
        if count > 0: steer /= count
        return steer

    def move_random(self) -> bool:
        occupied = self.env.get_agent_positions(exclude_id=self.id)
        neighbors_pos = [p for p in occupied if math.dist(p, self.pos) <= SEPARATION_DISTANCE]
        separation_vec = self.calculate_separation_vector(neighbors_pos)
        possible_moves = list(self.env.get_neighbors(self.pos)); random.shuffle(possible_moves)
        best_move = None; max_score = -float('inf')
        for move in possible_moves:
            if not self.env.is_passable(move) or move in occupied: continue
            move_vec = np.array(move) - np.array(self.pos)
            separation_score = np.dot(move_vec, separation_vec) * SEPARATION_WEIGHT
            random_score = random.random() * (1.0 - SEPARATION_WEIGHT)
            total_score = separation_score + random_score
            if total_score > max_score: max_score = total_score; best_move = move
        if best_move: self.pos = best_move; return True
        return False

    def move_towards(self, target_pos: Position) -> bool:
        if self.pos == target_pos: return True
        occupied = self.env.get_agent_positions(exclude_id=self.id)
        neighbors_pos = [p for p in occupied if math.dist(p, self.pos) <= SEPARATION_DISTANCE]
        separation_vec = self.calculate_separation_vector(neighbors_pos)
        possible_moves = list(self.env.get_neighbors(self.pos)); random.shuffle(possible_moves)
        best_move = None; min_weighted_dist = float('inf')
        current_target_dist = abs(self.pos[0] - target_pos[0]) + abs(self.pos[1] - target_pos[1])

        for move in possible_moves:
            if not self.env.is_passable(move) or move in occupied: continue
            target_dist = abs(move[0] - target_pos[0]) + abs(move[1] - target_pos[1])
            move_vec = np.array(move) - np.array(self.pos)
            # Separation Score: Positiv ist gut (zeigt weg von Nachbarn)
            separation_score = np.dot(move_vec, separation_vec)
            # Gewichtete Distanz: Zieldistanz - kleiner Bonus für Separation
            weighted_dist = target_dist - separation_score * SEPARATION_WEIGHT * 0.1 # Kleiner Gewichtungsfaktor

            if weighted_dist < min_weighted_dist:
                 min_weighted_dist = weighted_dist; best_move = move
            # Bei gleicher gewichteter Distanz, nimm den mit besserer Separation
            elif weighted_dist == min_weighted_dist and best_move:
                 best_move_vec = np.array(best_move) - np.array(self.pos)
                 best_move_sep_score = np.dot(best_move_vec, separation_vec)
                 if separation_score > best_move_sep_score:
                      best_move = move

        # Fallback: Wenn kein Zug näher kommt (oder gleich nah mit besserer Separation),
        # nimm besten Seitwärts-/Ausweichschritt basierend auf reiner Separation
        if best_move is None:
             random.shuffle(possible_moves); max_sep_score = -float('inf')
             for move in possible_moves:
                  if not self.env.is_passable(move) or move in occupied: continue
                  dist = abs(move[0] - target_pos[0]) + abs(move[1] - target_pos[1])
                  if dist <= current_target_dist: # Nur wenn nicht weiter weg
                       move_vec = np.array(move) - np.array(self.pos)
                       separation_score = np.dot(move_vec, separation_vec)
                       if separation_score > max_sep_score:
                           max_sep_score = separation_score; best_move = move

        if best_move: self.pos = best_move; return True
        else: return False


class SimulationEnvironment:
    """Verwaltet die Umgebung, Agenten und den Simulationsablauf."""
    def __init__(self, width: int, height: int, num_agents: int, num_resources: int, num_obstacles: int, num_depots: int):
        self.width = width
        self.height = height
        self.grid: np.ndarray = np.zeros((height, width), dtype=int)
        self.agents: List[RobotAgent] = []
        self.resources: Dict[Position, ResourcePatch] = {}
        self.depots: Set[Position] = set()

        self._place_obstacles(num_obstacles)
        self._place_depots(num_depots)
        self._place_resources(num_resources)
        self._place_agents(num_agents)

        self.message_board: Deque[Message] = deque()
        self.shared_knowledge = SharedKnowledge(width, height, list(self.depots))
        self.total_deposited: int = 0

    # --- Platzierungsfunktionen (wie zuvor) ---
    def _place_obstacles(self, num):
        count = 0; attempts = 0
        while count < num and attempts < self.width * self.height * 2:
            attempts += 1; x, y = random.randrange(self.width), random.randrange(self.height)
            if self.grid[y, x] == EMPTY and (x,y) not in self.depots:
                self.grid[y, x] = OBSTACLE; count += 1
    def _place_depots(self, num):
        count = 0; attempts = 0
        while count < num and attempts < self.width * self.height * 2:
            attempts += 1; x, y = random.randrange(self.width), random.randrange(self.height)
            if self.grid[y, x] == EMPTY and (x,y) not in self.depots:
                self.depots.add((x, y)); count += 1
    def _place_resources(self, num):
        count = 0; attempts = 0
        while count < num and attempts < self.width * self.height * 2:
            attempts += 1; x, y = random.randrange(self.width), random.randrange(self.height); pos = (x,y)
            if self.grid[y, x] == EMPTY and pos not in self.depots and pos not in self.resources:
                amount = random.randint(10, 30); self.resources[pos] = ResourcePatch(pos, amount); count += 1
    def _place_agents(self, num):
         obstacle_pos = set(tuple(p) for p in np.argwhere(self.grid == OBSTACLE))
         occupied_starts = set(self.resources.keys()) | self.depots | obstacle_pos
         count = 0; attempts = 0
         while count < num and attempts < self.width * self.height * 2:
             attempts += 1; x, y = random.randrange(self.width), random.randrange(self.height); start_pos = (x, y)
             if start_pos not in occupied_starts:
                 self.agents.append(RobotAgent(count, start_pos, self)); occupied_starts.add(start_pos); count += 1

    # --- Umgebungs-API (weitgehend unverändert) ---
    def is_within_bounds(self, pos: Position) -> bool:
        x, y = pos; return 0 <= x < self.width and 0 <= y < self.height
    def get_cell_type(self, pos: Position) -> GridType:
        if not self.is_within_bounds(pos): return OBSTACLE
        resource = self.resources.get(pos) # Hole Patch direkt
        if resource and resource.amount > 0 : return RESOURCE
        if pos in self.depots: return DEPOT
        return self.grid[pos[1], pos[0]]
    def is_passable(self, pos: Position) -> bool:
        return self.is_within_bounds(pos) and self.grid[pos[1], pos[0]] != OBSTACLE
    def get_resource_at(self, pos: Position) -> Optional[ResourcePatch]:
        return self.resources.get(pos)
    def mine_resource(self, pos: Position, amount: int) -> int:
        if pos in self.resources:
            patch = self.resources[pos]; mined = min(amount, patch.amount); patch.amount -= mined
            if patch.amount <= 0: del self.resources[pos]
            return mined
        return 0
    def get_agent_positions(self, exclude_id: Optional[int] = None) -> Set[Position]:
        return {agent.pos for agent in self.agents if agent.id != exclude_id}
    def get_neighbors(self, pos: Position) -> List[Position]:
         x, y = pos; neighbors = []
         for dx in [-1, 0, 1]:
             for dy in [-1, 0, 1]:
                 if dx == 0 and dy == 0: continue
                 npos = (x + dx, y + dy)
                 if self.is_within_bounds(npos): neighbors.append(npos)
         return neighbors

    # --- Kommunikation & Wissen ---
    def post_message(self, message: Message): self.message_board.append(message)
    def process_messages(self):
        messages_to_process = len(self.message_board)
        # print(f"Processing {messages_to_process} messages.") # Debug
        for _ in range(messages_to_process):
            if not self.message_board: break
            message = self.message_board.popleft()
            self.shared_knowledge.update_from_message(message)
    def notify_deposit(self, amount: int):
        self.total_deposited += amount; self.shared_knowledge.total_resources_collected += amount

    # --- Simulationsschritt ---
    def run_step(self):
        # 1. Nachrichten vom letzten Schritt verarbeiten -> Wissen aktualisieren
        self.process_messages()
        # Mische Agentenreihenfolge für Fairness
        random.shuffle(self.agents)
        # 2. Alle Agenten wahrnehmen lassen & Nachrichten posten
        for agent in self.agents: agent.sense_and_post()
        # 3. Alle Agenten entscheiden & handeln lassen
        for agent in self.agents: agent.decide_and_act()


# --- Visualisierung ---
def plot_simulation_state(env: SimulationEnvironment, ax: plt.Axes, step_num: int, legend_handles: List[plt.Artist]):
    """Zeichnet den aktuellen Zustand der Simulation."""
    # (Visualisierungslogik bleibt exakt wie im vorherigen Code)
    ax.clear(); width, height = env.width, env.height
    known_map = env.shared_knowledge.map
    cmap_list = [KNOWN_MAP_COLORS[UNKNOWN], KNOWN_MAP_COLORS[EMPTY], KNOWN_MAP_COLORS[OBSTACLE],
                 KNOWN_MAP_COLORS[RESOURCE], KNOWN_MAP_COLORS[DEPOT]]
    cmap = plt.cm.colors.ListedColormap(cmap_list)
    bounds = [UNKNOWN - 0.5, EMPTY - 0.5, OBSTACLE - 0.5, RESOURCE - 0.5, DEPOT - 0.5, DEPOT + 0.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
    ax.imshow(known_map.T, cmap=cmap, norm=norm, origin='lower', extent=[0, width, 0, height], interpolation='nearest', alpha=0.5)
    obstacle_coords = np.argwhere(env.grid == OBSTACLE)
    if len(obstacle_coords) > 0:
        y_obs, x_obs = obstacle_coords.T
        ax.scatter(x_obs + 0.5, y_obs + 0.5, marker='s', s=100, color=OBSTACLE_COLOR, zorder=1, label='_nolegend_')
    res_x, res_y, res_s = [], [], []
    for pos, patch in env.resources.items():
        if patch.amount > 0:
            res_x.append(pos[0] + 0.5); res_y.append(pos[1] + 0.5); res_s.append(50 + patch.amount * 4)
    ax.scatter(res_x, res_y, marker='o', s=res_s, color=RESOURCE_COLOR, edgecolors='black', zorder=2, label='_nolegend_')
    depot_x = [pos[0] + 0.5 for pos in env.depots]; depot_y = [pos[1] + 0.5 for pos in env.depots]
    ax.scatter(depot_x, depot_y, marker='s', s=200, color=DEPOT_COLOR, edgecolors='white', zorder=2, label='_nolegend_')
    for dx, dy in zip(depot_x, depot_y): ax.text(dx, dy, "D", ha='center', va='center', color='white', fontsize=9, weight='bold', zorder=3)
    for agent in env.agents:
        x, y = agent.pos; color = AGENT_STATE_COLORS.get(agent.visual_state, AGENT_COLOR) # Nutze visual_state
        circle = Circle([x + 0.5, y + 0.5], radius=0.4, color=color, ec='black', lw=0.5, zorder=4, label='_nolegend_')
        ax.add_patch(circle)
        if agent.inventory > 0: ax.text(x + 0.5, y + 0.5, str(agent.inventory), ha='center', va='center', color='white', fontsize=7, weight='bold', zorder=5)
    ax.set_xlim(0, width); ax.set_ylim(0, height)
    ax.set_xticks(np.arange(0, width + 1, 1)); ax.set_yticks(np.arange(0, height + 1, 1))
    ax.set_xticklabels([]); ax.set_yticklabels([])
    ax.grid(True, color='grey', linestyle='-', linewidth=0.5); ax.set_aspect('equal', adjustable='box')
    ax.set_title(f"Kooperative Kartierung & Sammlung - Schritt {step_num} - Gesammelt: {env.total_deposited}")
    ax.legend(handles=legend_handles, loc='upper left', bbox_to_anchor=(1.02, 1), borderaxespad=0., fontsize='small')


# --- Hauptausführung & Animation ---

# Simulationsparameter
GRID_WIDTH = 25
GRID_HEIGHT = 20
NUM_AGENTS = 10 # Etwas mehr Agenten zum Testen der Separation
NUM_RESOURCES = 25
NUM_OBSTACLES = 50
NUM_DEPOTS = 2
SIMULATION_STEPS = 250

# Umgebung erstellen
env = SimulationEnvironment(GRID_WIDTH, GRID_HEIGHT, NUM_AGENTS, NUM_RESOURCES, NUM_OBSTACLES, NUM_DEPOTS)

# Setup für Matplotlib Animation
fig, ax = plt.subplots(figsize=(12, 10))
plt.subplots_adjust(right=0.75)
current_step_anim = 0

# --- Legenden-Elemente erstellen (wie zuvor) ---
legend_elements = [
    Line2D([0], [0], marker='o', color='w', label='Agent (Farbe = Zustand)', markerfacecolor=AGENT_COLOR, markersize=10),
    Line2D([0], [0], marker='o', color='w', label='Rohstoff (Größe = Menge)', markerfacecolor=RESOURCE_COLOR, markeredgecolor='k', markersize=10),
    Line2D([0], [0], marker='s', color='w', label='Depot', markerfacecolor=DEPOT_COLOR, markeredgecolor='w', markersize=10),
    Line2D([0], [0], marker='s', color='w', label='Hindernis', markerfacecolor=OBSTACLE_COLOR, markersize=10),
]
for state, color in AGENT_STATE_COLORS.items():
     state_label_de = state.replace("GOING_TO_RESOURCE", "ZUM ROHSTOFF").replace("EXPLORING", "ERKUNDET").replace("GATHERING", "SAMMELT").replace("TRANSPORTING", "TRANSPORTIERT").replace("IDLE", "WARTET")
     legend_elements.append(Line2D([0], [0], marker='o', color='w', label=f'Zust.: {state_label_de}', markerfacecolor=color, markersize=8))

# Animations-Update-Funktion
def update(frame):
    global current_step_anim
    if current_step_anim < SIMULATION_STEPS:
        env.run_step()
        plot_simulation_state(env, ax, current_step_anim, legend_elements)
        current_step_anim += 1
    # Optional: Animation stoppen, wenn keine Ressourcen mehr da sind
    # elif not env.resources: # Prüfen, ob das Ressourcen-Dict leer ist
    #    print("Keine Ressourcen mehr übrig, stoppe Animation.")
    #    ani.event_source.stop()
    return []

# Animation erstellen
ani = animation.FuncAnimation(fig, update, frames=SIMULATION_STEPS + 10, # +10 Pufferframes
                              interval=150, repeat=False, blit=False)

print("Starte Simulation und GIF-Erstellung...")

# Speichern als GIF
gif_path = "mas_colab_simulation_rethought.gif"
try:
    ani.save(gif_path, writer='pillow', fps=10)
    print(f"Animation gespeichert als {gif_path}")
    plt.close(fig)
    # GIF in Colab anzeigen
    try:
        display(HTML(f'<h3>Multi-Agenten Simulation (Überarbeitet):</h3><img src="data:image/gif;base64,{base64.b64encode(open(gif_path, "rb").read()).decode()}">'))
    except NameError:
        print("GIF-Anzeige in Colab/Jupyter-Umgebung übersprungen.")
except Exception as e:
    print(f"Fehler beim Speichern/Anzeigen der Animation: {e}")
    print("Stellen Sie sicher, dass 'pillow' installiert ist.")

print(f"\nSimulation nach {current_step_anim} Schritten beendet.")
print(f"Insgesamt gesammelte Ressourcen: {env.total_deposited}")
print(f"Verbleibende Ressourcen-Patches: {len(env.resources)}")