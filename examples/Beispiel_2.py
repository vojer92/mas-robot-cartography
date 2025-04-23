import random
import time
import threading # Für die gemeinsame Wissensbasis (optional, aber gut für Konzept)
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# --- Konfiguration ---
GRID_WIDTH = 30
GRID_HEIGHT = 20
NUM_AGENTS = 5
NUM_RESOURCES = 15
NUM_DEPOTS = 2
STEPS_PER_RESOURCE = 3 # Zeit zum Abbauen
STEPS_PER_DEPOSIT = 2  # Zeit zum Einlagern
EXPLORATION_RANGE = 2 # Wie weit ein Agent "sehen" kann

# --- Umgebungselemente ---
UNKNOWN = -1
EMPTY = 0
RESOURCE = 1
DEPOT = 2
AGENT = 3 # Nur für Visualisierung

# --- Agentenzustände ---
STATE_EXPLORING = 'EXPLORING'
STATE_MOVING_TO_RESOURCE = 'MOVING_TO_RESOURCE'
STATE_MINING = 'MINING'
STATE_MOVING_TO_DEPOT = 'MOVING_TO_DEPOT'
STATE_DEPOSITING = 'DEPOSITING'

# --- Gemeinsame Wissensbasis ---
class KnowledgeBase:
    def __init__(self, width, height, depot_locations):
        # Karte: -1: Unbekannt, 0: Leer, 1: Ressource (bekannt), 2: Depot
        self.map = np.full((height, width), UNKNOWN, dtype=int)
        self.width = width
        self.height = height
        self.known_resources = set() # Set von (x, y) Tupeln
        self.claimed_resources = set() # Ressourcen, die gerade von einem Agenten anvisiert werden
        self.depot_locations = list(depot_locations) # Liste von (x, y) Tupeln
        self.agent_positions = {} # {agent_id: (x, y)}
        self._lock = threading.Lock() # Zum Schutz bei nebenläufigem Zugriff (optional hier)

        # Depots initial bekannt machen
        for x, y in self.depot_locations:
            if 0 <= y < height and 0 <= x < width:
                self.map[y, x] = DEPOT

    def update_map(self, x, y, cell_type):
        with self._lock:
            if 0 <= y < self.height and 0 <= x < self.width:
                current_cell = self.map[y, x]
                # Nur updaten wenn Zelle unbekannt war oder Ressource entfernt wird
                # oder wenn eine vorher als leer bekannte Zelle jetzt Ressource/Depot ist
                should_update = (
                    current_cell == UNKNOWN or
                    (current_cell == RESOURCE and cell_type == EMPTY) or
                    (current_cell == EMPTY and cell_type != EMPTY)
                )

                if should_update:
                     self.map[y, x] = cell_type

                # Wenn Ressource entdeckt wird und noch nicht bekannt war ODER wenn Karte EMPTY sagte
                if cell_type == RESOURCE and (current_cell == UNKNOWN or current_cell == EMPTY):
                     self.known_resources.add((x, y))

                # Ressource wurde abgebaut (cell_type ist EMPTY)
                if cell_type == EMPTY and current_cell == RESOURCE:
                    self.known_resources.discard((x, y))
                    self.claimed_resources.discard((x, y)) # Auch Freigabe

                # Depot immer setzen (falls initial außerhalb oder als EMPTY bekannt)
                if cell_type == DEPOT and current_cell != DEPOT :
                    self.map[y, x] = DEPOT


    def add_known_resource(self, x, y):
        with self._lock:
            pos = (x, y)
            if pos not in self.known_resources:
                #print(f"KB: Ressource bei ({x},{y}) hinzugefügt.")
                self.known_resources.add(pos)
                # Sicherstellen, dass es auf der Karte ist, falls noch unbekannt
                if 0 <= y < self.height and 0 <= x < self.width and self.map[y, x] == UNKNOWN:
                    self.map[y, x] = RESOURCE

    def get_map_view(self):
        with self._lock:
            return self.map.copy() # Kopie zurückgeben

    def get_known_resources(self):
        with self._lock:
            return list(self.known_resources)

    def get_unclaimed_resources(self):
         with self._lock:
            # Gibt nur Ressourcen zurück, die auch auf der Karte als solche bekannt sind
            valid_known = {(x,y) for x,y in self.known_resources if 0 <= y < self.height and 0 <= x < self.width and self.map[y,x] == RESOURCE}
            return list(valid_known - self.claimed_resources)

    def claim_resource(self, resource_pos):
        with self._lock:
            if resource_pos in self.known_resources and resource_pos not in self.claimed_resources:
                # Extra Check: Ist es auf der Karte noch eine Ressource?
                if 0 <= resource_pos[1] < self.height and 0 <= resource_pos[0] < self.width and self.map[resource_pos[1], resource_pos[0]] == RESOURCE:
                    self.claimed_resources.add(resource_pos)
                    #print(f"KB: Ressource {resource_pos} beansprucht.")
                    return True
                else:
                    #print(f"KB: Ressource {resource_pos} nicht beanspruchbar (nicht auf Karte als Ressource).")
                    # Ressource aus known_resources entfernen, falls sie fälschlicherweise drin ist
                    self.known_resources.discard(resource_pos)
                    return False
            #print(f"KB: Konnte Ressource {resource_pos} nicht beanspruchen (bekannt: {resource_pos in self.known_resources}, beansprucht: {resource_pos in self.claimed_resources}).")
            return False

    def release_resource(self, resource_pos):
         with self._lock:
            #print(f"KB: Ressource {resource_pos} freigegeben.")
            self.claimed_resources.discard(resource_pos)

    def remove_resource(self, x, y):
        with self._lock:
            pos = (x, y)
            #print(f"KB: Ressource bei {pos} entfernt.")
            self.known_resources.discard(pos)
            self.claimed_resources.discard(pos)
            if 0 <= y < self.height and 0 <= x < self.width:
                # Wichtig: Nur auf EMPTY setzen, wenn es nicht ein Depot ist!
                if self.map[y, x] != DEPOT:
                    self.map[y, x] = EMPTY

    def update_agent_position(self, agent_id, x, y):
        with self._lock:
            self.agent_positions[agent_id] = (x, y)

    def get_agent_positions(self):
        with self._lock:
            return self.agent_positions.copy()

    def get_nearest_depot(self, x, y):
        if not self.depot_locations:
            return None
        # Berechne Manhattan-Distanz zu allen Depots
        distances = [(depot, abs(depot[0] - x) + abs(depot[1] - y)) for depot in self.depot_locations]
        if not distances:
             return None
        # Finde das Depot mit der minimalen Distanz
        nearest_depot, _ = min(distances, key=lambda item: item[1])
        return nearest_depot


    def is_known(self, x, y):
         with self._lock:
            if 0 <= y < self.height and 0 <= x < self.width:
                return self.map[y, x] != UNKNOWN
            return True # Außerhalb der Grenzen gilt als "bekannt" (nicht explorable)


# --- Umgebung ---
class Environment:
    def __init__(self, width, height, num_resources, num_depots):
        self.width = width
        self.height = height
        self.grid = np.full((height, width), EMPTY, dtype=int)
        self.depot_locations = set()
        self.resource_locations = set()

        # Depots platzieren (zufällig, aber nicht übereinander)
        placed_locations = set()
        while len(self.depot_locations) < num_depots:
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            if (x,y) not in placed_locations:
                self.grid[y, x] = DEPOT
                self.depot_locations.add((x, y))
                placed_locations.add((x,y))


        # Ressourcen platzieren (zufällig, nicht auf Depots)
        while len(self.resource_locations) < num_resources:
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            if (x,y) not in placed_locations: # Nicht auf Depot oder anderer Ressource
                self.grid[y, x] = RESOURCE
                self.resource_locations.add((x, y))
                placed_locations.add((x,y))


    def get_cell_type(self, x, y):
        if 0 <= y < self.height and 0 <= x < self.width:
            return self.grid[y, x]
        return None # Ausserhalb der Grenzen

    def remove_resource(self, x, y):
        pos = (x, y)
        if pos in self.resource_locations:
            self.grid[y, x] = EMPTY
            self.resource_locations.remove(pos)
            #print(f"ENV: Ressource bei ({x},{y}) entfernt.")
            return True
        #print(f"ENV: Versuch, nicht-existente Ressource bei ({x},{y}) zu entfernen.")
        return False

    def get_depot_locations(self):
        return list(self.depot_locations)


# --- Agent ---
class Agent:
    def __init__(self, agent_id, start_x, start_y, environment, knowledge_base):
        self.id = agent_id
        self.x = start_x
        self.y = start_y
        self.env = environment
        self.kb = knowledge_base
        self.state = STATE_EXPLORING
        self.carrying_resource = False
        self.mining_timer = 0
        self.depositing_timer = 0
        self.target_pos = None # Zielkoordinate (für Bewegung)
        self.target_resource = None # Zielressource (zum Beanspruchen)
        self.last_direction = random.choice([(0,1), (0,-1), (1,0), (-1,0)])

        # Initialen Standort bekannt machen
        self.kb.update_agent_position(self.id, self.x, self.y)
        self.perceive_and_update_kb() # Erste Wahrnehmung am Startpunkt

    def perceive_and_update_kb(self):
        # Agent "sieht" Umgebung in einem bestimmten Radius
        # und aktualisiert die Wissensbasis
        for dy in range(-EXPLORATION_RANGE, EXPLORATION_RANGE + 1):
            for dx in range(-EXPLORATION_RANGE, EXPLORATION_RANGE + 1):
                 # Nur direktes Umfeld (Radius 1 wäre nur Nachbarn)
                 if abs(dx) + abs(dy) > EXPLORATION_RANGE:
                    continue
                 check_x, check_y = self.x + dx, self.y + dy
                 cell_type = self.env.get_cell_type(check_x, check_y)
                 if cell_type is not None:
                    # Update KB mit dem was der Agent *tatsächlich* sieht
                    self.kb.update_map(check_x, check_y, cell_type)
                 else:
                    # Bereich außerhalb der Karte als bekannt (leer/blockiert) markieren
                     # Nur wenn Zelle innerhalb der KB-Grenzen liegt
                     if 0 <= check_y < self.kb.height and 0 <= check_x < self.kb.width:
                          # Prüfen ob die Zelle in der KB überhaupt existiert und UNKNOWN ist
                          if self.kb.map[check_y, check_x] == UNKNOWN:
                               self.kb.update_map(check_x, check_y, EMPTY) # Vereinfachung


    def move_towards(self, target_x, target_y):
        # Priorisiert Bewegungen, die die Distanz verringern
        moves = []
        current_dist = abs(target_x - self.x) + abs(target_y - self.y)
        if current_dist == 0: return # Bereits am Ziel

        possible_directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        random.shuffle(possible_directions) # Fügt Zufälligkeit bei gleichen Distanzen hinzu

        for dx, dy in possible_directions:
            next_x, next_y = self.x + dx, self.y + dy
            if self.is_valid_move(next_x, next_y):
                 dist = abs(target_x - next_x) + abs(target_y - next_y)
                 # Nur Züge betrachten, die uns näher bringen oder gleich weit sind
                 if dist <= current_dist:
                     moves.append(((dx, dy), dist))

        if not moves:
             # Falls kein näherer Zug möglich ist (Hindernis?), erlaube auch Züge, die Distanz erhöhen
             # Vermeide Züge zurück zur vorherigen Position wenn möglich
             prev_x, prev_y = self.x - self.last_direction[0], self.y - self.last_direction[1]
             for dx, dy in possible_directions:
                 next_x, next_y = self.x + dx, self.y + dy
                 if self.is_valid_move(next_x, next_y) and (next_x, next_y) != (prev_x, prev_y):
                      dist = abs(target_x - next_x) + abs(target_y - next_y)
                      moves.append(((dx, dy), dist))
             # Wenn immer noch keine Züge (außer zurück), erlaube auch zurück
             if not moves:
                  for dx, dy in possible_directions:
                      next_x, next_y = self.x + dx, self.y + dy
                      if self.is_valid_move(next_x, next_y):
                           dist = abs(target_x - next_x) + abs(target_y - next_y)
                           moves.append(((dx, dy), dist))


        if not moves:
             #print(f"Agent {self.id}: Kann mich nicht bewegen.")
             return # Keine gültigen Züge

        # Sortiere Züge nach Distanz zum Ziel (aufsteigend)
        moves.sort(key=lambda item: item[1])

        # Wähle den besten Zug
        best_move = moves[0][0]
        next_x, next_y = self.x + best_move[0], self.y + best_move[1]
        self.x, self.y = next_x, next_y
        self.last_direction = best_move
        #print(f"Agent {self.id}: Bewege nach ({best_move[0]},{best_move[1]}) -> ({self.x},{self.y}) zum Ziel ({target_x},{target_y})")


    def is_valid_move(self, x, y):
         # Prüft ob Zelle innerhalb Grenzen und nicht von anderem Agenten besetzt
        if not (0 <= y < self.env.height and 0 <= x < self.env.width):
            return False
        # Prüfen, ob die Zielzelle von einem *anderen* Agenten besetzt ist
        other_agent_positions = self.kb.get_agent_positions()
        for agent_id, pos in other_agent_positions.items():
             if agent_id != self.id and pos == (x, y):
                 #print(f"Agent {self.id}: Kollision bei ({x},{y}) mit Agent {agent_id} vermieden.")
                 return False
        return True

    def explore_move(self):
        # Ziel: Finde das nächste (im Sinne von BFS-Distanz) unbekannte Feld
        q = [(self.x, self.y, 0)] # (x, y, distance)
        visited = set([(self.x, self.y)])
        found_unknown_target = None

        # Breitensuche starten
        queue_index = 0
        while queue_index < len(q):
            # Wenn die Queue zu groß wird, abbrechen (Performance)
            if queue_index > self.env.width * self.env.height * 2 : # Heuristik
                # print(f"Agent {self.id}: BFS Queue zu groß, breche Exploration ab.")
                break

            curr_x, curr_y, dist = q[queue_index]
            queue_index += 1

            # Max Suchtiefe
            if dist >= max(self.env.width, self.env.height): # Nicht weiter als die Map-Dimension suchen
                 continue

            # Prüfe Nachbarn in zufälliger Reihenfolge
            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            random.shuffle(neighbors)
            for dx, dy in neighbors:
                next_x, next_y = curr_x + dx, curr_y + dy

                # Ist Nachbar innerhalb der Grenzen?
                if 0 <= next_y < self.env.height and 0 <= next_x < self.env.width:
                    # Ist Nachbar unbekannt? -> Ziel gefunden!
                    if not self.kb.is_known(next_x, next_y):
                        found_unknown_target = (next_x, next_y)
                        break # Innere Schleife beenden

                    # Ist Nachbar schon besucht?
                    if (next_x, next_y) not in visited:
                        # Wenn Nachbar bekannt, aber noch nicht besucht, zur Queue hinzufügen
                        visited.add((next_x, next_y))
                        q.append((next_x, next_y, dist + 1))

            if found_unknown_target: # Wenn in der inneren Schleife gefunden, äußere auch beenden
                break

        # ---- Nach der Breitensuche ----
        if found_unknown_target:
            # Bewege dich auf das *erste* Feld zu, das zum Ziel führt
            self.move_towards(found_unknown_target[0], found_unknown_target[1])
        else:
            # Wenn nichts Unbekanntes gefunden wurde (alles erkundet?), bewege zufällig
            possible_moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            random.shuffle(possible_moves)
            moved = False
            for dx, dy in possible_moves:
                next_x, next_y = self.x + dx, self.y + dy
                if self.is_valid_move(next_x, next_y):
                    self.x, self.y = next_x, next_y
                    self.last_direction = (dx, dy)
                    moved = True
                    break


    def decide_and_act(self):
        # 1. Wahrnehmen & KB aktualisieren
        self.perceive_and_update_kb()

        # 2. Entscheiden basierend auf Zustand
        if self.state == STATE_EXPLORING:
            if not self.carrying_resource:
                unclaimed_resources = self.kb.get_unclaimed_resources()
                if unclaimed_resources:
                    try:
                        nearest_res = min(unclaimed_resources, key=lambda res: abs(res[0] - self.x) + abs(res[1] - self.y))
                        if self.kb.claim_resource(nearest_res):
                            self.target_resource = nearest_res
                            self.target_pos = nearest_res
                            self.state = STATE_MOVING_TO_RESOURCE
                            self.move_towards(self.target_pos[0], self.target_pos[1])
                        else:
                            self.explore_move()
                    except ValueError:
                        self.explore_move()
                else:
                    self.explore_move()
            else: # Trägt Ressource, aber ist im EXPLORING State? -> Zum Depot!
                depot = self.kb.get_nearest_depot(self.x, self.y)
                if depot:
                     self.target_pos = depot
                     self.state = STATE_MOVING_TO_DEPOT
                     self.move_towards(self.target_pos[0], self.target_pos[1])
                else: # Kein Depot bekannt
                     self.explore_move()

        elif self.state == STATE_MOVING_TO_RESOURCE:
            if self.target_pos and (self.x, self.y) == self.target_pos:
                if self.env.get_cell_type(self.x, self.y) == RESOURCE:
                    self.state = STATE_MINING
                    self.mining_timer = STEPS_PER_RESOURCE
                else: # Ressource weg bei Ankunft
                    self.kb.remove_resource(self.x, self.y)
                    if self.target_resource: self.kb.release_resource(self.target_resource)
                    self.target_resource = None
                    self.target_pos = None
                    self.state = STATE_EXPLORING
            elif self.target_pos: # Noch unterwegs
                # Prüfen ob Ziel noch gültig ist
                if self.target_resource not in self.kb.get_known_resources() or \
                   self.target_resource not in self.kb.claimed_resources:
                    # Ziel ungültig geworden
                    if self.target_resource: self.kb.release_resource(self.target_resource)
                    self.target_resource = None
                    self.target_pos = None
                    self.state = STATE_EXPLORING
                    self.explore_move() # Neuen Job suchen
                else: # Ziel noch gültig, weiter bewegen
                    self.move_towards(self.target_pos[0], self.target_pos[1])
            else: # Kein Ziel mehr? Zurück zum Erkunden
                self.state = STATE_EXPLORING

        elif self.state == STATE_MINING:
            # Prüfen ob Ressource noch da ist
            if self.env.get_cell_type(self.x, self.y) != RESOURCE:
                 self.kb.remove_resource(self.x, self.y) # KB aktualisieren
                 if self.target_resource: self.kb.release_resource(self.target_resource) # Claim freigeben
                 self.target_resource = None
                 self.target_pos = None
                 self.mining_timer = 0
                 self.state = STATE_EXPLORING
            elif self.mining_timer > 0 :
                self.mining_timer -= 1 # Weiter abbauen
            else: # Abbau fertig
                if self.env.remove_resource(self.x, self.y): # Versuche Ressource zu entfernen
                    self.kb.remove_resource(self.x, self.y) # Auch aus KB entfernen
                    self.carrying_resource = True
                    if self.target_resource: self.kb.release_resource(self.target_resource) # Claim freigeben
                    self.target_resource = None # Wichtig: target_resource löschen nach Abbau
                    depot = self.kb.get_nearest_depot(self.x, self.y)
                    if depot:
                        self.target_pos = depot
                        self.state = STATE_MOVING_TO_DEPOT
                        self.move_towards(self.target_pos[0], self.target_pos[1]) # Direkt ersten Schritt machen
                    else:
                        print(f"Agent {self.id}: Ressource abgebaut, aber kein Depot bekannt!")
                        self.state = STATE_EXPLORING # Zurück zum Erkunden
                else: # Entfernen fehlgeschlagen (anderer war schneller?)
                    if self.target_resource: self.kb.release_resource(self.target_resource)
                    self.target_resource = None
                    self.target_pos = None
                    self.state = STATE_EXPLORING

        elif self.state == STATE_MOVING_TO_DEPOT:
            if self.target_pos and (self.x, self.y) == self.target_pos: # Am Depot angekommen
                self.state = STATE_DEPOSITING
                self.depositing_timer = STEPS_PER_DEPOSIT
            elif self.target_pos: # Unterwegs zum Depot
                self.move_towards(self.target_pos[0], self.target_pos[1])
            else: # Kein Ziel? Sollte nicht passieren
                 self.state = STATE_EXPLORING

        elif self.state == STATE_DEPOSITING:
            if self.depositing_timer > 0:
                self.depositing_timer -= 1 # Weiter abladen
            else: # Abladen fertig
                self.carrying_resource = False
                self.target_pos = None
                self.state = STATE_EXPLORING # Zurück zum Erkunden

        # Stelle sicher, dass die Position am Ende des Schritts aktuell ist
        self.kb.update_agent_position(self.id, self.x, self.y)


# --- Simulation ---
class Simulation:
    # Korrekte Einrückung für __init__
    def __init__(self, width, height, num_agents, num_resources, num_depots):
        self.env = Environment(width, height, num_resources, num_depots)
        self.kb = KnowledgeBase(width, height, self.env.get_depot_locations())
        self.agents = []
        agent_start_positions = set()

        for i in range(num_agents):
             start_x, start_y = -1, -1
             attempts = 0
             while attempts < width * height: # Verhindert Endlosschleife bei voller Karte
                 start_x = random.randint(0, width - 1)
                 start_y = random.randint(0, height - 1)
                 if self.env.get_cell_type(start_x, start_y) == EMPTY and (start_x, start_y) not in agent_start_positions:
                     agent_start_positions.add((start_x, start_y))
                     break
                 attempts += 1
             if attempts >= width * height:
                 print(f"WARNUNG: Konnte keine freie Startposition für Agent {i} finden!")
                 # Optional: Agent trotzdem erstellen oder Fehler werfen
                 start_x = random.randint(0, width - 1) # Notfallposition
                 start_y = random.randint(0, height - 1)

             agent = Agent(f"Agent_{i}", start_x, start_y, self.env, self.kb)
             self.agents.append(agent)

        self.step_count = 0
        self.fig = None
        self.ax = None
        self.mat = None
        self.agent_scatter = None
        self.carrying_scatter = None
        self.ani = None
        # Die Visualisierung wird jetzt hier direkt aufgerufen
        self.setup_visualization(width, height)

    # Korrekte Einrückung für setup_visualization (gleich wie __init__)
    def setup_visualization(self, width, height):
         # Visualisierung Setup
        self.fig, self.ax = plt.subplots(figsize=(max(8, width/3), max(6, height/3)))
        cmap = plt.cm.viridis.copy()
        cmap.set_bad(color='lightgrey', alpha=0.5)
        cmap.set_under(color='white')
        self.mat = self.ax.matshow(self.get_display_grid(), cmap=cmap, vmin=0.1, vmax=4)

        # Farblegende hinzufügen
        cbar = self.fig.colorbar(self.mat, ticks=[0, 1, 2], ax=self.ax, shrink=0.8) # shrink gibt etwas mehr Platz

        # Deutsche Labels für die Farblegende
        cbar.ax.set_yticklabels(['Leer', 'Ressource', 'Depot'])
        cbar.ax.tick_params(labelsize=8)

        # Grid und Ticks konfigurieren
        # plt.xticks(np.arange(width)) # Nicht mehr nötig bei ausgeblendeten Labels
        # plt.yticks(np.arange(height))
        self.ax.set_xticks(np.arange(-.5, width, 1), minor=True)
        self.ax.set_yticks(np.arange(-.5, height, 1), minor=True)
        self.ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)
        # Achsenbeschriftungen ausblenden
        self.ax.tick_params(axis='both', which='both', bottom=False, top=False, left=False, right=False,
                            labelbottom=False, labelleft=False)

        # Agentenmarker
        self.agent_scatter = self.ax.scatter([], [], c='red', s=80, marker='o', label='Agent')
        self.carrying_scatter = self.ax.scatter([], [], c='yellow', edgecolors='black', s=25, marker='s', label='Trägt Ressource', zorder=5)

        # Depot-Marker
        depot_coords = list(self.kb.depot_locations)
        if depot_coords:
            dx, dy = zip(*depot_coords)
            self.ax.scatter(dx, dy, c='blue', s=120, marker='X', label='Depot', zorder=4)

        # Legende (für Agenten etc.) - Position angepasst
        # Legende innerhalb des Plots, aber an einer Ecke platziert
        self.ax.legend(loc='best', fontsize='small') # 'best' versucht automatisch eine gute Position

        # Layout Anpassung - Nach allen Elementen aufrufen
        # self.fig.tight_layout(rect=[0, 0, 0.85, 1]) # Alternative: tight_layout ohne rect versuchen
        self.fig.tight_layout() # Lässt Matplotlib versuchen, alles automatisch anzupassen

    # Korrekte Einrückung für get_display_grid
    def get_display_grid(self):
        display_grid = self.kb.get_map_view().astype(float)
        display_grid[display_grid == UNKNOWN] = np.nan
        return display_grid

    # Korrekte Einrückung für update_viz
    def update_viz(self, frame):
        all_resources_gone = self.run_step()
        display_grid = self.get_display_grid()
        self.mat.set_data(display_grid) # Update grid data

        # Update agent positions
        agent_coords = [(a.x, a.y) for a in self.agents]
        if agent_coords:
            self.agent_scatter.set_offsets(np.array(agent_coords))
        else:
            self.agent_scatter.set_offsets(np.empty((0, 2)))

        # Update carrying agent positions
        carrying_coords = [(a.x, a.y) for a in self.agents if a.carrying_resource]
        if carrying_coords:
            self.carrying_scatter.set_offsets(np.array(carrying_coords))
        else:
            self.carrying_scatter.set_offsets(np.empty((0, 2)))

        self.ax.set_title(f"Schritt: {self.step_count}") # Update title

        if all_resources_gone:
             print(f"*** Alle Ressourcen abgebaut oder nicht mehr erreichbar nach {self.step_count} Schritten. ***")
             if hasattr(self.ani, 'event_source') and self.ani.event_source is not None:
                  try:
                       self.ani.event_source.stop()
                  except Exception: pass # Ignore errors during stop

        # Return the artists that were updated
        return [self.mat, self.agent_scatter, self.carrying_scatter]

    # Korrekte Einrückung für run_step
    def run_step(self):
        agents_to_process = list(self.agents)
        random.shuffle(agents_to_process)
        active_agents_exist = False
        for agent in agents_to_process:
            agent.decide_and_act()
            # Check if any agent is still potentially productive
            if agent.state != STATE_EXPLORING or agent.target_pos is not None:
                active_agents_exist = True

        self.step_count += 1
        no_more_resources = not self.env.resource_locations and not self.kb.get_known_resources()

        # Optional: Stop if no agents seem active anymore (potential deadlock)
        # if not active_agents_exist and self.step_count > 20: # Check after some initial steps
        #     print(f"Keine aktiven Agenten mehr (potenzieller Deadlock) nach {self.step_count} Schritten.")
        #     return True

        return no_more_resources

    # Korrekte Einrückung für run_simulation
    def run_simulation(self, num_steps=100, interval_ms=200):
        # setup_visualization wird jetzt im __init__ aufgerufen
        if self.fig is None or self.ax is None:
             print("FEHLER: Visualisierung wurde nicht korrekt initialisiert.")
             return

        self.ani = animation.FuncAnimation(self.fig, self.update_viz, frames=num_steps,
                                           interval=interval_ms, blit=True, repeat=False,
                                           save_count=num_steps)
        try:
            plt.show()
        except AttributeError as e:
             # Ignoriert den bekannten Fehler beim Schließen unter bestimmten Umständen
             if "'FuncAnimation' object has no attribute '_resize_id'" in str(e):
                  pass # Bekannten Fehler ignorieren
             else:
                 print(f"Ein unerwarteter AttributeError ist aufgetreten: {e}")
                 # raise e # Optional: Fehler trotzdem auslösen für Debugging
        except Exception as e:
            print(f"Ein Fehler während der Simulation oder Visualisierung ist aufgetreten: {e}")
            # raise e # Optional: Fehler trotzdem auslösen für Debugging


# --- Hauptprogramm --- (Keine Einrückung)
if __name__ == "__main__":
    # Korrekte Einrückung für Code im Hauptblock
    sim = Simulation(GRID_WIDTH, GRID_HEIGHT, NUM_AGENTS, NUM_RESOURCES, NUM_DEPOTS)
    sim.run_simulation(num_steps=500, interval_ms=50) # Läuft für 500 Schritte, etwas schneller