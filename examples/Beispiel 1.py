import random
import time
import math
from collections import namedtuple, deque
import heapq # Priority Queue für A*
import pygame # Für Visualisierung

# --- Konstanten ---
CELL_UNKNOWN = -1
CELL_EMPTY = 0
CELL_OBSTACLE = 1
CELL_RESOURCE = 2
CELL_STORAGE = 3

AGENT_STATE_IDLE = 'IDLE'
AGENT_STATE_EXPLORING = 'EXPLORING'
AGENT_STATE_GOING_TO_RESOURCE = 'GOING_TO_RESOURCE'
AGENT_STATE_COLLECTING = 'COLLECTING'
AGENT_STATE_GOING_TO_STORAGE = 'GOING_TO_STORAGE'
AGENT_STATE_DEPOSITING = 'DEPOSITING'
AGENT_STATE_PATHFINDING_FAILED = 'PATHFINDING_FAILED'

RESOURCE_TYPE_DEFAULT = 'R'

# Farben (RGB)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_GREY = (190, 190, 190)      # Helleres Grau für Unbekannt
COLOR_DARKGREY = (100, 100, 100)   # Dunkler für Hindernis
COLOR_LIGHTBLUE = (173, 216, 230) # Für leere Felder
COLOR_GREEN = (34, 139, 34)      # Dunkleres Grün für Ressourcen
COLOR_RED = (220, 20, 60)        # Rot für Sammeln/Ablegen
COLOR_YELLOW = (255, 255, 0)     # Gelb für getragene Ressource
COLOR_ORANGE = (255, 140, 0)     # Orange für Weg zur Ressource
COLOR_PURPLE = (148, 0, 211)     # Lila für Weg zum Lager
COLOR_BROWN = (139, 69, 19)      # Braun für Lager
COLOR_AGENT_EXPLORE = (65, 105, 225) # Königsblau für Erkunden
COLOR_CLAIM_BORDER = (255, 0, 0)   # Rot für Claim-Markierung

# Visualisierungs-Konstanten
CELL_SIZE = 25 # Pixel pro Zelle (ggf. anpassen)
INFO_HEIGHT = 60 # Bereich über der Karte für Textinfos

# --- Nachrichtenstruktur & Typen ---
Message = namedtuple("Message", ["sender_id", "receiver_id", "type", "content", "timestamp"])
MSG_TYPE_MAP_TILE = "MAP_TILE"
MSG_TYPE_RESOURCE_FOUND = "RESOURCE_FOUND"
MSG_TYPE_RESOURCE_DEPLETED = "RESOURCE_DEPLETED"
MSG_TYPE_RESOURCE_CLAIM = "RESOURCE_CLAIM"
MSG_TYPE_RESOURCE_RELEASE = "RESOURCE_RELEASE"
BROADCAST = "BROADCAST"

# --- Message Broker ---
class MessageBroker:
    def __init__(self):
        self.mailboxes = {}
        self.registered_agents = set()

    def register_agent(self, agent_id):
        if agent_id not in self.registered_agents:
            self.mailboxes[agent_id] = deque()
            self.registered_agents.add(agent_id)

    def send_message(self, message):
        MAX_MAILBOX_SIZE = 50 # Limit
        if message.receiver_id == BROADCAST:
            for agent_id in self.registered_agents:
                if agent_id != message.sender_id and len(self.mailboxes[agent_id]) < MAX_MAILBOX_SIZE:
                       self.mailboxes[agent_id].append(message)
        elif message.receiver_id in self.mailboxes:
            if len(self.mailboxes[message.receiver_id]) < MAX_MAILBOX_SIZE:
                self.mailboxes[message.receiver_id].append(message)

    def get_messages(self, agent_id):
        if agent_id in self.mailboxes:
            messages = list(self.mailboxes[agent_id])
            self.mailboxes[agent_id].clear(); return messages
        return []

# --- Umgebung (Environment) ---
class Environment:
    def __init__(self, width, height, num_resources, num_storage, obstacle_prob=0.1):
        self.width = width; self.height = height; self.grid = [[CELL_EMPTY for _ in range(width)] for _ in range(height)]; self.resources = {}; self.storage_points = []; self.storage_inventory = {}; self.agent_positions = {}
        self._place_obstacles(obstacle_prob); self._place_resources(num_resources); self._place_storage(num_storage)
        for x,y in self.storage_points:
             if 0 <= y < self.height and 0 <= x < self.width:
                if self.grid[y][x] == CELL_OBSTACLE: self.grid[y][x] = CELL_STORAGE
                self.storage_inventory[(x,y)] = {RESOURCE_TYPE_DEFAULT: 0}

    def _place_obstacles(self, probability):
        for r in range(self.height):
            for c in range(self.width):
                if r == 0 or c == 0 or r == self.height -1 or c == self.width -1: continue # Rand freilassen
                if random.random() < probability: self.grid[r][c] = CELL_OBSTACLE

    def _place_resources(self, num):
        placed = 0; attempts = 0; max_attempts = num * 20
        while placed < num and attempts < max_attempts:
            r, c = random.randint(0, self.height - 1), random.randint(0, self.width - 1)
            if self.grid[r][c] == CELL_EMPTY:
                quantity = random.randint(5, 15)
                self.grid[r][c] = CELL_RESOURCE; self.resources[(c, r)] = {'type': RESOURCE_TYPE_DEFAULT, 'quantity': quantity}; placed += 1
            attempts += 1
        if placed < num: print(f"Warnung: Konnte nur {placed} von {num} Ressourcen platzieren.")

    def _place_storage(self, num):
        placed = 0; attempts = 0; max_attempts = num * 20
        while placed < num and attempts < max_attempts:
            r, c = random.randint(0, self.height - 1), random.randint(0, self.width - 1)
            if self.grid[r][c] == CELL_EMPTY:
                 if not any((sx, sy) == (c, r) for sx, sy in self.storage_points):
                     self.grid[r][c] = CELL_STORAGE; self.storage_points.append((c, r)); placed += 1
            attempts +=1
        if placed < num: print(f"Warnung: Konnte nur {placed} von {num} Lagerpunkten platzieren.")

    def get_cell_type(self, x, y):
        if 0 <= y < self.height and 0 <= x < self.width:
            if self.grid[y][x] == CELL_RESOURCE and self.resources.get((x, y), {}).get('quantity', 0) <= 0: return CELL_EMPTY
            return self.grid[y][x]
        return CELL_OBSTACLE

    def is_valid_pos(self, x, y): return 0 <= y < self.height and 0 <= x < self.width and self.grid[y][x] != CELL_OBSTACLE

    def get_resource_details(self, x, y): return self.resources.get((x, y), None)

    def move_agent(self, agent_id, new_x, new_y):
        target_is_valid = self.is_valid_pos(new_x, new_y)
        target_is_free = not any(id != agent_id and pos == (new_x, new_y) for id, pos in self.agent_positions.items())
        if target_is_valid and target_is_free: self.agent_positions[agent_id] = (new_x, new_y); return True
        return False

    def collect_resource(self, x, y, amount_to_collect=1):
        res_info = self.resources.get((x, y))
        if res_info and res_info['quantity'] > 0:
            collected = min(amount_to_collect, res_info['quantity']); res_info['quantity'] -= collected; res_type = res_info['type']
            if res_info['quantity'] <= 0: self.grid[y][x] = CELL_EMPTY
            return collected, res_type, res_info['quantity']
        return 0, None, 0

    def deposit_resource(self, x, y, resource_type, amount):
        if (x, y) in self.storage_points:
            if (x, y) not in self.storage_inventory: self.storage_inventory[(x, y)] = {}
            if resource_type not in self.storage_inventory[(x, y)]: self.storage_inventory[(x, y)][resource_type] = 0
            self.storage_inventory[(x, y)][resource_type] += amount; return True
        return False

    def sense_local(self, x, y, radius=1):
        view = {};
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy; ct = self.get_cell_type(nx, ny); details = {'type': ct}
                if ct == CELL_RESOURCE:
                    res = self.get_resource_details(nx, ny)
                    if res: details['quantity'] = res.get('quantity', 0); details['res_type'] = res.get('type', RESOURCE_TYPE_DEFAULT)
                view[(nx, ny)] = details
        return view

# --- Gemeinsame Wissensbasis (KnowledgeBase) ---
class KnowledgeBase:
    def __init__(self, width, height, storage_points):
        self.map_data = [[CELL_UNKNOWN for _ in range(width)] for _ in range(height)]; self.resource_locations = {}; self.resource_claims = {}; self.storage_locations = list(storage_points); self.agent_locations = {}; self.width = width; self.height = height
        for x,y in storage_points:
            if 0 <= y < height and 0 <= x < width: self.map_data[y][x] = CELL_STORAGE

    def add_claim(self, pos, agent_id, timestamp): self.resource_claims[pos] = {'agent_id': agent_id, 'timestamp': timestamp}
    def remove_claim(self, pos, agent_id_releasing):
        claim = self.resource_claims.get(pos)
        if claim and claim['agent_id'] == agent_id_releasing: del self.resource_claims[pos]; return True
        return False
    def get_claim(self, pos): return self.resource_claims.get(pos)
    def is_claimed_by_other(self, pos, self_id): claim = self.resource_claims.get(pos); return claim and claim['agent_id'] != self_id
    def get_unclaimed_resources(self, self_id): known = self.get_known_resources(); return {p: d for p, d in known.items() if not self.is_claimed_by_other(p, self_id)}

    def update_map(self, x, y, cell_type, source="SENSE", timestamp=0):
         if 0 <= y < self.height and 0 <= x < self.width:
             current = self.map_data[y][x]
             if current == CELL_UNKNOWN: self.map_data[y][x] = cell_type
             elif cell_type == CELL_OBSTACLE and current != CELL_OBSTACLE: self.map_data[y][x] = cell_type

    def add_resource_info(self, x, y, res_type, quantity, current_step, source="SENSE"):
         if quantity > 0:
            existing = self.resource_locations.get((x, y))
            if not existing or current_step >= existing.get('last_seen_step', -1):
                self.resource_locations[(x, y)] = {'type': res_type, 'quantity': quantity, 'last_seen_step': current_step}
                if 0 <= y < self.height and 0 <= x < self.width and self.map_data[y][x] != CELL_STORAGE: self.map_data[y][x] = CELL_RESOURCE
                return True
         elif (x, y) in self.resource_locations:
              existing = self.resource_locations.get((x, y))
              if existing and current_step >= existing.get('last_seen_step', -1): self.remove_resource_info(x, y, source=source)
         return False

    def update_resource_quantity(self, x, y, collected, current_step):
        if (x, y) in self.resource_locations:
            self.resource_locations[(x, y)]['quantity'] -= collected; self.resource_locations[(x, y)]['last_seen_step'] = current_step
            if self.resource_locations[(x, y)]['quantity'] <= 0: self.remove_resource_info(x, y, "COLLECTION"); return True
        return False

    def remove_resource_info(self, x, y, source="UNKNOWN"):
        if (x,y) in self.resource_locations:
            del self.resource_locations[(x,y)]
            if (x, y) in self.resource_claims: del self.resource_claims[(x,y)]
            if 0 <= y < self.height and 0 <= x < self.width and not any((sx, sy) == (x, y) for sx, sy in self.storage_locations): self.map_data[y][x] = CELL_EMPTY

    def get_known_map_tile(self, x, y):
        if not (0 <= y < self.height and 0 <= x < self.width): return CELL_OBSTACLE
        # Check other agents as obstacles
        for agent_id, info in self.agent_locations.items():
            # Ensure agent_id is checked if self.id is available or passed
            # For now, assume any agent at (x,y) blocks it
             if info.get('pos') == (x, y): return CELL_OBSTACLE
        if (x,y) in self.resource_locations and self.resource_locations[(x,y)].get('quantity', 0) <= 0 and not any((sx, sy) == (x, y) for sx, sy in self.storage_locations): return CELL_EMPTY
        return self.map_data[y][x]

    def get_known_resources(self): return {p: d for p, d in self.resource_locations.items() if d.get('quantity', 0) > 0}
    def update_agent_pos(self, agent_id, x, y, state=None):
        if agent_id not in self.agent_locations: self.agent_locations[agent_id] = {}
        self.agent_locations[agent_id]['pos'] = (x, y);
        if state: self.agent_locations[agent_id]['state'] = state
    def get_storage_locations(self): return self.storage_locations
    def get_closest_unclaimed_resource(self, ax, ay, self_id):
        unclaimed = self.get_unclaimed_resources(self_id);
        if not unclaimed: return None, float('inf')
        return min(((p, math.sqrt((ax - p[0])**2 + (ay - p[1])**2)) for p in unclaimed), key=lambda i: i[1], default=(None, float('inf')))
    def get_closest_storage(self, ax, ay):
        if not self.storage_locations: return None, float('inf')
        return min(((p, math.sqrt((ax - p[0])**2 + (ay - p[1])**2)) for p in self.storage_locations), key=lambda i: i[1], default=(None, float('inf')))

# --- Agent ---
class Agent:
    def __init__(self, agent_id, start_x, start_y, env, knowledge_base, message_broker):
        self.id = agent_id; self.x = start_x; self.y = start_y; self.env = env; self.kb = knowledge_base; self.broker = message_broker
        self.state = AGENT_STATE_EXPLORING; self.carrying_resource = None; self.target_pos = None; self.current_path = deque()
        self.current_step = 0; self.message_outbox = deque(); self.pathfinding_attempts = 0
        self.env.agent_positions[self.id] = (self.x, self.y); self.kb.update_agent_pos(self.id, self.x, self.y, self.state); self.broker.register_agent(self.id)

    def set_step(self, step):
        self.current_step = step

    def sense(self):
        local_view = self.env.sense_local(self.x, self.y, radius=1)
        for (px, py), details in local_view.items():
            ctype = details['type']
            ktype = CELL_UNKNOWN
            if 0 <= py < self.kb.height and 0 <= px < self.kb.width:
                 ktype = self.kb.map_data[py][px]

            self.kb.update_map(px, py, ctype, f"SENSE_{self.id}", self.current_step)

            if ktype == CELL_UNKNOWN and ctype != CELL_UNKNOWN:
                 msg={'pos':(px,py),'type':ctype}
                 self.message_outbox.append(Message(self.id,BROADCAST,MSG_TYPE_MAP_TILE,msg,self.current_step))

            if ctype == CELL_RESOURCE:
                rtype=details.get('res_type',RESOURCE_TYPE_DEFAULT); qty=details.get('quantity',0)
                added = self.kb.add_resource_info(px, py, rtype, qty, self.current_step, f"SENSE_{self.id}")
                if added and qty > 0:
                    msg={'pos':(px,py),'type':rtype,'quantity':qty}
                    self.message_outbox.append(Message(self.id,BROADCAST,MSG_TYPE_RESOURCE_FOUND,msg,self.current_step))

            elif (px, py) in self.kb.resource_locations and ctype == CELL_EMPTY:
                 if self.kb.resource_locations[(px, py)].get('quantity', 0) > 0:
                      claim = self.kb.get_claim((px, py))
                      self.kb.remove_resource_info(px, py, f"SENSE_{self.id}")
                      msg={'pos':(px,py)}
                      self.message_outbox.append(Message(self.id,BROADCAST,MSG_TYPE_RESOURCE_DEPLETED,msg,self.current_step))
                      if claim and claim['agent_id'] == self.id:
                           self._release_claim((px, py)) # Sendet Nachricht + entfernt Claim aus KB

    def communicate(self):
        # Empfangen & Verarbeiten
        received = self.broker.get_messages(self.id)
        for msg in received:
            mtype=msg.type; content=msg.content; sender=msg.sender_id; ts=msg.timestamp
            if mtype == MSG_TYPE_MAP_TILE:
                pos,ct = content['pos'],content['type']
                self.kb.update_map(pos[0], pos[1], ct, f"MSG_{sender}", ts)
            elif mtype == MSG_TYPE_RESOURCE_FOUND:
                pos,rt,qty = content['pos'],content['type'],content['quantity']
                self.kb.add_resource_info(pos[0], pos[1], rt, qty, ts, f"MSG_{sender}")
            elif mtype == MSG_TYPE_RESOURCE_DEPLETED:
                pos = content['pos']
                self.kb.remove_resource_info(pos[0], pos[1], f"MSG_{sender}")
            elif mtype == MSG_TYPE_RESOURCE_CLAIM:
                pos = content['pos']
                self.kb.add_claim(pos, sender, ts)
                # Eigene Pläne anpassen?
                if self.target_pos == pos and self.state == AGENT_STATE_GOING_TO_RESOURCE and sender != self.id:
                     # print(f"  Agent {self.id} notices target {pos} claimed by {sender}. Exploring.") # Debug
                     self.target_pos = None; self.current_path.clear(); self.state = AGENT_STATE_EXPLORING
            elif mtype == MSG_TYPE_RESOURCE_RELEASE:
                 pos = content['pos']
                 self.kb.remove_claim(pos, sender) # KB entfernt Claim (prüft intern ID)

        # Senden der vorbereiteten Nachrichten
        while self.message_outbox:
            self.broker.send_message(self.message_outbox.popleft())

    # --- A* Pfadfindung ---
    def _heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def _find_path(self, start, goal):
        if not start or not goal or start == goal: return None
        # Nutze (f, g, node) Tupel, um bei gleichem f den niedrigeren g zu bevorzugen
        open_set = []; heapq.heappush(open_set, (0 + self._heuristic(start, goal), 0, start)); came_from = {}
        g_score = {start: 0}; open_set_hash = {start}

        while open_set:
            try: f, g, current = heapq.heappop(open_set)
            except IndexError: break
            if current not in open_set_hash: continue # Schon verarbeitet
            open_set_hash.remove(current)

            # --- Pfadrekonstruktion (Korrigiert) ---
            if current == goal:
                path = deque()
                temp = current # Temp Variable nutzen
                while temp in came_from:
                    path.appendleft(temp)
                    temp = came_from[temp] # Gehe rückwärts
                return path # Gibt Pfad als Deque zurück (ohne Start)
            # --- Ende Pfadrekonstruktion ---

            for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
                neighbor = (current[0]+dx, current[1]+dy)
                if not (0 <= neighbor[0] < self.kb.width and 0 <= neighbor[1] < self.kb.height): continue
                neighbor_tile = self.kb.get_known_map_tile(neighbor[0], neighbor[1]); cost = 1
                if neighbor_tile == CELL_OBSTACLE: continue
                tentative_g = g + cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current; g_score[neighbor] = tentative_g
                    f_neighbor = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_neighbor, tentative_g, neighbor)); open_set_hash.add(neighbor)
        return None # Kein Pfad

    # --- Entscheidung ---
    def decide(self):
        # 1. Pfadfehler behandeln
        if self.state == AGENT_STATE_PATHFINDING_FAILED:
            self._release_claim_if_held()
            self.state = AGENT_STATE_EXPLORING; self.target_pos = None; self.current_path.clear(); self.pathfinding_attempts = 0
            self.kb.update_agent_pos(self.id, self.x, self.y, self.state)
            return self._decide_explore()

        # 2. Aktiven Pfad verfolgen
        if self.current_path:
            next_step = self.current_path[0]
            if self.kb.get_known_map_tile(next_step[0], next_step[1]) == CELL_OBSTACLE:
                 self._release_claim_if_held()
                 self.current_path.clear(); self.target_pos = None; self.state = AGENT_STATE_EXPLORING
                 self.kb.update_agent_pos(self.id, self.x, self.y, self.state)
                 return self._decide_explore()
            else:
                 return ('MOVE', next_step[0], next_step[1])

        # 3. Kein Pfad aktiv -> Logische Zustände
        # Prio 1: Ablegen
        if self.state == AGENT_STATE_DEPOSITING:
            return ('DEPOSIT', self.x, self.y)

        # Prio 2: Ressource tragen -> Pfad zum Lager
        if self.carrying_resource:
            if self.state != AGENT_STATE_GOING_TO_STORAGE:
                closest_storage_pos, _ = self.kb.get_closest_storage(self.x, self.y)
                path = None
                if closest_storage_pos:
                    path = self._find_path((self.x, self.y), closest_storage_pos)

                # Korrigierter Block: Pfad prüfen und starten
                if path is not None: # Pfad gefunden
                    self._start_following_path(path, closest_storage_pos, AGENT_STATE_GOING_TO_STORAGE)
                    if self.current_path: # Pfad nicht leer
                        return ('MOVE', self.current_path[0][0], self.current_path[0][1])
                    else: # Leerer Pfad
                        return ('WAIT',)
                else: # Kein Pfad/Lager
                    self.state = AGENT_STATE_PATHFINDING_FAILED
                    self.kb.update_agent_pos(self.id, self.x, self.y, self.state)
                    return ('WAIT',)
            else: # Schon unterwegs
                 return('WAIT',)

        # Prio 3: Am Ressourcenziel -> Sammeln
        elif self.state == AGENT_STATE_GOING_TO_RESOURCE and (self.x, self.y) == self.target_pos:
            resource_pos = self.target_pos
            ct = self.env.get_cell_type(self.x, self.y); rd = self.env.get_resource_details(self.x, self.y)
            # Korrigierter Block: Sammeln starten
            if ct == CELL_RESOURCE and rd and rd.get('quantity', 0) > 0:
                self.state = AGENT_STATE_COLLECTING
                self.kb.update_agent_pos(self.id, self.x, self.y, self.state)
                self.target_pos = None
                return ('COLLECT', self.x, self.y)
            # Ende Korrektur
            else: # Ressource nicht (mehr) da
                self._release_claim_if_held(resource_pos);
                if resource_pos: self.kb.remove_resource_info(resource_pos[0], resource_pos[1], f"DECIDE_{self.id}"); self.kb.update_map(resource_pos[0], resource_pos[1], CELL_EMPTY, f"DECIDE_{self.id}")
                self.state = AGENT_STATE_EXPLORING; self.kb.update_agent_pos(self.id, self.x, self.y, self.state); self.target_pos = None; return self.decide()

        # Prio 4: Pfadfehler / Neuberechnung
        elif self.state == AGENT_STATE_GOING_TO_RESOURCE and self.target_pos and not self.current_path:
             path = self._find_path((self.x,self.y), self.target_pos)
             if path is not None:
                 self.current_path = path; self.pathfinding_attempts = 0
                 if self.current_path: return ('MOVE', self.current_path[0][0], self.current_path[0][1])
                 else: return ('WAIT',) # Leerer Pfad
             else: # Pfad auch jetzt nicht gefunden
                self.pathfinding_attempts += 1
                if self.pathfinding_attempts >= 3:
                    self._release_claim_if_held(); self.target_pos = None; self.state = AGENT_STATE_EXPLORING; self.kb.update_agent_pos(self.id, self.x, self.y, self.state); self.pathfinding_attempts = 0; return self._decide_explore()
                else:
                    self.state = AGENT_STATE_PATHFINDING_FAILED; self.kb.update_agent_pos(self.id, self.x, self.y, self.state); return ('WAIT',)

        # Prio 5: Exploring/Idle -> Unbeanspruchte Ressource suchen & claimen
        elif self.state == AGENT_STATE_EXPLORING or self.state == AGENT_STATE_IDLE:
            unclaimed_pos, _ = self.kb.get_closest_unclaimed_resource(self.x, self.y, self.id)
            path = None
            if unclaimed_pos:
                path = self._find_path((self.x,self.y), unclaimed_pos)

            # Korrigierter Block für Prio 5 (SyntaxError Zeile ~363)
            if unclaimed_pos and path is not None: # Ziel gefunden UND Pfad dorthin existiert
                self._claim_resource(unclaimed_pos) # Ressource beanspruchen
                self._start_following_path(path, unclaimed_pos, AGENT_STATE_GOING_TO_RESOURCE) # Pfadverfolgung starten
                # Entscheide nächsten Zug basierend auf dem neuen Pfad
                next_move = ('WAIT',) # Standard: Warten
                if self.current_path: # Prüfe, ob Pfad nicht leer ist
                    path_step = self.current_path[0]
                    next_move = ('MOVE', path_step[0], path_step[1])
                return next_move # Gib den nächsten Schritt (oder WAIT) zurück
            else: # Nichts gefunden oder kein Pfad -> Erkunden
                 return self._decide_explore()
            # Ende Korrektur Prio 5

        # Fallback
        return ('WAIT',)

    # --- Hilfsmethoden für Decide (KORRIGIERT) ---
    def _start_following_path(self, path, target, next_state):
        """ Setzt Pfad, Ziel, Zustand und resetet Pfadfindungsversuche. """
        self.current_path = path
        self.target_pos = target
        self.state = next_state
        self.kb.update_agent_pos(self.id, self.x, self.y, self.state)
        self.pathfinding_attempts = 0 # Reset counter

    def _claim_resource(self, pos):
        """ Sendet Claim-Nachricht und setzt Claim lokal in KB. """
        # print(f"  Agent {self.id} claiming resource {pos}.") # Optional Debug
        msg_content = {'pos': pos}
        # Nachricht für Broadcast vorbereiten
        self.message_outbox.append(
            Message(self.id, BROADCAST, MSG_TYPE_RESOURCE_CLAIM, msg_content, self.current_step)
        )
        # Claim auch sofort lokal in KB setzen
        self.kb.add_claim(pos, self.id, self.current_step)

    def _release_claim(self, pos):
        """ Sendet Release-Nachricht und entfernt Claim lokal (wenn er vom Agenten stammt). """
        claim_info = self.kb.get_claim(pos)
        # Nur eigenen Claim freigeben
        if claim_info and claim_info['agent_id'] == self.id:
            # print(f"  Agent {self.id} releasing claim on {pos}.") # Optional Debug
            msg_content = {'pos': pos}
            # Nachricht für Broadcast vorbereiten
            self.message_outbox.append(
                Message(self.id, BROADCAST, MSG_TYPE_RESOURCE_RELEASE, msg_content, self.current_step)
            )
            # Claim aus KB entfernen
            self.kb.remove_claim(pos, self.id)

    def _release_claim_if_held(self, specific_pos=None):
        """ Hilfsmethode zum Freigeben des Claims auf das aktuelle Ziel (oder eine spezifische Position). """
        pos_to_release = specific_pos if specific_pos else self.target_pos
        if pos_to_release:
            self._release_claim(pos_to_release) # Ruft die korrigierte Methode auf

    def _decide_explore(self):
        """ Wählt einen Zug zum Erkunden (bevorzugt unbekannte Nachbarn). """
        self.state = AGENT_STATE_EXPLORING; self.kb.update_agent_pos(self.id, self.x, self.y, self.state); self.target_pos = None; self.current_path.clear(); unknown_adj = []; possible_moves = []; empty_adj = []
        # Prüfe Nachbarn
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nx,ny=self.x+dx,self.y+dy; kct=self.kb.get_known_map_tile(nx,ny); # Beachtet Agenten
            if kct != CELL_OBSTACLE:
                possible_moves.append((nx,ny));
                if kct == CELL_UNKNOWN: unknown_adj.append((nx,ny));
                elif kct == CELL_EMPTY: empty_adj.append((nx,ny));
        # Wähle Ziel: Unbekannt > Leer > Andere
        if unknown_adj: target = random.choice(unknown_adj); return ('MOVE', target[0], target[1])
        elif empty_adj: target = random.choice(empty_adj); return ('MOVE', target[0], target[1])
        elif possible_moves: target = random.choice(possible_moves); return ('MOVE', target[0], target[1])
        else: return ('WAIT',) # Blockiert

    # --- Handeln (KORRIGIERT) ---
    def act(self, decision):
        """ Führt die beschlossene Aktion aus. """
        atype = decision[0]

        if atype == 'MOVE':
            _, nx, ny = decision
            # Ist dieser Zug Teil des aktuellen Pfades?
            is_path_move = bool(self.current_path and self.current_path[0] == (nx,ny))
            moved = self.env.move_agent(self.id, nx, ny) # Bewegung versuchen

            if moved: # Erfolgreich bewegt
                self.x, self.y = nx, ny
                self.kb.update_agent_pos(self.id, self.x, self.y, self.state)
                # Wenn auf Pfad, entferne ersten Schritt
                if is_path_move:
                    # --- BEGINN KORRIGIERTER try/except Block ---
                    try:
                        # Entferne den gerade ausgeführten Schritt vom Anfang des Pfades
                        self.current_path.popleft()
                    except IndexError:
                        # Sollte nicht passieren, wenn is_path True war,
                        # aber sicherheitshalber abfangen, falls Pfad unerwartet leer.
                        pass # Ignoriere den Fehler
                    # --- ENDE KORRIGIERTER try/except Block ---
            else: # Bewegung fehlgeschlagen
                if is_path_move:
                    # print(f"  Agent {self.id} move on path to {nx, ny} failed! Clearing path.") # Optional Debug
                    self._release_claim_if_held() # Wichtig: Claim releasen!
                    self.current_path.clear()
                    # Zustand bleibt GOING_TO..., damit decide() neu plant/aufgibt
                # KB über Hindernis informieren (wenn kein anderer Agent)
                act_type = self.env.get_cell_type(nx,ny)
                is_agent = any(i!=self.id and info.get('pos')==(nx,ny) for i,info in self.kb.agent_locations.items())
                if not is_agent and act_type == CELL_OBSTACLE:
                     k_type = self.kb.get_known_map_tile(nx,ny)
                     self.kb.update_map(nx, ny, CELL_OBSTACLE, f"ACT_{self.id}", self.current_step)
                     # Nachricht nur senden, wenn neu als Hindernis erkannt
                     if k_type != CELL_OBSTACLE:
                          msg={'pos':(nx,ny),'type':CELL_OBSTACLE}
                          self.message_outbox.append(Message(self.id, BROADCAST, MSG_TYPE_MAP_TILE, msg, self.current_step))

        elif atype == 'COLLECT':
            res_pos = (self.x,self.y) # Aktuelle Position ist Ressourcenposition
            collected, res_type, remaining = self.env.collect_resource(self.x,self.y,1)
            if collected > 0: # Erfolgreich gesammelt
                self.carrying_resource = {'type':res_type,'amount':collected}
                # KB über Menge informieren
                self.kb.update_resource_quantity(res_pos[0],res_pos[1],collected,self.current_step)
                # Wenn Ressource dadurch erschöpft wurde
                if remaining <= 0:
                    # print(f"  Agent {self.id} depleted resource at {res_pos}.") # Optional Debug
                    # Depleted-Nachricht senden
                    msg_dep={'pos':res_pos}; self.message_outbox.append(Message(self.id, BROADCAST, MSG_TYPE_RESOURCE_DEPLETED, msg_dep, self.current_step))
                    # Claim freigeben (falls gehalten)
                    self._release_claim_if_held(res_pos)
                # Nach erfolgreichem Sammeln -> Zustand IDLE (decide wird Transport starten)
                self.state = AGENT_STATE_IDLE
                self.kb.update_agent_pos(self.id, self.x, self.y, self.state)
            else: # Sammeln fehlgeschlagen (Ressource war leer)
                # print(f"  Agent {self.id} tried collect at {res_pos}, but empty.") # Optional Debug
                self._release_claim_if_held(res_pos) # Claim freigeben
                # KB informieren & Depleted-Nachricht senden (falls vorher bekannt)
                known = (res_pos[0],res_pos[1]) in self.kb.resource_locations
                self.kb.remove_resource_info(res_pos[0],res_pos[1],f"ACT_{self.id}"); self.kb.update_map(res_pos[0],res_pos[1],CELL_EMPTY,f"ACT_{self.id}")
                if known:
                     msg_dep={'pos':res_pos}; self.message_outbox.append(Message(self.id, BROADCAST, MSG_TYPE_RESOURCE_DEPLETED, msg_dep, self.current_step))
                # Zurück zum Erkunden
                self.state = AGENT_STATE_EXPLORING
                self.kb.update_agent_pos(self.id, self.x, self.y, self.state)

        elif atype == 'DEPOSIT':
            _, sx, sy = decision
            if self.carrying_resource:
                res_t,amo = self.carrying_resource['type'],self.carrying_resource['amount']
                success = self.env.deposit_resource(sx,sy,res_t,amo)
                if success:
                    self.carrying_resource = None # Inventar leeren
                else:
                    print(f"  Agent {self.id} FAILED deposit at ({sx},{sy}).") # Sollte nicht passieren
            # Nach Deposit (erfolgreich oder nicht) -> Erkunden
            self.state = AGENT_STATE_EXPLORING
            self.kb.update_agent_pos(self.id, self.x, self.y, self.state)

        elif atype == 'WAIT':
             pass # Nichts tun

# --- Visualizer Klasse ---
class Visualizer:
    def __init__(self, width, height, cell_size):
        pygame.init()
        self.width = width; self.height = height; self.cell_size = cell_size; self.info_height = INFO_HEIGHT
        screen_width = self.width * self.cell_size; screen_height = self.height * self.cell_size + self.info_height
        self.screen = pygame.display.set_mode((screen_width, screen_height)); pygame.display.set_caption("Multi-Agent Resource Collection")
        try: self.font_small = pygame.font.SysFont(None, 18); self.font_medium = pygame.font.SysFont(None, 24); self.font_info = pygame.font.SysFont(None, 20)
        except: self.font_small = pygame.font.Font(None, 18); self.font_medium = pygame.font.Font(None, 24); self.font_info = pygame.font.Font(None, 20)
        self.agent_colors = { AGENT_STATE_IDLE: COLOR_GREY, AGENT_STATE_EXPLORING: COLOR_AGENT_EXPLORE, AGENT_STATE_GOING_TO_RESOURCE: COLOR_ORANGE,
                              AGENT_STATE_COLLECTING: COLOR_RED, AGENT_STATE_GOING_TO_STORAGE: COLOR_PURPLE, AGENT_STATE_DEPOSITING: COLOR_RED, AGENT_STATE_PATHFINDING_FAILED: COLOR_BLACK }

    def _get_color(self, cell_type):
        if cell_type == CELL_OBSTACLE: return COLOR_DARKGREY
        if cell_type == CELL_RESOURCE: return COLOR_GREEN
        if cell_type == CELL_STORAGE: return COLOR_BROWN
        if cell_type == CELL_EMPTY: return COLOR_LIGHTBLUE
        if cell_type == CELL_UNKNOWN: return COLOR_GREY
        return COLOR_BLACK

    def draw(self, sim_kb, sim_env, agents, current_step):
        try:
            self.screen.fill(COLOR_BLACK)
            info_rect = pygame.Rect(0, 0, self.width*self.cell_size, self.info_height); pygame.draw.rect(self.screen, COLOR_DARKGREY, info_rect)
            step_text = self.font_medium.render(f"Step: {current_step}", True, COLOR_WHITE); self.screen.blit(step_text, (10, 5))
            total_stored = sum(inv.get(RESOURCE_TYPE_DEFAULT, 0) for inv in sim_env.storage_inventory.values()); stored_text = self.font_medium.render(f"Stored: {total_stored}", True, COLOR_WHITE); self.screen.blit(stored_text, (10, 30))
            num_known_res = len(sim_kb.get_known_resources()); res_text = self.font_medium.render(f"Known Res: {num_known_res}", True, COLOR_WHITE); self.screen.blit(res_text, (150, 5))
            num_claims = len(sim_kb.resource_claims); claims_text = self.font_medium.render(f"Claims: {num_claims}", True, COLOR_RED if num_claims > 0 else COLOR_WHITE ); self.screen.blit(claims_text, (150, 30))

            resource_coords = sim_kb.get_known_resources(); storage_coords = set(sim_kb.storage_locations); claim_coords = set(sim_kb.resource_claims.keys())
            for r in range(sim_kb.height):
                for c in range(sim_kb.width):
                    pos = (c, r); cell_rect = pygame.Rect(c*self.cell_size, r*self.cell_size+self.info_height, self.cell_size, self.cell_size)
                    known_type = sim_kb.map_data[r][c]; is_res = pos in resource_coords; is_store = pos in storage_coords
                    if is_res: known_type = CELL_RESOURCE
                    if is_store: known_type = CELL_STORAGE
                    color = self._get_color(known_type); pygame.draw.rect(self.screen, color, cell_rect); pygame.draw.rect(self.screen, COLOR_DARKGREY, cell_rect, 1)
                    if pos in claim_coords and is_res: pygame.draw.rect(self.screen, COLOR_CLAIM_BORDER, cell_rect, 2)

            agent_radius = self.cell_size // 2 - 3
            for agent in agents:
                agent_rect = pygame.Rect(agent.x*self.cell_size, agent.y*self.cell_size+self.info_height, self.cell_size, self.cell_size)
                center_x = agent_rect.left + self.cell_size // 2; center_y = agent_rect.top + self.cell_size // 2
                agent_color = self.agent_colors.get(agent.state, COLOR_WHITE); pygame.draw.circle(self.screen, agent_color, (center_x, center_y), agent_radius)
                if agent.carrying_resource: pygame.draw.circle(self.screen, COLOR_YELLOW, (center_x, center_y), agent_radius // 2 + 1)

            pygame.display.flip()
        except Exception as e: print(f"Visualization error: {e}")

    def close(self): pygame.quit()

# --- Simulation ---
class Simulation:
    def __init__(self, width, height, num_agents, num_resources, num_storage, cell_size=CELL_SIZE, use_visualizer=True):
        self.env = Environment(width, height, num_resources, num_storage); self.kb = KnowledgeBase(width, height, self.env.storage_points); self.broker = MessageBroker(); self.agents = []; starts = set(); self.current_step = 0; self.visualizer = None
        if use_visualizer:
            try: self.visualizer = Visualizer(width, height, cell_size)
            except Exception as e: print(f"Pygame Visualisierung fehlerhaft: {e}. Keine Visualisierung."); self.visualizer = None
        for i in range(num_agents):
            added = False; attempts = 0; max_attempts = width * height
            while attempts < max_attempts: # Korrigierte Schleife
                sx, sy = random.randint(0, width - 1), random.randint(0, height - 1)
                if self.env.grid[sy][sx] == CELL_EMPTY and (sx, sy) not in starts:
                    starts.add((sx, sy)); self.agents.append(Agent(f"A{i+1}", sx, sy, self.env, self.kb, self.broker)); added = True; break
                attempts += 1
            if not added: print(f"Warnung: Start für Agent {i+1} nicht gefunden.")

    def run_step(self):
        self.current_step += 1; random.shuffle(self.agents)
        for agent in self.agents: agent.set_step(self.current_step); agent.sense(); agent.communicate(); decision = agent.decide(); agent.act(decision)
        if self.visualizer: self.visualizer.draw(self.kb, self.env, self.agents, self.current_step)

    def run(self, num_steps):
        t_start = time.time(); running = True; i = 0
        clock = pygame.time.Clock()

        if self.visualizer: self.visualizer.draw(self.kb, self.env, self.agents, self.current_step); print("Visualizer aktiv. Fenster schließen zum Beenden.")
        else: print("Initial State:"); self.print_environment_map(); self.print_kb_map_overview()

        while running and i < num_steps:
            if self.visualizer:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT: running = False; print("Fenster geschlossen."); break
                if not running: break
            self.run_step(); i += 1
            if not self.visualizer:
                 known_res=bool(self.kb.get_known_resources()); claimed=bool(self.kb.resource_claims); carrying=any(a.carrying_resource for a in self.agents); working=any(a.state not in [AGENT_STATE_EXPLORING,AGENT_STATE_IDLE] for a in self.agents); unknown=any(CELL_UNKNOWN in r for r in self.kb.map_data)
                 if not known_res and not claimed and not carrying and not working:
                      if not unknown: print(f"\nWork done & map explored at step {i}. Stop."); running=False; break
                      elif i > num_steps*0.1: print(f"\nNo more work found at step {i}. Stop."); running=False; break
            if self.visualizer: clock.tick(60) # Framerate limit

        t_end = time.time(); print("\nSimulation Finished."); print(f"Finished at step {i} after {t_end - t_start:.2f} sec."); print("Final Storage Inventory:", self.env.storage_inventory); self.print_kb_map_overview()
        if self.visualizer: self.visualizer.close()

    def print_agent_states(self): # Fallback Print Methode
        print("Agent States & Positions:")
        claims = {p: i['agent_id'] for p,i in self.kb.resource_claims.items()}
        for a in self.agents:
            pos=(a.x,a.y); 
            st=a.state; 
            carry=a.carrying_resource; 
            targ=a.target_pos; 
            path=len(a.current_path); 
            claim=targ if targ in claims and claims[targ]==a.id else None; 
            cs=f" Claims:{claim}" if claim else ""
            print(f"  {a.id}: St={st}, Pos={pos}, Carry={carry}, Target={targ}{cs}, Path={path}")
    def print_kb_map_overview(self): # Fallback Print Methode
        print("KB Map (U:Unk, .:Empty, #:Obst, R:Res, S:Store, A:Agent, C:Claimed R):")
        map_str=""; ac={i.get('pos'):id for id,i in self.kb.agent_locations.items() if i.get('pos')}; rc=set(self.kb.get_known_resources().keys()); sc=set(self.kb.storage_locations); clc = set(self.kb.resource_claims.keys())
        for r in range(self.kb.height):
            row_str = ""
            for c in range(self.kb.width):
                 p=(c,r); is_a=p in ac; is_r=p in rc; is_s=p in sc; is_c=p in clc; cell = self.kb.map_data[r][c]
                 if is_a: row_str += "A"
                 elif is_c and is_r: row_str += "C"
                 elif is_r : row_str += "R"
                 elif is_s : row_str += "S"
                 elif cell == CELL_UNKNOWN: row_str += "?"
                 elif cell == CELL_EMPTY: row_str += "."
                 elif cell == CELL_OBSTACLE: row_str += "#"
                 else: row_str += "."
            map_str += row_str + "\n"
        print(map_str)
    def print_environment_map(self): # Fallback Print Methode
        print("Actual Map (.=Empty, #=Obst, R:Res, S:Store):")
        map_str="";
        for r in range(self.env.height):
            row_str = ""
            for c in range(self.env.width):
                ct=self.env.get_cell_type(c,r); rd=self.env.get_resource_details(c,r); q=rd.get('quantity',0) if rd else 0
                if ct == CELL_EMPTY: row_str += "."
                elif ct == CELL_OBSTACLE: row_str += "#"
                elif ct == CELL_RESOURCE and q > 0: row_str += "R"
                elif ct == CELL_STORAGE: row_str += "S"
                elif ct == CELL_RESOURCE and q <= 0: row_str += "."
                else: row_str += "X"
            map_str += row_str + "\n"
        print(map_str)

# --- Hilfsfunktion für Agent ---
Agent.get_initial_pos = lambda self: (self.x, self.y) # Muss nach Agent def stehen

# --- Hauptprogramm ---
if __name__ == "__main__":
    # Parameter anpassen für Tests
    GRID_WIDTH = 30
    GRID_HEIGHT = 20
    NUM_AGENTS = 10
    NUM_RESOURCES = 40
    NUM_STORAGE = 3
    CELL_PIXEL_SIZE = 20
    MAX_STEPS = 2000
    USE_VISUALIZER = True # Auf False setzen für reine Textausgabe

    # Simulation erstellen und starten
    sim = Simulation(
        width=GRID_WIDTH, height=GRID_HEIGHT, num_agents=NUM_AGENTS,
        num_resources=NUM_RESOURCES, num_storage=NUM_STORAGE,
        cell_size=CELL_PIXEL_SIZE, use_visualizer=USE_VISUALIZER
    )
    sim.run(num_steps=MAX_STEPS)