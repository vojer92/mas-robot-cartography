# Projektarchitektur: mas-robot-cartography

Dieses Dokument beschreibt die Architektur unseres Projekts zur Erkundung eines unbekannten Gebiets mit autonomen Robotern. Die Simulation basiert auf der Mesa-Bibliothek (Version 3) und nutzt eine modulare, erweiterbare Architektur, die sich an Best Practices orientiert.

## Überblick

Die Architektur folgt einem Schichtenmodell, kombiniert mit einem Factory- und Repository-Muster. Dies fördert die Trennung von Verantwortlichkeiten, erleichtert den Austausch von Algorithmen und erlaubt reproduzierbare, vergleichbare Simulationen.

---

## Verzeichnisstruktur

```
mas-robot-cartography/
├── agents/
│   ├── base_agent.py           # Basisklasse für Agenten
│   ├── explorer_agent.py       # Agent mit Explorationsverhalten
│   └── factory.py              # Agent-Factory für flexible Instanziierung
│
├── algorithms/
│   ├── astar.py                # Pfadfindung mit A*
│   ├── frontier.py             # Frontier-basierte Exploration
│   ├── random_walk.py          # Zufällige Exploration
│   └── repository.py           # Algorithmus-Repository (Mapping Name → Funktion)
│
├── model/
│   └── robot_model.py          # Enthält Model-Klasse mit Grid, Scheduler, DataCollector
│
├── visualization/
│   └── server.py               # Setup für ModularServer, CanvasGrid, ChartModule
│
├── batch/
│   └── batch_runner.py         # Automatisierte Mehrfachläufe (ohne GUI)
│
├── utils/
│   ├── metrics.py              # Berechnung von Metriken (z. B. erkundete Fläche)
│   └── seed.py                 # Zufallsseed-Handling
│
├── data/
│   └── ergebnisse/             # Ergebnisse und CSV-Dateien der Batch-Simulationen
│
├── examples/                   # Beispiele
│
├── docs/                       # Dokumentation
│
├── main_gui.py                 # Startet die Web-Visualisierung (ModularServer)
├── main_batch.py               # Startet automatisierte Vergleichssimulationen
├── requirements.txt            # Abhängigkeiten
└── README.md
```

---

## Architekturprinzipien

### 1. **Schichtenmodell** (MVC-ähnlich)

- **Model**: Beinhaltet das Grid, Agenten, Scheduler, und Metriken (robot_model.py)
- **Agenten**: Verhalten der Agenten ist modular aufgebaut, Factory-basiert
- **Visualisierung**: Getrennt in `server.py` (GUI), mit interaktiven Parametern und Chart-Modulen

### 2. **Factory Pattern (Agenten)**

- Alle Agenten werden über `factory.py` erstellt
- Ermöglicht die einfache Erweiterung mit neuen Agententypen oder Startparametern

### 3. **Repository Pattern (Algorithmen)**

- `repository.py` liefert je nach Name den gewünschten Pfadfindungs-/Explorationsalgorithmus
- Trennung von Verhalten (Agent) und Strategie (Algorithmus)

### 4. **Batch-fähige Ausführung**

- `batch_runner.py` führt viele Simulationen mit unterschiedlichen Algorithmen durch
- Ergebnisse werden als CSV gespeichert und mit `pandas` ausgewertet

### 5. **Optional: Tests und Konfigurierbarkeit**

- Erweiterbar um `tests/` für Unit-Tests und `config/` für YAML-basierte Parameter

---

## Beispiel: Algorithmuswechsel

```python
from algorithms.repository import AlgorithmRepository

class RobotModel(Model):
    def __init__(self, algorithm_name: str):
        self.pathfinder = AlgorithmRepository.get_pathfinder(algorithm_name)

class ExplorerAgent(Agent):
    def step(self):
        path = self.model.pathfinder(self.pos, self.goal, self.model.grid)
```

---

## Verwendung

- Zum Starten mit Visualisierung:

  ```bash
  python main_gui.py
  ```

- Zum automatisierten Vergleich:
  ```bash
  python main_batch.py
  ```

---

## Ziel

Diese Architektur soll es uns ermöglichen, verschiedene Explorationstechniken vergleichbar zu testen, modular weiterzuentwickeln und die Ergebnisse klar zu dokumentieren. Bei Fragen oder Erweiterungsvorschlägen gerne melden.
