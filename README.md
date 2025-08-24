# 🛰️ mas-robot-cartography

Ein Multiagentensystem zur Kartographierung eines unbekannten Gebiets mit autonomen Robotern. Die Simulation basiert auf der [Mesa](https://mesa.readthedocs.io/en/stable/) Agenten-Framework und ist Teil eines Informatikprojekts zur Erforschung verteilter Intelligenz.

---

## 🚀 Projekt starten

### Voraussetzungen

- **Python 3.13** oder höher, da bessere Typannotationen möglich
- Virtuelle Umgebung empfohlen:

```bash
python -m venv .venv
source .venv/bin/activate  # Linux/macOS
.venv\Scripts\activate     # Windows
```

### Installation

```bash
pip install -r requirements.txt
```

### Visualisierung starten

```bash
solara run app.py
```

### Batchrunner starten

```bash
python batch.py <start_seed> <end_seed>
```

---

## 💡 Git & GitHub Basics

### 📥 Projekt klonen

```bash
git clone https://github.com/vojer92/mas-robot-cartography.git
cd mas-robot-cartography
```

### 🔀 Neues Feature entwickeln

```bash
git checkout -b feature/kartographie-agent
# Änderungen machen ...
git status
git add .
git commit -m "feat(agent): explorer agent erkennt Hindernisse"
git push --set-upstream origin feature/kartographie-agent
```

### 📤 Änderungen an bestehendem Branch pushen

```bash
git add .
git commit -m "fix(agent): Hindernislogik korrigiert"
git push
```

### 📥 Änderungen vom Team holen

```bash
git pull origin main
```

### 🔄 Branch in `main` mergen (über GitHub)

1. Auf GitHub: Pull Request (PR) öffnen
2. Reviewer zuweisen & kommentieren
3. PR wird nach Freigabe gemerged

### 🧹 Branch lokal löschen (nach Merge)

```bash
git branch -d feature/kartographie-agent
```

---

## 🧑‍💻 Code Guideline

### ✅ Docstrings (PEP 257)

```python
def explore_area(agent: ExplorerAgent, radius: int) -> list[tuple[int, int]]:
    """
    Erkundet das umliegende Gebiet und gibt erkannte Felder zurück.

    Args:
        agent (ExplorerAgent): Der erkundende Agent.
        radius (int): Der Radius der Umgebung, die untersucht wird.

    Returns:
        list[tuple[int, int]]: Koordinaten der erkannten Felder.
    """
    ...
```

### ✅ Typannotationen (PEP 484)

```python
def add(a: float, b: float) -> float:
    return a + b
```

### ✅ Weitere Konventionen

- Snake_case für Variablen und Funktionen
- PascalCase für Klassen
- Snake_case für Dateinamen
- 4 Leerzeichen Einrückung
- `f-Strings` statt `%` oder `.format()`

---

## 📁 Projektstruktur

```
mas-robot-cartography/
├── agents/                     # Agentenklassen
├── algorithms/                 # Algorithmen
├──── movement_goal_finding/    # Methoden zur Findung von Bewegungszielen + Interface + Factory + Enum
├──── movement_goal_selection/  # Methoden zur Auswahl von Bewegungszielen + Interface + Factory + Enum
├──── pathfinding/              # Methoden zur Pfadfindung + Interface + Factory + Enum
├── communication/              # Methoden zur Kommunikation
├── data/                       # Simulationsdaten
├── docs/                       # Dokumentation
├── masks/                      # Zwischenspeicher erkundbare Felder
├── .gitignore                  # Git-Ignore
├── requirements.txt            # Abhängigkeiten
├── model.py                    # Modell
├── app.py                      # Simulationsvisualisierung
├── batch.py                    # Batchrunner
└── README.md                   # Diese Datei
```

---

## 📜 Lizenz

MIT – Feel free to use, extend and contribute! 🚀
