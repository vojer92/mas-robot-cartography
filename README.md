# 🛰️ Robot Cartography Simulation

Ein Multiagentensystem zur Kartographierung eines unbekannten Gebiets mit autonomen Robotern. Die Simulation basiert auf der [Mesa](https://mesa.readthedocs.io/en/stable/) Agenten-Framework und ist Teil eines Informatikprojekts zur Erforschung verteilter Intelligenz.

---

## 🚀 Projekt starten

### Voraussetzungen
- Python 3.11 oder höher
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

### Server starten
```bash
python server/server.py
```

---

## 💡 GitHub Basics

### Projekt klonen
```bash
git clone https://github.com/DEIN-USERNAME/mas-robot-cartography.git
cd robot-cartography-sim
```

### Neues Feature entwickeln
```bash
git checkout -b feature/kartographie-agent
# Änderungen machen, dann:
git add .
git commit -m "feat(agent): kartographiert Umgebung durch Rasteranalyse"
git push origin feature/kartographie-agent
```

### Änderungen vom Team holen
```bash
git pull origin main
```

### Änderungen zusammenführen (via GitHub GUI)
1. Auf GitHub unter "Pull Requests" neuen PR öffnen
2. Reviewer zuweisen
3. Nach Freigabe: `Merge` in `main` klicken

---

## 🧑‍💻 Code Guideline

### ✅ Docstrings (PEP 257 konform)
Verwendet **dreifache Anführungszeichen** (`"""`) für Funktionen, Methoden und Klassen. Beispiel:

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
Alle Funktionen und Variablen sollten mit Typen versehen werden.

```python
x: int = 5
name: str = "Robo"

def add(a: float, b: float) -> float:
    return a + b
```

### ✅ Weitere Konventionen
- Snake_case für Variablen und Funktionen
- PascalCase für Klassen
- 4 Leerzeichen als Einrückung
- Nutzt f-Strings statt `%` oder `str.format`

---

## 🧪 Tests

Schreibt einfache Unit-Tests mit `pytest`:
```bash
pytest
```

---

## 📁 Projektstruktur

```
robot-cartography-sim/
├── agents/                # Agentenklassen
├── model/                 # Mesa Model & Logik
├── server/                # Webserver & Darstellung
├── simulation/            # Szenarien & Parameter
├── tests/                 # Unit-Tests
├── docs/                  # Dokumentation
├── requirements.txt       # Abhängigkeiten
├── .gitignore             # Git-Ignore
└── README.md              # Diese Datei
```

---

## 📜 Lizenz

MIT – Feel free to use, extend and contribute! 🚀
