# 🛰️ mas-robot-cartography

Ein Multiagentensystem zur Kartographierung eines unbekannten Gebiets mit autonomen Robotern. Die Simulation basiert auf der [Mesa](https://mesa.readthedocs.io/en/stable/) (TODO: Steht noch nicht fest!) Agenten-Framework und ist Teil eines Informatikprojekts zur Erforschung verteilter Intelligenz.

---

## 🚀 Projekt starten

### Voraussetzungen

- **Python 3.11** oder höher
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
python server/server.py # TODO: Wird sich je nach Struktur noch ändern!
```

---

## 💡 Git & GitHub Basics

### 📥 Projekt klonen (wenn ihr beitretet)

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
- 4 Leerzeichen Einrückung
- `f-Strings` statt `%` oder `.format()`

---

## 🧪 Tests (TODO: je nachdem, ob wir testing betreiben wollen)

Einfaches Testen mit `pytest`:

```bash
pytest
```

---

## 📁 Projektstruktur

```
mas-robot-cartography/
├── agents/                # Agentenklassen
├── model/                 # Mesa Model & Logik
├── server/                # Webserver & Darstellung
├── simulation/            # Szenarien & Parameter
├── tests/                 # Unit-Tests
├── docs/                  # Dokumentation
├── examples/              # Beispiele Mesa
├── requirements.txt       # Abhängigkeiten
├── .gitignore             # Git-Ignore
└── README.md              # Diese Datei

# TODO: Über die Struktur wird noch entschieden. Wurde nur als Platzhalter angelegt.
```

---

## 📜 Lizenz

MIT – Feel free to use, extend and contribute! 🚀
