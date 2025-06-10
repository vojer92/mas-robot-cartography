from matplotlib.markers import MarkerStyle
from mesa.experimental.devs import ABMSimulator
from mesa.visualization import (
    CommandConsole,
    Slider,
    SolaraViz,
    make_plot_component,
    make_space_component,
)

from agents.ground import Ground
from agents.obstacle import Obstacle
from agents.random_walk_robot import RandomWalkRobot
from model import Exploration

# Pre-compute markers for different angles (e.g., every 5 degrees)
MARKER_CACHE = {}
for angle in range(0, 360, 5):
    marker = MarkerStyle(10)
    marker._transform = marker.get_transform().rotate_deg(angle)
    MARKER_CACHE[angle] = marker


def exploration_portrayal(agent):
    if agent is None:
        return

    portrayal = {
        "size": 25,
    }

    if isinstance(agent, RandomWalkRobot):
        portrayal["color"] = "tab:red"
        portrayal["marker"] = MARKER_CACHE[round((agent.orientation + 360) % 360)]
        portrayal["zorder"] = 2
        portrayal["size"] = 85
    elif isinstance(agent, Ground):
        if agent.explored:
            portrayal["color"] = "tab:green"
            portrayal["marker"] = "s"
            portrayal["size"] = 85
        else:
            portrayal["color"] = "#DDDDDD"
            portrayal["marker"] = "s"
            portrayal["size"] = 85
    elif isinstance(agent, Obstacle):
        portrayal["color"] = "#000000"
        portrayal["marker"] = "s"
        portrayal["size"] = 100

    return portrayal


model_params = {
    "seed": {
        "type": "InputText",
        "value": 42,
        "label": "Random Seed",
    },
    "initial_random_walk_robot": Slider("Random Walk Robot Population", 0, 0, 20),
    "view_radius": Slider("View Radius", 1, 1, 5),
    "view_angle": Slider("View Angle", 180, 45, 360, 45),
    "view_resulution": Slider("View Resulution", 32, 10, 62),
}


def post_process_space(ax):
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])


def post_process_lines(ax):
    ax.legend(loc="center left", bbox_to_anchor=(1, 0.9))


space_component = make_space_component(
    exploration_portrayal, draw_grid=False, post_process=post_process_space
)
lineplot_component = make_plot_component(
    {"Explored": "tab:blue"},
    post_process=post_process_lines,
)

simulator = ABMSimulator()
model = Exploration(simulator=simulator)

page = SolaraViz(
    model,
    components=[space_component, lineplot_component, CommandConsole],
    model_params=model_params,
    name="Exploration",
    simulator=simulator,
)
page  # noqa
