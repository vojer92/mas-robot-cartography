import csv
from datetime import datetime

import pandas as pd
from mesa.batchrunner import batch_run

from model import Exploration

MAX_STEPS = 1000
SEED = 1

if __name__ == "__main__":
    now = datetime.now()

    params = {
        "initial_no_robots": [1, 3, 6, 9],
        "grid_size": range(40, 101, 20),
        "robot_type_str": ["RandomWalkRobot", "FBERobot"],
        "view_radius": range(1, 4, 1),
        "view_angle": [45, 90, 180, 360],
        "seed": range(1, 401, 1),
        "factor_distance": 1.0,
        "factor_size": [0.0, 0.1, 0.25],
    }

    # params = {
    #     "initial_no_robots": 5,
    #     "grid_size": 100,
    #     "robot_type_str": ["RandomWalkRobot", "FBERobot"],
    #     "view_radius": 1,
    #     "view_angle": 90,
    #     "seed": range(1, 51, 1),
    #     "factor_distance": 0.5,
    #     "factor_size": 0.5,
    # }

    results = batch_run(
        Exploration,
        parameters=params,
        iterations=1,
        max_steps=MAX_STEPS,
        number_processes=None,
        data_collection_period=25,
        display_progress=True,
    )

    df_results = pd.DataFrame.from_dict(results)
    print(df_results)
    df_results.to_csv(f"data/Results-{now.strftime("%d.%m.%Y-%H:%M:%S")}.csv", sep=";", index=False)
