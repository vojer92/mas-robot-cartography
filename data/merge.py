import glob

import pandas as pd


def main():
    # get all filespaths
    filepaths = glob.glob("*[Ss][Ee][Ee][Dd]*.csv")
    print(filepaths)

    # read csv's
    dataframes = [pd.read_csv(path, sep=";") for path in filepaths]

    for i, df in enumerate(dataframes): 
        print(f"Max Explored: {max(df["Explored"])} Filename: {filepaths[i]}")

    # concat csv's
    df = pd.concat(dataframes, ignore_index=True)

    # drop columns who are not needed
    df = df.drop(columns=["RunId", "iteration", "seed"])

    # aggregate all seeds
    df = df.groupby(
        [
            "Step",
            "initial_no_robots",
            "grid_size",
            "robot_type_str",
            "view_radius",
            "view_angle",
            "factor_distance",
            "factor_size",
        ]
    ).agg(
        {
            "Explored": ["min", "max", "mean", "median", "std"],
            "Explored_fields": ["min", "max", "mean", "median", "std"],
            "Step_Count_All_Agents": ["min", "max", "mean", "median", "std"],
        }
    )

    # flatten dataframe
    df.columns = ["_".join(col).strip() for col in df.columns.values]
    df = df.reset_index()

    print(f"Explored max after aggregation: {max(df["Explored_max"])}")
    print(f"shape: {df.shape}")
    # df.to_csv("Results-aggregated.csv", sep=";", index=False)


if __name__ == "__main__":
    main()
