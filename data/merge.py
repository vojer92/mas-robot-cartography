import glob

import pandas as pd


def main():
    # get all filespaths
    filepaths = glob.glob("*[Ss][Ee][Ee][Dd]*.csv")
    print(filepaths)

    # read csv's
    dataframes = [pd.read_csv(path, sep=";") for path in filepaths]

    # concat csv's
    df = pd.concat(dataframes, ignore_index=True)

    # drop columns who are not needed
    df = df.drop(columns=["RunId", "iteration", "seed"])

    # aggregate all seeds
    df = df.groupby(
        [
            "factor_size",
            "factor_distance",
            "view_angle",
            "view_radius",
            "robot_type_str",
            "grid_size",
            "initial_no_robots",
            "Step",
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

    print(df.head())
    print(f"shape: {df.shape}")
    df.to_csv("Results-aggregated.csv", sep=";", index=False)


if __name__ == "__main__":
    main()
