import glob

import pandas as pd


def main():
    # get all filespaths
    filepaths = glob.glob("*fehlerhaft*.csv")
    print(filepaths)

    # read csv's
    dataframes = [pd.read_csv(path, sep=";") for path in filepaths]
    for df in dataframes:
        df["Explored"] = df["Explored_fields"] / df["grid_size"] ** 2 * 100.0
        df.to_csv(f"Results-fixed-seed{df["seed"][0]}.csv", sep=";", index=False)


if __name__ == "__main__":
    main()
