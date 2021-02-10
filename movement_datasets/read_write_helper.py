import numpy as np
import pandas as pd


def read_fep_dataset(fep_movement_dataset_path, dtype):
    """"""
    pd_dataset = pd.read_csv(fep_movement_dataset_path, sep=' ', header=None, dtype=dtype)

    print("\nSample of the first 10 lines of the FEP movement dataset:")
    print(pd_dataset.head(10))

    pos = pd_dataset.iloc[:, 0:7].to_numpy(dtype=dtype)
    vel = pd_dataset.iloc[:, 7:14].to_numpy(dtype=dtype)
    acc = pd_dataset.iloc[:, 14:21].to_numpy(dtype=dtype)
    tau = pd_dataset.iloc[:, 21:28].to_numpy(dtype=dtype)

    return pos, vel, acc, tau


def write_fep_dataset(fep_sim_output_dataset_path, datasets, dtype):
    """"""
    np_dataset = np.concatenate(datasets, axis=1, dtype=dtype)
    pd_dataset = pd.DataFrame(data=np_dataset, dtype=dtype)

    print("\nSample of the first 10 lines of the created dataset:")
    print(pd_dataset.head(10))

    pd_dataset.to_csv(fep_sim_output_dataset_path, sep=' ', index=False, header=False)
