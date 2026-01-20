import os
import pandas as pd
directory = "output"

header = []
data = ";".join(header) + "\n"
for i in range(40):
    if not os.path.exists(os.path.join(directory, f"ctx_{i}_responsibility_discrete_discrete_sampling_results.csv")):
        continue
    # try:
    #     with open(os.path.join(directory, f"ctx_{i}_exhaustive_results.csv"), "r") as file:
    #         data += file.read().rstrip()
    # except FileNotFoundError:
    #     data += str(i) + ";-;-;-;-"
    # data += ";"
    # try:
    #     with open(os.path.join(directory, f"ctx_{i}_exhaustive_observed_results.csv"), "r") as file:
    #         data += ";".join(file.read().rstrip().split(";")[2:])
    # except FileNotFoundError:
    #     data += "-;-;-"
    # data += ";"
    # with open(os.path.join(directory, f"ctx_{i}_responsibility_distance_results.csv"), "r") as file:
    #     data += ";".join(file.read().rstrip().split(";")[0:])
    # data += ";"
    # with open(os.path.join(directory, f"ctx_{i}_responsibility_discrete_results_only_greedy.csv"), "r") as file:
    #     data += ";".join(file.read().rstrip().split(";")[2:])
    # data += ";"
    # with open(os.path.join(directory, f"ctx_{i}_responsibility_distance_results.csv"), "r") as file:
    #     data += ";".join(file.read().rstrip().split(";")[2:])
    with open(os.path.join(directory, f"ctx_{i}_responsibility_discrete_discrete_sampling_results.csv"), "r") as file:
        data += ";".join(file.read().rstrip().split(";")[0:])
    data += "\n"

with open(os.path.join(directory, "combined_responsibility_discrete_discrete_sampling_results.csv"), "w") as file:
    file.write(data)
