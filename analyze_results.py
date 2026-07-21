import os
# import pandas as pd
directory = "output"
header = [
    "ctx_idx", "num_of_obstacles", "explanation", "explanation_size", "runs",
    "explanation_100", "explanation_size_100", "runs_100", "order_100", "explanation_200", "explanation_size_200", "runs_200", "order_200",  "explanation_300", "explanation_size_300", "runs_300", "order_300",
    "greedy_explanation_100", "greedy_explanation_size_100", "greedy_runs_100", "greedy_order_100", "greedy_explanation_200", "greedy_explanation_size_200", "greedy_runs_200", "greedy_order_200",  "greedy_explanation_300", "greedy_explanation_size_300", "greedy_runs_300", "greedy_order_300",
]
data = ";".join(header) + "\n"
for i in range(40):
    if (not os.path.exists(os.path.join(directory, f"ctx_{i}_responsibility_discrete_discrete_sampling_results.csv"))) or (not os.path.exists(os.path.join(directory, f"ctx_{i}_exhaustive_observed_results.csv"))):
        continue
    try:
        with open(os.path.join(directory, f"ctx_{i}_exhaustive_observed_results.csv"), "r") as file:
            data += ";".join(file.read().rstrip().split(";"))
    except FileNotFoundError:
        data += "-;-;-"
    data += ";"
    with open(os.path.join(directory, f"ctx_{i}_responsibility_discrete_discrete_sampling_results.csv"), "r") as file:
        data += ";".join(file.read().rstrip().split(";")[2:])
    data += ";"
    with open(os.path.join(directory, f"ctx_{i}_responsibility_discrete_discrete_sampling_results_only_greedy.csv"), "r") as file:
        data += ";".join(file.read().rstrip().split(";")[2:])
    data += "\n"

with open(os.path.join(directory, "combined_results.csv"), "w") as file:
    file.write(data)
