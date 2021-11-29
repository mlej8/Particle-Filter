import pandas as pd

# df = pd.read_csv("parallel_manual.csv")
# df = df.groupby(['block_size','num_particles']).mean().reset_index()
# df.to_csv("parallel_manual_summary.csv", index=False)

df = pd.read_csv("sequential.csv")
df = df.groupby(['num_particles']).mean().reset_index()
df.to_csv("sequential_summary.csv", index=False)
