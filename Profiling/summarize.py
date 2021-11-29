import pandas as pd

df = pd.read_csv("parallel_manual.csv")
df = df.groupby(['block_size','num_particles']).mean().reset_index()
df.to_csv("parallel_manual_summary.csv", index=False)
