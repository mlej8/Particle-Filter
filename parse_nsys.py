
import re
from pathlib import Path
import pandas as pd
import seaborn as sns

row_interest = {
    "particle_filter(Robot*, double*, double, double, int, double const*, int, double const*)": "Kernel Time",
}

# Method is just an arbirary name for the name of the program running
nsys_columns = "Method Time(%)  TotalTime(ns)  Count  Average(ns)  Minimum(ns)  Maximum(ns)           Operation".split()

result = pd.DataFrame(columns=nsys_columns)
files = Path("reports").glob("*.txt")
for file in files:
    f = open(file).read().split("\n")
    for i, e in enumerate(f):
        for row in row_interest:
            if row in e:
                res = {}
                res['Method'] = " ".join(file.stem.split("_")[:-2])
                subs = e.replace(row, "").split()
                subs.append(row_interest[row])
                if len(subs) != len(nsys_columns)-1:
                    continue
                result = result.append(
                    {**res, **dict(zip(nsys_columns[1:], subs))}, ignore_index=True)

r = result.sort_values(by=["Method", 'Operation'])
r.to_csv("Profiling/parallel.csv", index=False)
# df = pd.read_csv("parallel.csv", thousands=",")
# df['Method'] = df['Method'].apply(lambda x: " ".join(x.split()[:-2]))
# df = df.groupby(['Method', 'Operation']).mean().reset_index()
# df.to_csv("summary.csv", index=False)