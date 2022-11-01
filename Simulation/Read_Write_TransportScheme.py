import pandas as pd

DIR      = "C:/Users/MunckJande/OneDrive - PostNL/Documenten/Projecten/Robots_at_Xdocks/"
FILE     = "Transportschema.xlsx"
FILE_OUT = "Transport_summary.xlsx"
X_DOCKS = ["C_TL","C_ZWD","NWG","WW","X_UTR","XASD","XZBM"]

def get_decimal_time(time):
    t = time.hour + time.minute/60 + time.second/3600.
    if t<12.: t += 24
    return t

df1 = pd.read_excel(DIR+FILE, skiprows=1, sheet_name="Inter1-ritten")
df2 = pd.read_excel(DIR+FILE, skiprows=0, sheet_name="Inter2-ritten")
dep_dict1 = dict([(int(num), abb) for (num,abb) in zip(df1["Vertrek Depot nr"], df1["Vertrek Depot naam"])])
dep_dict2 = dict([(int(num), abb) for (num,abb) in zip(df2["Depot nr"], df2["Depot naam"])])

dep_dict = {**dep_dict1, **dep_dict2}
col_dict = dict()
for col in df1.columns:
    col = str(col)
    dest = col.split('-')[0]
    try:
        dest = dep_dict.get(int(dest), "unkown")
    except:
        dest = "unknown"
    shift = 1 if "NMG" in col or "Vroeg" in col else 2 if "Laat" in col or "Int" in col or "Ret" in col else -1

    if shift>0 and dest!="unkown":
        col_dict[col] = (dest, shift)
    else:
        print(col, "\t:", dest, shift)

df  = pd.DataFrame(columns=["x_dock", "time", "IO", "nrc", "load", "destination", "shift"])
for i,row in df1.iterrows():
    x_dock = row["Naar CD naam"]
    time   = get_decimal_time(row["Aankomst tijd"])

    load   = []
    for col in col_dict:
        if pd.isna(row[col]): continue
        load += [col_dict[col]]*int(row[col])
    load_s = ";".join(str(rol) for rol in load)
    df.loc[len(df)] = [x_dock, time, "in_bound", len(load), load_s, "", -1]

for i,row in df2.iterrows():
    x_dock = row["Vertrek CD naam"]
    time   = get_decimal_time(row["Vertrek tijd"])
    shift  = 1 if row["Type stroom"]=='V' else 2 if row["Type stroom"]=='L' else -1
    if shift<0: continue

    df.loc[len(df)] = [x_dock, time, "out_bound", 0, [], row["Depot naam"], shift]

df = df.sort_values(["x_dock","IO","time"])

with pd.ExcelWriter(DIR + FILE_OUT) as writer:
    df.to_excel(writer, sheet_name="scheme", index=False, header=True)


for xd,df_dep in df[df.IO=="out_bound"].groupby("x_dock"):
    print(xd, sorted(df_dep["destination"].unique()))