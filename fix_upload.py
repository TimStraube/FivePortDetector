Import("env")

flags = env.get("UPLOADERFLAGS", [])
env.Replace(UPLOADERFLAGS=[
    f.replace("verify reset;", "verify;") if isinstance(f, str) else f
    for f in flags
])
