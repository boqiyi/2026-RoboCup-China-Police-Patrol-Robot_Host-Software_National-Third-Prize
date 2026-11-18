"""阶段1+2联合比赛入口（出发首转前进40 mm，原地转弯）。"""
from competition_runtime import competition_entrypoint

if __name__ == "__main__":
    raise SystemExit(competition_entrypoint("combined", False,
                                          departure_strategy="40mm_pivot"))
