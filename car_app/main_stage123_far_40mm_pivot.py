"""阶段1+2+3远距比赛入口（出发首转前进40 mm，原地转弯）。"""
from competition_runtime import competition_entrypoint

if __name__ == "__main__":
    raise SystemExit(competition_entrypoint("far", True,
                                          departure_strategy="40mm_pivot"))
