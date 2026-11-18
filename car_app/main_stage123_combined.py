"""阶段1+2+3联合比赛入口（默认出发首转前进10 mm，正常TURN）。"""
from competition_runtime import competition_entrypoint

if __name__ == "__main__":
    raise SystemExit(competition_entrypoint("combined", True))
