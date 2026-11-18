"""阶段1第二套：首个正常转弯路口追加40mm后双轮±91原地转。"""
import sys

from test_stage1_patrol import main_cli


if __name__ == "__main__":
    sys.exit(main_cli(departure_strategy="40mm_pivot"))
