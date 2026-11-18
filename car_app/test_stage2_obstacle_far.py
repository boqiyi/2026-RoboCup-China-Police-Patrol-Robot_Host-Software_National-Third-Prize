"""阶段二远距离颜色面积标定入口，保留浏览器预览。"""
import sys
from test_stage2_obstacle import _main


if __name__ == "__main__":
    sys.exit(_main(distance_profile="far"))
