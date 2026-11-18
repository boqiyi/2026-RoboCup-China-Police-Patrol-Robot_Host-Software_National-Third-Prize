"""指定20步路线实车入口：无障碍识别，保留居中、路口、RFID和转弯云台。"""
from __future__ import annotations

import argparse

import competition_runtime
import route_test_settings
import runlog


def main_cli(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(argv)
    try:
        settings = route_test_settings.load_settings()
        field_map = competition_runtime.coverage.load_map()
        steps = competition_runtime.build_fixed_route(field_map, settings.actions)
    except (ValueError, competition_runtime.coverage.CoverageError) as exc:
        parser.error(str(exc))
    print(f"路线测试：{len(settings.actions)}个动作、{len(steps)}段；相机采集和模型已禁用；"
          "每次转弯前左右2500、转后1500，上下固定700。")
    with runlog.RunLog("route_sequence_test"):
        return competition_runtime.run_competition(profile="route-test", stage3_enabled=False,
            settings=settings, vision_enabled=False)


if __name__ == "__main__":
    raise SystemExit(main_cli())
