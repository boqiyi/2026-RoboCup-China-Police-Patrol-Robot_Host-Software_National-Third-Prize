"""线索映射：模型类别 / RFID 卡 → 播报文本 + 音频文件。纯逻辑，可桌面单测。

- 模型 20 类：1~10 = 嫌疑人，11~20 = 刀具（刀具号 = 类别号 − 10）。
- RFID 卡0扇区1块第0字节的数值01~0C = 巡逻点号1~12；UID仅作身份。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import config


class CluesError(RuntimeError):
    """线索映射加载失败。"""


@dataclass(frozen=True)
class DetectionClue:
    category: str       # "SUSPECT" / "KNIFE"
    number: int         # 1..10
    text: str           # 播报文本，如 "发现3号嫌疑人"
    audio: str          # 音频文件名，如 clue_suspect_03.wav


@dataclass(frozen=True)
class PatrolClue:
    number: int         # 1..12
    text: str           # "到达05号巡逻点"
    audio: str          # patrol_05.wav


SUSPECT_TEXT = "发现{}号嫌疑人"
KNIFE_TEXT = "发现物体{}号刀具"
PATROL_TEXT = "到达{:02d}号巡逻点"


class Clues:
    """读取 direct20-map.json 的类别→音频映射，并按用户措辞生成播报文本。"""

    def __init__(
        self,
        clues_json: str | Path | None = None,
        audio_dir: str | Path | None = None,
    ) -> None:
        self.audio_dir = Path(audio_dir) if audio_dir is not None else config.AUDIO_DIR
        self._detection: dict[int, DetectionClue] = {}
        self._patrol: dict[int, PatrolClue] = {}
        self._load(clues_json)

    def _load(self, clues_json: str | Path | None) -> None:
        p = Path(clues_json) if clues_json is not None else config.CLUES_JSON
        try:
            doc = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise CluesError(f"无法读取线索映射 {p}: {exc}") from exc

        by_class: dict[int, dict] = {}
        for entry in doc.get("entries", []):
            cls = int(entry["model_class_name"])
            by_class[cls] = entry
        # 注意：JSON 每条的 text 字段（如「侦查到1号嫌疑人」）不采用——播报文本统一用
        # 下方模板 SUSPECT_TEXT/KNIFE_TEXT（按用户措辞生成），仅 audio 取自 JSON。

        for cls in range(1, 11):  # 嫌疑人 1~10
            entry = by_class.get(cls)
            audio = entry["audio"] if entry else f"clue_suspect_{cls:02d}.wav"
            self._detection[cls] = DetectionClue("SUSPECT", cls, SUSPECT_TEXT.format(cls), audio)
        for cls in range(11, 21):  # 刀具 11~20
            number = cls - 10
            entry = by_class.get(cls)
            audio = entry["audio"] if entry else f"clue_knife_{number:02d}.wav"
            self._detection[cls] = DetectionClue("KNIFE", number, KNIFE_TEXT.format(number), audio)

        for number in range(1, 13):  # 巡逻点 1~12
            self._patrol[number] = PatrolClue(
                number, PATROL_TEXT.format(number), f"patrol_{number:02d}.wav"
            )

    def detection(self, class_id: int) -> DetectionClue | None:
        return self._detection.get(class_id)

    def patrol(self, number: int) -> PatrolClue | None:
        return self._patrol.get(number)

    def patrol_audio_path(self, number: int) -> Path | None:
        clue = self._patrol.get(number)
        return self.audio_dir / clue.audio if clue else None

    def detection_audio_path(self, class_id: int) -> Path | None:
        clue = self._detection.get(class_id)
        return self.audio_dir / clue.audio if clue else None

