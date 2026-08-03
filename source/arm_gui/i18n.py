# -*- coding: utf-8 -*-
"""极简 i18n：源文案 = 英文，中文经查表翻译。

用法：
    from ..i18n import tr, register       # ui / core 模块内
    tr('Connect')                         # 返回当前语言文本
    register({'Connect': '连接', ...})    # 模块底部注册本模块词条（EN -> ZH）

约定：
* tr() 的参数就是英文原文；中文模式查不到词条时原样返回英文（安全回退，永不抛错）。
* 含 % 占位符的词条（如 'Connected %s @ %d'），中文翻译必须保留全部占位符，
  调用方写法：tr('Connected %s @ %d') % (port, baud)。
* 本模块无 Qt 依赖，core 后端（工作线程）也可安全使用。
* 语言在应用启动时由 main.py 按 AppConfig.ui.language 设定；运行中切换语言
  走「保存配置 + 重启应用」路径（见 MainWindow 语言菜单）。
"""
from __future__ import annotations

from typing import Dict

LANGUAGES = {'en': 'English', 'zh': '中文'}

_zh: Dict[str, str] = {}
_lang = 'en'


def set_language(lang: str) -> None:
    """设置当前语言（'en' | 'zh'；非法值回退 'en'）。"""
    global _lang
    _lang = lang if lang in LANGUAGES else 'en'


def get_language() -> str:
    return _lang


def register(mapping: Dict[str, str]) -> None:
    """注册词条（英文原文 -> 中文）。各模块在文件底部注册自己的词表。"""
    _zh.update(mapping)


def tr(text: str) -> str:
    """翻译：英文原文 -> 当前语言文本。中文模式缺词时原样返回英文。"""
    if _lang == 'zh':
        return _zh.get(text, text)
    return text
