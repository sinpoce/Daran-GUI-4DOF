# -*- coding: utf-8 -*-
"""运行位置解析：同时支持源码运行和 PyInstaller 打包后运行。

打包后必须区分两类目录，否则会出严重问题：

* :func:`app_dir` —— **可写、用户可见**的目录。
  源码运行时是项目根目录；打包后是 **exe 所在目录**（不是解压临时目录）。
  config.json、标定输出都必须放这里，否则用户改的配置会被写进
  PyInstaller 的临时解压目录 ``_MEIPASS``，程序一退出就连同临时目录一起消失。

* :func:`res_dir` —— **只读**的随包资源目录。
  源码运行时同样是项目根；打包后是 ``sys._MEIPASS``（解压出来的只读资源）。
  靶标模型 JSON、标定板 PDF 这类跟着程序走、用户不需要改的文件放这里。
"""
from __future__ import annotations

import os
import sys


def is_frozen() -> bool:
    """是否运行在 PyInstaller 打包环境中。"""
    return bool(getattr(sys, 'frozen', False))


def app_dir() -> str:
    """可写目录：打包后 = exe 所在目录；源码运行 = 项目根目录。"""
    if is_frozen():
        return os.path.dirname(os.path.abspath(sys.executable))
    # arm_gui/core/apppaths.py -> arm_gui/core -> arm_gui -> 项目根
    return os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))


def res_dir() -> str:
    """只读随包资源目录：打包后 = _MEIPASS；源码运行 = 项目根目录。"""
    if is_frozen():
        return getattr(sys, '_MEIPASS', app_dir())
    return app_dir()


def user_path(*parts: str) -> str:
    """拼接 :func:`app_dir` 下的可写路径。"""
    return os.path.join(app_dir(), *parts)


def res_path(*parts: str) -> str:
    """拼接 :func:`res_dir` 下的只读资源路径。

    打包后若资源没被收进包里，回退到 exe 同级目录再找一次，
    便于用户把缺失的资源直接放到 exe 旁边补上。
    """
    p = os.path.join(res_dir(), *parts)
    if not os.path.exists(p):
        alt = os.path.join(app_dir(), *parts)
        if os.path.exists(alt):
            return alt
    return p
