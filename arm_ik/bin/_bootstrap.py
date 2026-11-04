# -*- coding: utf-8 -*-
"""Bootstrap для запуска нод arm_ik из bin/ или install/lib/arm_ik/.

Решает три проблемы, которые по очереди ломали запуск:
1) ImportError: circular import — обёртка называлась тем же именем, что и
   модуль пакета (arm_ik_node.py). Лечится тем, что обёртки называются
   run_*.py, а импорт идёт из arm_ik.nodes.*.
2) ModuleNotFoundError: No module named 'rclpy' — скрипт запускался системным
   python3 без ROS-окружения. Лечится авто-перезапуском под интерпретатором,
   в котором есть rclpy (см. _find_python_with_rclpy).
3) ModuleNotFoundError: No module named 'numpy'/'scipy' — тот же rclpy-питон
   без научных пакетов (например, venv поверх system-site-packages). Лечится
   поиском подходящего python'а среди нескольких кандидатов и понятной
   инструкцией по установке в ТОТ интерпретатор, который реально используется.
"""
import os
import subprocess
import sys


def _candidates():
    """Интерпретаторы, в которых стоит искать связку rclpy+numpy+scipy."""
    out = []
    cur = sys.executable
    if cur:
        out.append(cur)
    env_py = os.environ.get("ARM_IK_PYTHON")
    if env_py:
        out.append(env_py)
    for p in ("/usr/bin/python3", "/usr/local/bin/python3"):
        if p not in out and os.path.exists(p):
            out.append(p)
    return out


def _has(mods, py):
    code = "import importlib,sys;" + ";".join(
        f"importlib.import_module('{m}')" for m in mods
    ) + ";sys.exit(0)"
    try:
        return subprocess.run([py, "-c", code], capture_output=True).returncode == 0
    except OSError:
        return False


def _pick_python():
    """Возвращает (python, missing). missing==[] если всё найдено."""
    best_missing = None
    for py in _candidates():
        for need in (("rclpy",), ("rclpy", "numpy"), ("rclpy", "numpy", "scipy")):
            pass
        # считаем, чего не хватает в этом python'е
        missing = [m for m in ("rclpy", "numpy", "scipy") if not _has([m], py)]
        if not missing:
            return py, []
        if best_missing is None or len(missing) < len(best_missing[1]):
            best_missing = (py, missing)
    return best_missing[0], best_missing[1]


def bootstrap(module_name):
    """Возвращает callable main() пакета arm_ik.nodes.<module_name>.

    module_name: 'arm_ik_node' | 'demo_goal_publisher' | 'demo_delta_publisher'
    """
    pkg_root = _find_pkg_root()
    if pkg_root and pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    missing = [m for m in ("rclpy", "numpy", "scipy") if _try_import(m) is None]
    if missing:
        # пробуем перезапуститься под python'ом, где всё есть
        if os.environ.get("ARM_IK_REEXEC") != "1":
            py, still_missing = _pick_python()
            if py and py != sys.executable and not still_missing:
                os.environ["ARM_IK_REEXEC"] = "1"
                os.execv(py, [py] + sys.argv)
        _die_install(missing)

    import importlib
    mod = importlib.import_module(f"arm_ik.nodes.{module_name}")
    return getattr(mod, "main")


def _try_import(name):
    import importlib
    try:
        return importlib.import_module(name)
    except Exception:
        return None


def _find_pkg_root():
    """Каталог, содержащий пакет arm_ik (рядом с bin/ или lib/arm_ik/...)."""
    here = os.path.dirname(os.path.abspath(__file__))
    for base in (os.path.dirname(here), here, os.getcwd()):
        if os.path.isdir(os.path.join(base, "arm_ik")) and os.path.isfile(
            os.path.join(base, "arm_ik", "__init__.py")
        ):
            return base
    # install-вариант: .../install/arm_ik/lib/arm_ik/run_x.py -> .../site-packages уже в sys.path
    return None


def _die_install(missing):
    print(
        "arm_ik: в интерпретаторе {py} не хватает модулей: {miss}.\n"
        "Установите ИХ В ЭТОТ python:\n"
        "  sudo apt install python3-numpy python3-scipy\n"
        "  {py} -m pip install numpy scipy\n"
        "(если rclpy тоже отсутствует — source /opt/ros/humble/setup.bash\n"
        " перед запуском; либо укажите нужный интерпретатор: ARM_IK_PYTHON=/usr/bin/python3)".format(
            py=sys.executable, miss=", ".join(missing)
        ),
        file=sys.stderr,
    )
    sys.exit(1)
