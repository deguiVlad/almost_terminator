# -*- coding: utf-8 -*-
"""Bootstrap для запуска нод arm_ik из bin/ или install/lib/arm_ik/.

Решает все проблемы запуска, которые встречались по ходу отладки:

1) ImportError: circular import — обёртка называлась тем же именем, что и
   модуль пакета. Лечится тем, что обёртки называются run_*.py, а импорт идёт
   из arm_ik.nodes.*.
2) ModuleNotFoundError: No module named 'rclpy' — запуск системным python без
   ROS-окружения. Лечится авто-перезапуском (os.execv) под интерпретатором,
   в котором есть rclpy.
3) ModuleNotFoundError: No module named 'numpy'/'scipy' — с v0.2.3 больше НЕ
   ошибка: пакет автоматически переключается на чистый Python-fallback
   (arm_ik/_math_compat.py). Проверка зависимостей ограничена rclpy (+
   bodyctrl_msgs, если не отключено ARM_IK_REQUIRE_BODYCTRL=0).
4) Затенение пакета при запуске из libexec: каталог .../install/arm_ik/lib/arm_ik
   попадает в sys.path[0] и его файлы перекрывают установленный python-пакет
   `arm_ik`. Лечится вычисткой таких путей перед импортом.
"""
import os
import subprocess
import sys


def _candidates():
    """Интерпретаторы, в которых стоит искать связку rclpy+... ."""
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
    code = ("import importlib,sys;" + ";".join(
        f"importlib.import_module('{m}')" for m in mods) + ";sys.exit(0)")
    try:
        return subprocess.run([py, "-c", code], capture_output=True).returncode == 0
    except OSError:
        return False


def _pick_python(check):
    """Возвращает (python, missing). missing==[] если всё найдено."""
    best_missing = None
    for py in _candidates():
        missing = [m for m in check if not _has([m], py)]
        if not missing:
            return py, []
        if best_missing is None or len(missing) < len(best_missing[1]):
            best_missing = (py, missing)
    return best_missing


def _strip_shadowing_paths(here):
    """Убрать из sys.path libexec-каталоги пакетов arm_ik* и сам here.

    Если их оставить, `import arm_ik` может найти вместо настоящего пакета
    одноимённый каталог lib/arm_ik (или bootstrap затеняет модули), что и
    давало циклические импорты/ModuleNotFoundError на собранной машине.
    """
    def bad(p):
        ap = os.path.abspath(p) if p else ""
        if ap == here:
            return True
        base = os.path.basename(ap)
        parent = os.path.basename(os.path.dirname(ap))
        return base.startswith("arm_ik") and parent == "lib"
    sys.path[:] = [p for p in sys.path if not bad(p)]


def bootstrap(module_name):
    """Возвращает callable main() пакета arm_ik.nodes.<module_name>.

    module_name: 'arm_ik_node'
    """
    here = os.path.dirname(os.path.abspath(__file__))
    _strip_shadowing_paths(here)
    pkg_root = _find_pkg_root()
    if pkg_root and pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    check = ["rclpy"]
    # bodyctrl_msgs по умолчанию не проверяем: без него нода остаётся в dry-run
    # и не публикует команды приводам.
    if os.environ.get("ARM_IK_REQUIRE_BODYCTRL", "0") == "1":
        check.append("bodyctrl_msgs")
    missing = [m for m in check if _try_import(m) is None]
    if missing:
        # пробуем перезапуститься под python'ом, где всё есть
        if os.environ.get("ARM_IK_REEXEC") != "1":
            picked = _pick_python(check)
            if picked is not None:
                py, still_missing = picked
                if py and py != sys.executable and not still_missing:
                    os.environ["ARM_IK_REEXEC"] = "1"
                    try:
                        os.execv(py, [py] + sys.argv)
                    except OSError:
                        pass
        _die_install(missing)

    import importlib
    try:
        mod = importlib.import_module(f"arm_ik.nodes.{module_name}")
    except ImportError as e:
        print(
            "arm_ik: не могу импортировать arm_ik.nodes.%s: %s\n"
            "  sys.path[0..3] = %s\n"
            "  артефакт запущен из: %s\n"
            "  Вероятно, запущена СТАРАЯ сборка пакета (или пакет вообще не\n"
            "  собран/не source'нут). Лечится так:\n"
            "    rm -rf <ws>/build/arm_ik <ws>/install/arm_ik\n"
            "    (удалите также старые копии в других workspace, напр.\n"
            "     rm -rf /nssr_sim/install/arm_ik /nssr_sim/build/arm_ik)\n"
            "    cd <ваш_ws> && colcon build --packages-select arm_ik\n"
            "    source install/setup.bash   # последним!\n"
            "  либо запускайте напрямую из исходников: ./bin/run_arm_ik.py\n"
            % (module_name, e, sys.path[:4], os.path.abspath(__file__)),
            file=sys.stderr)
        raise
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
    bases = [os.path.dirname(here)]                      # bin/../ или lib/arm_ik/../../.. ?
    b = os.path.dirname(os.path.dirname(here))           # lib/<pkg>/../ -> install prefix
    bases.append(b)
    bases.append(os.path.dirname(b))                     # ещё уровнем выше
    bases.append(os.getcwd())
    for base in bases:
        if os.path.isdir(os.path.join(base, "arm_ik")) and os.path.isfile(
                os.path.join(base, "arm_ik", "__init__.py")):
            return base
    return None


def _die_install(missing):
    print(
        "arm_ik: в интерпретаторе {py} не хватает модулей: {miss}.\n"
        "Скорее всего ROS-окружение не source'нуто. Выполните:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source <ваш_ws>/install/setup.bash   # последним\n"
        "и повторите запуск; либо укажите интерпретатор явно:\n"
        "  ARM_IK_PYTHON=/usr/bin/python3 ./bin/run_arm_ik.py\n"
        "(numpy/scipy ставить НЕ нужно — включается pure-python режим;\n"
        " bodyctrl_msgs опционален по умолчанию; ARM_IK_REQUIRE_BODYCTRL=1 делает его обязательным)".format(
            py=sys.executable, miss=", ".join(missing)),
        file=sys.stderr,
    )
    sys.exit(1)
