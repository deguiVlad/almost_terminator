# -*- coding: utf-8 -*-
"""Единый bootstrap для standalone-запуска пакетa arm_ik.

Делает три вещи, каждое из которых лечит конкретную ошибку из эксплуатации:

1. rclpy есть только в системном python3 (/usr/lib/python3/dist-packages).
   Если нас запустили другим интерпретатором (например /usr/local/bin/python3
   из контейнера/venv) — перезапускаемся под /usr/bin/python3 с теми же argv.
   Лечит: ModuleNotFoundError: No module named 'rclpy'.

2. Гарантирует, что импорт `arm_ik` разрешается в Python-пакет ИСХОДНИКОВ
   этого пакета (<pkg>/arm_ik), а не в битые артефакты старых сборок
   (например /nssr_sim/install/arm_ik/lib/arm_ik/*.py, где скрипт-обёртка
   назывался так же, как модуль, и давал ImportError: circular import).
   Лечит: ImportError: cannot import name 'main' from partially initialized
   module 'arm_ik.arm_ik_node'.

3. Проверяет наличие ROS 2 (rclpy) и numpy/scipy и даёт человекочитаемую
   подсказку вместо трейсбека, если окружение не source'нуто.
"""
import os
import sys

_MIN_PY = (3, 8)


def _has(mod):
    try:
        __import__(mod)
        return True
    except Exception:
        return False


def bootstrap(node_module, main_attr="main"):
    """Возвращает callable main() ноды из arm_ik.nodes.<node_module>.

    node_module: "arm_ik_node" | "demo_goal_publisher" | "demo_delta_publisher"
    """
    # --- 0) понятная ошибка, если python совсем неподходящий -------------------
    if sys.version_info < _MIN_PY:
        sys.stderr.write("arm_ik: нужен python >= 3.8, запущен %s\n" % sys.version)
        sys.exit(1)

    # --- 1) перезапуск под интерпретатором с rclpy -----------------------------
    if not _has("rclpy"):
        for py in ("/usr/bin/python3", "/usr/bin/python3.10", "/usr/bin/python3.8"):
            if os.path.exists(py) and os.path.realpath(py) != os.path.realpath(sys.executable):
                try:
                    os.execv(py, [py] + sys.argv)
                except OSError:
                    pass
        sys.stderr.write(
            "arm_ik: rclpy не найден ни в одном доступном интерпретаторе.\n"
            "         Source'ните ROS 2 перед запуском:\n"
            "             source /opt/ros/humble/setup.bash\n")
        sys.exit(1)

    if not (_has("numpy") and _has("scipy")):
        sys.stderr.write(
            "arm_ik: не хватает numpy/scipy. Установите:\n"
            "             sudo apt install python3-numpy python3-scipy\n"
            "         или pip3 install numpy scipy (для того же python3, "
            "что содержит rclpy).\n")
        sys.exit(1)

    # --- 2) находим корень исходников пакета ----------------------------------
    here = os.path.dirname(os.path.abspath(__file__))          # <pkg>/bin (или lib/<pkg>)
    pkg_root = os.path.dirname(here)                            # <pkg>
    mod_ok = os.path.join(pkg_root, "arm_ik", "nodes", node_module + ".py")

    if not os.path.exists(mod_ok):
        # Мы НЕ внутри целого пакета (например, исполняемые скопированы setup.py
        # в install/arm_ik/lib/arm_ik/). Ищем пакет-исходники по кандидатам.
        cand_roots = []
        try:
            from ament_index_python.packages import get_package_prefix
            cand_roots.append(get_package_prefix("arm_ik"))     # install/arm_ik
        except Exception:
            pass
        ws_guesses = [
            os.path.abspath(os.path.join(here, "..", "..", "src", "arm_ik")),   # ws/src/arm_ik
            os.path.abspath(os.path.join(here, "..", "..", "..", "src", "arm_ik")),
            "/home/user/nssr-polygon/hackathon/ros2/arm_ik",
            "/home/user/nssr-polygon/hackathon/arm_ik",
            "/workspace/arm_ik",
        ]
        cand_roots += ws_guesses
        chosen = None
        for c in cand_roots:
            if c and os.path.exists(os.path.join(c, "arm_ik", "nodes", node_module + ".py")):
                chosen = c
                break
        if chosen is None:
            sys.stderr.write(
                "arm_ik: не найден python-пакет arm_ik/nodes/%s.py.\n"
                "         Запустите из исходников пакета (bin/ рядом с arm_ik/) "
                "или пересоберите: rm -rf build/arm_ik install/arm_ik && "
                "colcon build --packages-select arm_ik\n" % node_module)
            sys.exit(1)
        pkg_root = chosen

    # --- 3) чистим sys.path от тенирующих каталогов и вставляем наш -------------
    bad_tail = os.path.join(pkg_root, "arm_ik")       # если CWD==pkg_root/arm_ik
    cleaned = []
    for p in sys.path:
        rp = os.path.realpath(p or os.getcwd())
        # удаляем пути, где arm_ik резолвится НЕ в наш python-пакет:
        #  - libexec старого install (там файлы-обёртки arm_ik_node.py вместо пакета)
        #  - сам каталог пакета (arm_ik/), из-за которого 'import arm_ik' падает в пустоту
        if rp == os.path.realpath(bad_tail):
            continue
        if os.path.basename(rp.rstrip("/")) in ("arm_ik", "bin") and \
                os.path.exists(os.path.join(rp, node_module + ".py")) and \
                not os.path.isdir(os.path.join(rp, "arm_ik")):
            continue  # wrapper-директория из битой сборки
        cleaned.append(p)
    sys.path[:] = cleaned
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    # sanity: 'import arm_ik' должен указывать на наш пакет
    try:
        import arm_ik as _ai
        got = os.path.realpath(os.path.dirname(_ai.__file__))
        want = os.path.realpath(os.path.join(pkg_root, "arm_ik"))
        if got != want:
            sys.stderr.write("arm_ik: WARNING 'import arm_ik' указывает на %s (ожидался %s)\n"
                             % (got, want))
    except Exception as e:
        sys.stderr.write("arm_ik: импорт пакета не удался: %s\n" % e)
        raise

    from importlib import import_module
    mod = import_module("arm_ik.nodes." + node_module)
    return getattr(mod, main_attr)
