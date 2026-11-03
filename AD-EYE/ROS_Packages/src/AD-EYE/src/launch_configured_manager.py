#!/usr/bin/env python
"""Read saved startup settings only when the user runs this launcher."""
import argparse
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from startup_configuration import ARGUMENTS, DEFAULTS, default_path, read_file, validate


def launch_arguments(path, ignore_saved=False):
    saved = None if ignore_saved else read_file(path)[0]
    settings = dict(DEFAULTS if saved is None else saved)
    validate(settings)
    return ["roslaunch", "adeye", "manager_real_world_configured.launch"] + [
        "%s:=%s" % (ARGUMENTS[key], "true" if settings[key] else "false") for key in sorted(settings)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ignore-saved", action="store_true", help="Use original configured-launch defaults without deleting anything")
    parser.add_argument("--print-only", action="store_true", help="Show the launch command without starting it")
    args = parser.parse_args()
    try: command = launch_arguments(default_path(), args.ignore_saved)
    except (ValueError, OSError, IOError) as error:
        parser.exit(2, "Startup configuration rejected: %s\nUse --ignore-saved to start without it.\n" % error)
    print(" ".join(command))
    if not args.print_only: os.execvp(command[0], command)


if __name__ == "__main__": main()
