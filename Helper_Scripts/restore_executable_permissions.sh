#!/usr/bin/env bash

# Restore execute permissions after copying this repository through a medium
# that does not preserve Unix file modes (for example, some ZIP or Windows
# transfers). Run with: bash Helper_Scripts/restore_executable_permissions.sh

set -euo pipefail

repository_root="$(git rev-parse --show-toplevel 2>/dev/null)" || {
    echo "Error: run this script from inside the AD-EYE Git repository." >&2
    exit 1
}

restored_count=0

# Git records the intended executable bit for all tracked files.
while IFS=$'\t' read -r metadata path; do
    if [[ "$metadata" == 100755* && -f "$repository_root/$path" ]]; then
        chmod +x "$repository_root/$path"
        restored_count=$((restored_count + 1))
    fi
done < <(git -C "$repository_root" ls-files -s)

# These fault-injection files may be new or uncommitted when this script is
# used, so they are restored independently of the Git index.
fault_injection_dir="$repository_root/AD-EYE/ROS_Packages/src/AD-EYE/src/fault_injections"
if [[ -d "$fault_injection_dir" ]]; then
    while IFS= read -r -d '' script; do
        chmod +x "$script"
        restored_count=$((restored_count + 1))
    done < <(
        find "$fault_injection_dir" -maxdepth 1 -type f \
            \( -name '*.py' -o -name '*.sh' \) -print0
    )
fi

echo "Restored executable permissions on $restored_count file(s)."
echo "Rebuild and source the workspace before launching ROS nodes."
