"""
Semantic version bump based on Conventional Commits.

Reads commit messages since the latest tag and determines the bump type:
  fix:    → patch  (1.4.2 → 1.4.3)
  feat:   → minor  (1.4.2 → 1.5.0)
  feat!:  → major  (1.4.2 → 2.0.0)
  BREAKING CHANGE: in commit body → major

Usage:
  python scripts/version_bump.py           # dry-run, prints what would happen
  python scripts/version_bump.py --apply   # creates git tag (no file changes)
  python scripts/version_bump.py --check   # print current version only

Note: FIRMWARE_VERSION is NOT stored in any file — it is derived at
build time from git tags by scripts/pre_build_version.py.
So this script only needs to create a tag; no file edits needed.
"""

import subprocess
import re
import sys
import os

TAG_PREFIX = "v"
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(cmd):
    if isinstance(cmd, str):
        cmd = cmd.split()
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO_ROOT)
    return result.stdout.strip()


def get_latest_tag():
    """Return the most recent semver tag, or None."""
    tags = run(["git", "tag", "--list", "v*", "--sort=-version:refname"])
    if not tags:
        return None
    for tag in tags.split("\n"):
        if re.match(r"^v\d+\.\d+", tag.strip()):
            return tag.strip()
    return None


def parse_version(tag):
    """Parse 'v1.4.2' → (1, 4, 2). Handles 'v1.0' → (1, 0, 0)."""
    if tag is None:
        return (0, 0, 0)
    m = re.match(r"v?(\d+)\.(\d+)\.?(\d*)", tag)
    if not m:
        return (0, 0, 0)
    patch = int(m.group(3)) if m.group(3) else 0
    return (int(m.group(1)), int(m.group(2)), patch)


def get_commits_since(tag):
    """Return list of (subject, body) tuples since the given tag."""
    fmt = "--pretty=format:%s|||%b|||END"
    if tag:
        log = run(["git", "log", f"{tag}..HEAD", fmt])
    else:
        log = run(["git", "log", fmt])
    if not log:
        return []

    commits = []
    for entry in log.split("|||END"):
        entry = entry.strip()
        if not entry:
            continue
        parts = entry.split("|||")
        subject = parts[0].strip() if len(parts) > 0 else ""
        body = parts[1].strip() if len(parts) > 1 else ""
        if subject:
            commits.append((subject, body))
    return commits


def determine_bump(commits):
    """
    Return 'major', 'minor', 'patch', or None.
    Stops at 'major' — no need to keep scanning.
    """
    bump = None
    for subject, body in commits:
        # Breaking change
        if "BREAKING CHANGE" in body or "BREAKING CHANGE" in subject:
            return "major"
        if re.match(r"^\w[\w-]*(\(.+\))?!:", subject):
            return "major"
        # Feature → minor
        if re.match(r"^feat(\(.+\))?:", subject):
            bump = "minor"
        # Fix / perf / refactor / build → patch (only if not already minor+)
        elif bump is None and re.match(r"^(fix|perf|refactor|build)(\(.+\))?:", subject):
            bump = "patch"
    return bump


def apply_bump(major, minor, patch, bump_type):
    if bump_type == "major":
        return (major + 1, 0, 0)
    if bump_type == "minor":
        return (major, minor + 1, 0)
    if bump_type == "patch":
        return (major, minor, patch + 1)
    return (major, minor, patch)


def main():
    mode = "dry-run"
    if "--apply" in sys.argv:
        mode = "apply"
    elif "--check" in sys.argv:
        mode = "check"

    latest_tag = get_latest_tag()
    current = parse_version(latest_tag)
    current_str = f"{current[0]}.{current[1]}.{current[2]}"

    if mode == "check":
        print(current_str)
        return

    commits = get_commits_since(latest_tag)
    bump_type = determine_bump(commits)

    print(f"Latest tag:       {latest_tag or '(none)'}")
    print(f"Current version:  {current_str}")
    print(f"Commits since:    {len(commits)}")
    print(f"Bump type:        {bump_type or '(none — no conventional commits found)'}")

    if not bump_type:
        print("\nNo version bump needed.")
        sys.exit(0)

    new = apply_bump(*current, bump_type)
    new_str = f"{new[0]}.{new[1]}.{new[2]}"
    new_tag = f"{TAG_PREFIX}{new_str}"

    print(f"New version:      {new_str}  →  tag {new_tag}")

    if mode == "dry-run":
        print("\nDry run — use --apply to create the tag.")
        sys.exit(0)

    # Create annotated tag (no file changes — version lives in git tags)
    result = subprocess.run(
        ["git", "tag", "-a", new_tag, "-m", f"Release {new_str}"],
        cwd=REPO_ROOT, capture_output=True, text=True
    )
    if result.returncode != 0:
        print(f"ERROR: {result.stderr.strip()}")
        sys.exit(1)

    print(f"\n✅ Tag {new_tag} created.")
    print("   Run: git push --tags")


if __name__ == "__main__":
    main()
