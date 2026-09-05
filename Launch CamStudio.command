#!/bin/zsh

# Friendly macOS launcher for people who downloaded the source from GitHub.
# It checks the two native dependencies and rebuilds only changed files.

project_dir="${0:A:h}"
cd "$project_dir" || exit 1

fail() {
  print ""
  print -u2 "CamStudio could not start: $1"
  print ""
  read -r "?Press Return to close this window."
  exit 1
}

install_with_homebrew() {
  local formula="$1"
  local purpose="$2"
  if ! command -v brew >/dev/null 2>&1; then
    fail "$purpose is missing. Install Homebrew from https://brew.sh and then open this launcher again."
  fi
  print "$purpose is required but is not installed."
  local reply
  read -r "reply?Install it now with Homebrew? [Y/n] "
  if [[ "${reply:l}" == "n" || "${reply:l}" == "no" ]]; then
    fail "$purpose is required to run CamStudio."
  fi
  brew install "$formula" || fail "Homebrew could not install $formula."
}

if [[ "$(uname -s)" != "Darwin" ]]; then
  fail "the double-click launcher currently supports macOS only. See README.md for command-line build instructions."
fi

if ! command -v cmake >/dev/null 2>&1; then
  install_with_homebrew cmake "CMake"
fi

python_command="$(command -v python3 2>/dev/null)"
if [[ -z "$python_command" ]]; then
  install_with_homebrew python "Python 3"
  python_command="$(command -v python3 2>/dev/null)"
fi

print "Preparing CamStudio…"
if ! cmake -S . -B build -DCMAKE_BUILD_TYPE=Release; then
  if command -v brew >/dev/null 2>&1 && ! brew list --versions opencascade >/dev/null 2>&1; then
    install_with_homebrew opencascade "Open CASCADE (the STEP geometry engine)"
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release || fail "CMake configuration still failed. Copy the red error text when asking for help."
  else
    fail "CMake configuration failed. Copy the red error text when asking for help."
  fi
fi
cmake --build build --parallel || fail "the CamStudio build failed. Copy the red error text when asking for help."

"$python_command" tools/camstudio_app.py || fail "the local CamStudio window stopped unexpectedly."
