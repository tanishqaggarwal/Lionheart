# 2024-12-13

Notes on getting Bazel to work:
- Had to install Python 3.7 on my Mac using pyenv (`brew install pyenv`)
- Must link Python3.7 into the directory on my system expected by PlatformIO
- Must do `~/.pyenv/versions/3.7.17/bin/python -m pip install platformio` for Bazel compilation to work!