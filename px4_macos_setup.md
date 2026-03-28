# PX4 macOS Setup

## 1. Clone the Repository

```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive --force
```

## 2. Set Up Python Environment

```bash
uv python install 3.11
uv venv --python 3.11 .venv
source .venv/bin/activate
```

## 3. Install Dependencies via Homebrew

```bash
brew tap osx-cross/arm
brew tap PX4/px4

# Install (or reinstall) PX4 dev tools
brew install px4-dev
# brew reinstall px4-dev

brew link --overwrite --force arm-gcc-bin@13

# Install (or reinstall) PX4 sim tools
brew install px4-sim
# brew reinstall px4-sim
```

## 4. Install Python Packages

```bash
uv pip install future
uv pip install -r Tools/setup/requirements.txt
```

## 5. Verify Installation

```bash
# NuttX cross-compiler (from arm-gcc-bin@13)
arm-none-eabi-gcc --version

# Build tools
cmake --version
ninja --version

# Gazebo (if --sim-tools was used)
gz sim --versions
```

## 6. Configure Environment Variables (only need to do this everytime you fresh build)

Add to your shell profile, or export before building:

```bash
export CMAKE_PREFIX_PATH="/opt/homebrew/opt/qt@5:$CMAKE_PREFIX_PATH"
export CXXFLAGS="-Wno-error=double-promotion"
```

If you are new to Homebrew, you may need to tell macOS where to find the libraries:

```bash
echo 'export LIBRARY_PATH="/opt/homebrew/lib:$LIBRARY_PATH"' >> ~/.zshrc
echo 'export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:$PKG_CONFIG_PATH"' >> ~/.zshrc
source ~/.zshrc
```

## 7. Run

**Gazebo Backend:**

```bash
uv run make px4_sitl gz_x500
```

**Gazebo Frontend:**

```bash
export GZ_IP=127.0.0.1
gz sim -g
```
