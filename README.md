# 🌹 `rosa` — ROS2 Automation Macros
`rosa` is a thin wrapper on ROS2 tools to streamline common ROS2 development tasks. It automatically sources the correct files, and provides a file watcher to automatiaclly build packages upon file changes.
## 📦 Installation
- Make sure you have `deno` installed on your system
```bash
deno install --allow-sys --allow-env --allow-run --allow-read --allow-write --unstable -f https://deno.land/x/rosa@v0.0.1c/rosa.ts
# You may need to add the binary directory to your PATH variable to make it runnable everywhere, or add it to your .bashrc file
export PATH="/home/$USER/.deno/bin:$PATH"
```
## 👨‍💻 Sample usage
> NOTE: Make sure your current working directory is inside of a ROS2 workspace (could be any sub-directory of the workspace). During first use of any `rosa` commands in a new workspace, it prompts for user configuration (mainly choosing which ROS2 installation to use).

### Setting up terminal environment to use `ros2` CLI
<details>
  <summary>Click to show original steps...</summary>
  
  ```bash
  source /opt/ros/<distribution>/setup.sh
  source ../../install/setup.sh
  ros2
  ```
</details>

```bash
rosa wsh # or alternatively, rosa workspace-shell
ros2
```

### Building a package
<details>
  <summary>Click to show original steps...</summary>
  
  ```bash
  # Open a new terminal
  source /opt/ros/<distribution>/setup.sh
  cd ../../ # (cd'ing to workspace root)
  colcon build --packages-select <package name>
  ```
</details>

```bash
# Run this from any existing terminal inside the package
rosa build # Automatically builds the package folder you are in
```

### Watching and automatically building packages in workspace (experimental)
```bash
# Run this as long as you are inside a workspace
rosa watch-all # Maps all packages within workspace, and if there are any files changes, the package is rebuilt.
```
## Todo
### General
- [ ] Compatibility with more shell environments (currently only bash is supported)
### Watcher
- [ ] Allow per package configuration of build options (e.g., always use --symlink-install)
- [ ] Make watcher dependency aware and automatically dependent packages
