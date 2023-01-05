# 🌹 `rosa` — ROS2 Automation Macros
> ⚠️ WARNING: Still under active development

`rosa` is a thin wrapper on ROS2 tools to streamline common ROS2 development tasks. It automatically sources the correct files, and provides a file watcher to automatiaclly build packages upon file changes.
## 📦 Installation
REQUIREMENT: Make sure you have `deno` installed on your system ([instructions](https://deno.land/manual/getting_started/installation))
```bash
deno install --allow-sys --allow-env --allow-run --allow-read --allow-write --unstable -f https://deno.land/x/rosa@v0.0.1e/rosa.ts
# Add deno bin to .bashrc if it doesn't exist (tested on ubuntu)
LINE='export PATH="/home/$USER/.deno/bin:$PATH"'
FILE="${HOME}/.bashrc"
grep -qF -- "$LINE" "$FILE" || echo "$LINE" >> "$FILE"
```
## 👨‍💻 Sample usage
> ℹ️ NOTE: Make sure your current working directory is inside of a ROS2 workspace (could be any sub-directory of the workspace). During first use of any `rosa` commands in a new workspace, it prompts for user configuration (mainly choosing which ROS2 installation to use).

### Creating a new package
<details>
  <summary>Click to show original steps...</summary>
  
  ```bash
  # Open a new terminal
  source /opt/ros/<distribution>/setup.sh
  cd ../../ # (cd'ing to workspace root)
  ros2 pkg create --build-type ament_cmake <package name>
  ```
</details>
  
  ```bash
  # Run this from any existing terminal inside the workspace
  rosa init-pkg
  # ? Enter the name of the package › new_package
  # ? Enter the description of the package › A sample package
  # ? Select the build type › ament_python
  # ? Create empty node? (y/n) › Yes
  # ? Enter the name of the node › new_node
  # ? Select a license? (y/n) › Yes
  # ? Select a license › MIT
  # ...
  ```

### Building the current package
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
rosa build # Automatically builds the package folder you are in
```
Or, if you want to automatically build the package upon file changes:
```bash
rosa watch
```

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
rosa wsh # wsh = workspace shell
ros2
```

For more information, run `rosa --help` or `rosa <command> --help`

## Todo
### General
- [ ] Compatibility with more shell environments (currently only bash is supported)

### Watcher
- [ ] Allow per package configuration of build options (e.g., always use --symlink-install)
- [ ] Make watcher dependency aware and automatically dependent packages