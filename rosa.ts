// rosa - ROS2 macros
import { Command } from "https://deno.land/x/cmd@v1.2.0/commander/index.ts";
import { build_packages, find_package_from_cd, find_workspace, find_workspace_from_cd } from "./macros.ts"
import { bashPreprocessPath, InteractiveShell } from "./shell.ts";
import {bold, brightWhite, brightMagenta, brightCyan} from "https://deno.land/std@0.167.0/fmt/colors.ts";
import { Watcher } from "./watcher.ts";

const config = {
    workspaceSearchDepth: 10,
    packageSearchDepth: 10,
    ros2Path: "/opt/ros/humble/",
}

// Check if ROS2 path is valid
try {
    const info = await Deno.lstat(config.ros2Path);
    if (!info.isDirectory) {
        console.log(`❌ The ROS2 path '${config.ros2Path}' is not a directory`);
        Deno.exit(1);
    }
} catch (e) {
    console.log(`❌ The ROS2 path '${config.ros2Path}' does not exist`);
    Deno.exit(1);
}

export type Config = typeof config;

const program = new Command();

export async function getCurrentPackage() {
    const pkg_info = await find_package_from_cd(config);
    if (!pkg_info) {
        console.log(`❌ No package found (searched up to ${config.packageSearchDepth} levels)`);
        Deno.exit(1);
    }
    return pkg_info;
}

export async function getCurrentWorkspace() {
    const ws_dir = await find_workspace_from_cd(config);
    if (!ws_dir) {
        console.log(`❌ No workspace found (searched up to ${config.workspaceSearchDepth} levels)`);
        Deno.exit(1);
    }
    return ws_dir;
}

async function main() {
    program
    .name("rosa")
    .version("0.0.1")
    .description(bold(brightWhite(`🤖 ROS2 automation macros`)))

    // program
    //     .command("cws")
    //     .description("Finds the ROS2 workspace given the current directory")
    //     .action(async () => {
    //         return await getCurrentWorkspace();
    //     })

    // program
    //     .command("cpkg")
    //     .description("Finds the ROS2 package given the current directory")
    //     .action(async () => {
    //         return await getCurrentPackage();
    //     })

    program
        .command("workspace_shell")
        .alias("wsh")
        .description("Opens a shell with ROS and workspace sourced (for interacting with packages)")
        .action(async () => {
            const ws_dir = await getCurrentWorkspace();
            // Get last directory of workspace and ros2 path
            const ws_name = ws_dir.split("/").slice(-1)[0];
            const ros2_distro = config.ros2Path.split("/").slice(-2)[0];
            // Create PS1
            const ps1 = `${brightCyan( ros2_distro)}:${brightMagenta(ws_name)}$ `;   
            // Launch shell
            const shell = new InteractiveShell({dir: ws_dir, ps1, initCommands: [
                `source ${bashPreprocessPath(config.ros2Path)}/setup.bash`,
                `source ${bashPreprocessPath(ws_dir)}/install/setup.bash`,
            ]});
            await shell.status;
        })

    program
        .command("shell")
        .alias("sh")
        .description("Opens a shell with just ROS sourced (for use with colcon)")
        .action(async () => {
            const ros2_distro = config.ros2Path.split("/").slice(-2)[0];
            const ps1 = `${brightCyan(ros2_distro)}$ `;
            const shell = new InteractiveShell({ps1, initCommands: [
                `source ${bashPreprocessPath(config.ros2Path)}/setup.bash`,
            ]});
            await shell.status;
        })

    program
        .command("build_all")
        .description("Builds all packages in the workspace")
        .action(async (...args: string[]) => {
            console.log(args);
            const ws_dir = await getCurrentWorkspace();
            await build_packages(ws_dir);
        })

    program
        .command("watch_all")
        .description("Watches all packages in the workspace")
        .action(async (...args: string[]) => {
            const ws_dir = await getCurrentWorkspace();
            const watcher = new Watcher(ws_dir);
        })
        

    program.parse(Deno.args);
}

await main();