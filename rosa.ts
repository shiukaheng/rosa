// Main entry point for the CLI

import { Command } from "https://deno.land/x/cliffy@v0.25.6/command/mod.ts";
import { build_package, build_packages, createPackageWatcherBuilder, createWatcherBuilder, initPackage, initWorkspace } from "./macros.ts"
import { find_package_from_cd, find_workspace_from_cd } from "./path_finding.ts";
import { bashPreprocessPath, InteractiveShell, runCommands } from "./shell.ts";
import {bold, brightWhite, brightMagenta, brightCyan, brightRed} from "https://deno.land/std@0.167.0/fmt/colors.ts";
import { getConfig } from "./config.ts";
import { resolve, join, sep, parse } from "https://deno.land/std/path/mod.ts";

// Get workspace and package directories
const ws_dir = await find_workspace_from_cd();
const pkg_info = await find_package_from_cd();

const program = new Command();

async function main() {

    // CLI main entry point
    program
        .name("rosa")
        .version("0.0.1")
        .description(bold(brightWhite(`🌹 ROS2 Automation Macros`)))
        .arguments("[command:string]")
        .action(() => {
            program.showHelp();
        })
        .help({
            colors: false
        })

    // From this point on, require a workspace to be present
    program
        .command("workspace-shell")
        .alias("wsh")
        .description("Opens a shell with ROS and workspace sourced (for interacting with packages)")
        .action(async () => {
            const ws_dir = requireWorkspace();
            const config = await getConfig();
            const ws_name = ws_dir.split("/").slice(-1)[0];
            const ros2_distro = parse(config.ros2Path).base
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
            const _ws_dir = requireWorkspace();
            const config = await getConfig();
            const ros2_distro = config.ros2Path.split("/").slice(-2)[0];
            const ps1 = `${brightCyan(ros2_distro)}$ `;
            const shell = new InteractiveShell({ps1, initCommands: [
                `source ${bashPreprocessPath(config.ros2Path)}/setup.bash`,
            ]});
            await shell.status;
        })

    program
        .command("build-all")
        .alias("ba")
        .description("Builds all packages in the workspace")
        .action(async () => {
            const ws_dir = await requireWorkspace();
            const _config = await getConfig();
            await build_packages(ws_dir);
        })

    program
        .command("watch-all")
        .alias("wa")
        .description("Watches all packages in the workspace")
        .action(async () => {
            const _ws_dir = await requireWorkspace();
            const _config = await getConfig();
            const _watcher = createWatcherBuilder();
        })

    // From this point on, require a package to be present
        
    program
        .command("build")
        .alias("b")
        .description("Builds the current package")
        .option("--symlink-install", "Use symlink install instead of copying")
        .option("--continue-on-error", "Continue building even if a package fails")
        .action(async (options) => {
            const ws_dir = await requireWorkspace();
            const pkg_info = await requirePackage();
            console.log(`Building ${pkg_info.toStringColor()}...`);
            await build_package(ws_dir, pkg_info.name, [
                ...(options.symlinkInstall ? ["--symlink-install"] : []),
                ...(options.continueOnError ? ["--continue-on-error"] : []),
            ]);
        })

    program
        .command("watch")
        .alias("w")
        .description("Watches the current package")
        .option("--symlink-install", "Use symlink install instead of copying")
        .option("--continue-on-error", "Continue building even if a package fails")
        .action(async (options) => {
            const _ws_dir = await requireWorkspace();
            const pkg_info = await requirePackage();
            console.log(`Watching ${pkg_info.toStringColor()}...`);
            const _watcher = createPackageWatcherBuilder(pkg_info.name, [
                ...(options.symlinkInstall ? ["--symlink-install"] : []),
                ...(options.continueOnError ? ["--continue-on-error"] : []),
            ]);
        })
    
    // program
    //     .command("config")
    //     .description("Configures rosa for the current workspace")
    //     .action(async () => {
    //         const ws_dir = await requireWorkspace();
    //         const config = await getConfig();
    //     })

    program
        .command("init-ws")
        .description("Initializes a new workspace from the current directory")
        .action(async () => {
            await initWorkspace(".")
        })

    program
        .command("init-pkg")
        .description("Runs an interactive wizard to initialize a new package in the current workspace")
        .action(async () => {
            const ws_dir = await requireWorkspace();
            const config = await getConfig();
            await initPackage(ws_dir, config);
        })

    program.parse(Deno.args);
}

await main();

// Convenience functions
export function requirePackage() {
    if (!pkg_info) {
        console.log(brightRed(`❌ No package found, please run this command from within a package directory`));
        Deno.exit(1);
    }
    return pkg_info;
}

export function requireWorkspace() {
    if (!ws_dir) {
        console.log(brightRed(`❌ Not currently in a workspace, please run this command from within a workspace directory`));
        Deno.exit(1);
    }
    return ws_dir;
}