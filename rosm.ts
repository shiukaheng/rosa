// rosm - ROS2 macros
import { Command } from "https://deno.land/x/cmd@v1.2.0/commander/index.ts";
import { find_package_from_cd, find_workspace, find_workspace_from_cd } from "./macros.ts"
import { InteractiveShell } from "./shell.ts";

// Create CLI with two sub-commands
// > rosm build -- builds the current ROS package
// > rosm watch -- watches the current ROS package for changes and rebuilds
// Both support for a --symlink-install option

const config = {
    workspaceSearchDepth: 10,
    packageSearchDepth: 10,
    ps1: `\x1b[35m\x1b[1mROS Workspace\x1b[0m$ `,
}

export type Config = typeof config;

const program = new Command();

async function main() {
    program
    .name("rosm")
    .version("0.0.1")
    .description("ROS2 macros");

    program
        .command("cws")
        .description("Finds the ROS2 workspace given the current directory")
        .action(async () => {
            const ws_dir = await find_workspace_from_cd(config);
            if (ws_dir) {
                console.log(`✅ Found workspace at '${ws_dir}'`);
            } else {
                console.log(`❌ No workspace found (searched up to ${config.workspaceSearchDepth} levels)`);
            }
        })

    program
        .command("cpkg")
        .description("Finds the ROS2 package given the current directory")
        .action(async () => {
            const pkg_info = await find_package_from_cd(config);
            if (pkg_info) {
                console.log(`✅ Found package ${pkg_info}`);
            } else {
                console.log(`❌ No package found (searched up to ${config.packageSearchDepth} levels)`);
            }
        })


    // program
    //     .command("build")
    //     .description("Builds the current ROS package")
    //     // .option("-s, --symlink-install", "Symlink install the package")
    //     .action(async () => {
    //         // Find the workspace
    //         const ws_dir = await find_workspace(Deno.cwd(), config.workspaceSearchDepth);
    //         if (!ws_dir) {
    //             console.log("❌ No workspace found (searched up to ${config.workspaceSearchDepth} levels)");
    //             Deno.exit(1);
    //         }
    //         // Source
    //     })

    // program
    //     .command("watch")
    //     .description("Watches the current ROS package for changes and rebuilds")
    //     // .option("-s, --symlink-install", "Symlink install the package")
    //     .action((options) => {
    //         // Watch the package
    //         console.log("Watching package for changes...")
    //     })

    program
        .command("shell")
        .description("Opens a shell in the current ROS workspace")
        .action(async () => {
            // Open a shell
            const ws_dir = await find_workspace_from_cd(config);
            if (!ws_dir) {
                console.log(`❌ No workspace found (searched up to ${config.workspaceSearchDepth} levels)`);
                Deno.exit(1);
            }
            const shell = new InteractiveShell({dir: ws_dir, ps1: config.ps1});
        })

    program.parse(Deno.args)
}

await main();