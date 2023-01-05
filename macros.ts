// High level functions meant to be called by the CLI

import { requireWorkspace } from "./rosa.ts";
import { Package } from "./package.ts";
import { Watcher } from "./watcher.ts";
import { Confirm, prompt, Select, Input, Number, Checkbox } from "https://deno.land/x/cliffy@v0.25.6/prompt/mod.ts";
import RosaConfig, { Config, getConfig } from "./config.ts";
import { resolve, join } from "https://deno.land/std/path/mod.ts";
import { runCommands } from "./shell.ts";
import { Licenses } from "./constants.ts";
import { brightMagenta } from "https://deno.land/std@0.167.0/fmt/colors.ts";

/**
 * Builds all packages in the workspace
 * @param cwd The directory to start searching from
 * @param args Additional arguments to pass to colcon
 * @returns The status of the build process
 */
export function build_packages(cwd: string, args: string[]=[]) {
    // To be extended with a package list
    // For now, just build all packages in the workspace
    const process = Deno.run({
        cmd: ["colcon", "build", ...args],
        cwd: cwd
    });
    return process
}

/**
 * Builds a single package in the workspace
 * @param cwd The directory to start searching from
 * @param package_name The name of the package to build
 * @param args Additional arguments to pass to colcon
 * @returns The status of the build process
 */
export function build_package(cwd: string, package_name: string, args: string[]=[]) {
    // To be extended with a package list
    // For now, just build all packages in the workspace
    const process = Deno.run({
        cmd: ["colcon", "build", "--packages-select", package_name, ...args],
        cwd: cwd
    });
    return process
}

/**
 * Creates a watcher for all the packages of the current workspace
 * @returns The watcher instance
 */
export async function createWatcherBuilder(): Promise<Watcher> {
    // TODO: require path
    console.warn(brightMagenta("WARNING: This feature is experimental and may not work as expected."))
    // TODO: Kill the previous process if it is still running
    // Start watcher
    const ws_dir = await requireWorkspace();
    const watcher = new Watcher(ws_dir);
    // let last_builder: null | Deno.Process = null;
    const last_builder_map = new Map<string, Deno.Process>();
    watcher.addEventListener("package_modified_debounced", (e)=>{
        // @ts-ignore - events.detail is not typed
        const pkg = e.detail as Package;
        console.log(`Package modified: ${pkg.toStringColor()}, rebuilding...`);
        if (last_builder_map.has(pkg.name)) {
            try {
                last_builder_map.get(pkg.name)!.kill("SIGINT")
            } catch (e) {
                // Do nothing, this means the process has already exited
            }
        }
        const last_builder = build_package(ws_dir, pkg.name)
        last_builder_map.set(pkg.name, last_builder);
        last_builder.status().then((status)=>{
            if (status.success) {
                console.log(`✅ Build successful for ${pkg.toStringColor()}`);
            } else {
                if (status.code === 2) {
                    // Do nothing, this means it has been aborted by rosa
                } else {
                    console.log(`❌ Build failed for ${pkg.toStringColor()}`);
                }
            }
        });
        
    })
    return watcher;
}

/**
 * Creates a watcher for a single package
 * @param package_name The name of the package to watch
 * @returns The watcher instance
 */
export async function createPackageWatcherBuilder(package_name: string, buildOptions:string[]=[]): Promise<Watcher> {
    // Start watcher
    const ws_dir = await requireWorkspace();
    const watcher = new Watcher(ws_dir);
    let last_builder: null | Deno.Process = null;
    watcher.addEventListener("package_modified_debounced", (e)=>{
        // @ts-ignore - events.detail is not typed
        const pkg = e.detail as Package;
        if (pkg.name === package_name) {
            console.log(`Package modified: ${pkg.toStringColor()}, rebuilding...`);
            if (last_builder) {
                try {
                    last_builder.kill("SIGINT")
                } catch (e) {
                    // Do nothing, this means the process has already exited
                }
            }
            last_builder = build_package(ws_dir, pkg.name, buildOptions)
            last_builder.status().then((status)=>{
                if (status.success) {
                    console.log(`✅ Build successful for ${pkg.toStringColor()}`);
                } else {
                    if (status.code === 2) {
                        // Do nothing, this means it has been aborted by rosa
                    } else {
                        console.log(`❌ Build failed for ${pkg.toStringColor()}`);
                    }
                }
            });
        }
    })
    return watcher;
}

export async function initWorkspace(dir: string): Promise<void> {
    dir = Deno.realPathSync(dir);
    // Check if dir is a directory
    const dir_info = await Deno.lstat(dir);
    if (!dir_info.isDirectory) {
        throw new Error(`Directory ${dir} is not a directory`);
    }
    // Check if the current directory is empty
    const fileIterator = await Deno.readDir(dir);
    const files = [];
    for await (const file of fileIterator) {
        files.push(file);
    }
    if (files.length > 0) {
        // Warn the user that the directory is not empty
        const confirm = await prompt([{
            // @ts-ignore - Confirm is not typed
            type: Confirm,
            name: "confirm",
            message: `Directory ${dir} is not empty, do you want to continue?`,
        }])
        if (!confirm.confirm) {
            Deno.exit(0);
            return;
        }
    }
    // Create the workspace by running colcon build
    const process = Deno.run({
        cmd: ["colcon", "build"],
        cwd: dir,
        stdin: "null",
        stdout: "null",
        stderr: "null"
    });
    const status = await process.status();
    if (!status.success) {
        throw new Error(`Failed to initialize workspace with colcon: ${status.code}`);
    }
    // Make src directory
    await Deno.mkdir(join(dir, "src"));
    // Run config
    await getConfig();
}
    

// Thoughts: Maybe convention should be that we assume the wsDir is resolved already..?
export async function initPackage(wsDir: string, config: Config) {
    // TODO: Special case for initializing in a potential package directory
    wsDir = Deno.realPathSync(wsDir); // Fix path
    const pkgConfig = await prompt([{
        type: Input,
        name: "name",
        message: "Enter the name of the package",
        // Validate that it is a valid package name (no spaces, all lowercase alphanumeric and underscores)
        validate: (value: string)=>{
            if (value.match(/^[a-z0-9_]+$/)) {
                return true;
            } else {
                return "Invalid package name. Must be all lowercase alphanumeric and underscores";
            }
        }
    }, {
        type: Input,
        name: "description",
        message: "Enter the description of the package"
    }, {
        type: Select,
        name: "buildType",
        message: "Select the build type",
        options: ["ament_cmake", "ament_python", "cmake"],
    }]);
    // Optional node creation
    const createEmptyNode = await prompt([{
        type: Confirm,
        name: "confirm",
        message: "Create empty node?"
    }]);
    let nodeConfig
    if (createEmptyNode.confirm) {
        nodeConfig = await prompt([{
            type: Input,
            name: "name",
            message: "Enter the name of the node"
        }]);
    }
    // Optional license
    const createLicense = await prompt([{
        type: Confirm,
        name: "confirm",
        message: "Select a license?"
    }]);
    const customLicenseString = "[Custom license string]";
    let licenseConfig
    if (createLicense.confirm) {
        licenseConfig = await prompt([{
            type: Select,
            name: "license",
            message: "Select a license",
            options: [...Licenses, customLicenseString]
        }]);
    }
    if (licenseConfig?.license === customLicenseString) {
        licenseConfig.license = (await prompt([{
            type: Input,
            name: "license",
            message: "Enter the license string",
            validate: (value: string)=>{
                if (value.length > 0) {
                    return true;
                } else {
                    return "License string cannot be empty";
                }
            }
        }])).license;
    }
    // Run ros2 pkg create
    const cmd = ["ros2", "pkg", "create", pkgConfig.name as string, "--build-type", pkgConfig.buildType as string, "--description", pkgConfig.description as string,
        ...(createEmptyNode.confirm ? ["--node-name", (nodeConfig?.name as string)] : []),
        ...(createLicense.confirm ? ["--license", (licenseConfig?.license as string)] : [])
        ]
    const wsSrcDir = join(wsDir, "src")
    await runCommands(
        [
            ["source", join(config.ros2Path, "setup.bash")],
            ["source", join(wsDir, "install", "setup.bash")],
            cmd
        ],
        wsSrcDir
    )
}