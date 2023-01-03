// High level functions meant to be called by the CLI

import { resolve, join, dirname } from "https://deno.land/std/path/mod.ts";
import { Config, getCurrentWorkspace } from "./rosa.ts";
import * as parser from "npm:@rgrove/parse-xml"
import { Package } from "./package.ts";
import { Watcher } from "./watcher.ts";
import { brightRed, brightGreen, yellow, brightYellow, gray, brightMagenta } from "https://deno.land/std@0.167.0/fmt/colors.ts";

class InvalidPathError extends Error {
    constructor(message: string) {
        super(message);
        this.name = "InvalidPathError";
    }
}

class NotWorkspaceError extends Error {
    constructor(message: string) {
        super(message);
        this.name = "NotWorkspaceError";
    }
}

class NotPackageError extends Error {
    constructor(message: string) {
        super(message);
        this.name = "NotPackageError";
    }
}

/**
 * Determines if the given directory is a ROS2 workspace using the criteria:
 * - The directory contains "src", "log", "build", "install" directories
 * @param path The path to check
 */
export async function assert_workspace(proposed_path: string): Promise<void> {
    // Fix the path
    proposed_path = resolve(proposed_path);
    var info
    try {
        info = await Deno.lstat(proposed_path);
    } catch (e) {
        throw new InvalidPathError(`The path '${proposed_path}' does not exist`);
    }
    if (!info.isDirectory) {
        throw new InvalidPathError(`The path '${proposed_path}' is not a directory`);
    }
    // Check if path contains the required directories
    const required_dirs = ["src", "log", "build", "install"];
    for (const dir of required_dirs) {
        try {
            info = await Deno.lstat(join(proposed_path, dir));
        } catch (e) {
            throw new NotWorkspaceError(`The path '${proposed_path}' does not contain the '${dir}' directory`);
        }
        if (!info.isDirectory) {
            throw new NotWorkspaceError(`The path '${proposed_path}' does not contain the '${dir}' directory`);
        }
    }
}

/**
 * Finds the ROS2 workspace given the current directory, which may be:
 * - The root of the workspace
 * - A package in the workspace
 * - A subdirectory of a package in the workspace
 * - Not within a workspace at all
 * 
 * @param cd The directory to start searching from
 * @param depth The maximum depth to search
 */
export async function find_workspace(cd: string, depth: number): Promise<string|null> {
    let current_depth = 0;
    let current_dir = cd;
    while (current_depth < depth) {
        try {
            await assert_workspace(current_dir);
            return current_dir;
        } catch (e) {
            if (e instanceof NotWorkspaceError) {
                // Terminate if we've reached the root directory
                if (current_dir === "/") {
                    break;
                }
                current_dir = join(current_dir, "..");
                current_depth++;
            } else {
                throw e;
            }
        }
    }
    return null;
}

export async function find_workspace_from_cd(config: Config): Promise<string|null> {
    // Get the current directory
    const current_dir = Deno.cwd();
    // Find the workspace
    const ws_dir = await find_workspace(current_dir, config.workspaceSearchDepth);
    return ws_dir;
}

// export interface PackageInfo {
//     name: string;
//     version: string;
//     description: string;
//     maintainer: string;
//     license: string;
// }

/**
 * Checks if the given directory is a ROS2 package using the criteria:
 * - The directory contains a "package.xml" file
 * @param proposed_path The path to check
 */
export async function assert_package(proposed_path: string): Promise<Package> {
    // Fix the path
    proposed_path = resolve(proposed_path);
    var info
    try {
        info = await Deno.lstat(proposed_path);
    } catch (e) {
        throw new InvalidPathError(`The path '${proposed_path}' does not exist`);
    }
    if (!info.isDirectory) {
        throw new InvalidPathError(`The path '${proposed_path}' is not a directory`);
    }
    // Check if path contains the required files
    const required_files = ["package.xml"];
    for (const file of required_files) {
        try {
            info = await Deno.lstat(join(proposed_path, file));
        } catch (e) {
            throw new NotPackageError(`The path '${proposed_path}' does not contain the '${file}' file`);
        }
        if (!info.isFile) {
            throw new NotPackageError(`The path '${proposed_path}' does not contain the '${file}' file`);
        }
    }
    // Parse the package.xml file
    const package_xml = parser.parseXml(await Deno.readTextFile(join(proposed_path, "package.xml")));
    return new Package(package_xml, proposed_path);
}

/**
 * Finds the ROS2 package given the current directory
 * @param cd The directory to start searching from
 * @param depth The maximum depth to search
 * @returns The package descriptor or null if not found
 * @throws InvalidPathError if the path does not exist
 * @throws NotPackageError if the path is not a package
*/
export async function find_package(cd: string, depth: number): Promise<Package | null> {
    let current_depth = 0;
    let current_dir = cd;
    while (current_depth < depth) {
        try {
            const package_info = await assert_package(current_dir);
            return package_info;
        } catch (e) {
            if (e instanceof NotPackageError) {
                // Terminate if we've reached the root directory
                if (current_dir === "/") {
                    break;
                }
                current_dir = join(current_dir, "..");
                current_depth++;
            } else {
                throw e;
            }
        }
    }
    return null;
}

export async function find_package_from_cd(config: Config): Promise<Package | null> {
    // Get the current directory
    const current_dir = Deno.cwd();
    // Find the package
    const package_info = await find_package(current_dir, config.packageSearchDepth);
    return package_info;
}

export async function build_packages(cwd: string, args: string[]=[]) {
    // To be extended with a package list
    // For now, just build all packages in the workspace
    const process = Deno.run({
        cmd: ["colcon", "build", ...args],
        cwd: cwd
    });
    const status = await process.status();
    return status;
}

export async function build_package(cwd: string, package_name: string, args: string[]=[]) {
    // To be extended with a package list
    // For now, just build all packages in the workspace
    const process = Deno.run({
        cmd: ["colcon", "build", "--packages-select", package_name, ...args],
        cwd: cwd
    });
    const status = await process.status();
    return status;
}

export async function createWatcherBuilder(): Promise<Watcher> {
    // Start watcher
    const ws_dir = await getCurrentWorkspace();
    const watcher = new Watcher(ws_dir);
    watcher.addEventListener("package_modified_debounced", (e)=>{
        console.log(`Package modified: ${e.detail.toStringColor()}, rebuilding...`);
        build_package(ws_dir, e.detail.name).then((status)=>{
            if (status.success) {
                console.log(`✅ Build successful for ${e.detail.toStringColor()}`);
            } else {
                console.log(`❌ Build failed for ${e.detail.toStringColor()}`);
            }
        });
    })
    return watcher;
}

export async function getConfigPath(configName:string="rosa.config"): Promise<string> {
    // Try to get workspace
    const cwd = Deno.cwd();
    const ws_dir = await find_workspace(cwd, Infinity);
    let config_path: string;
    if (ws_dir === null) {
        console.log(brightRed(`Please run this command from within a ROS2 workspace`));
        Deno.exit(1);
        // const script_path = await Deno.realPath(Deno.execPath());
        // config_path = join(dirname(script_path), configName);
        // console.log(`Not currently in a workspace, using global config at ${config_path}`);
    } else {
        config_path = join(ws_dir, configName);
    }
    return config_path;
}