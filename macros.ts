// import * as path from 'https://deno.land/std@0.102.0/path/mod.ts';
import { resolve, join } from "https://deno.land/std/path/mod.ts";
import { Config } from "./rosm.ts";
import { parse } from "https://deno.land/x/xml/mod.ts";

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
export async function assert_package(proposed_path: string): Promise<object> {
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
    const package_xml = await Deno.readTextFile(join(proposed_path, "package.xml"));
    console.log(package_xml);
    return {"string": package_xml};
}

/**
 * Finds the ROS2 package given the current directory
 * @param cd The directory to start searching from
 * @param depth The maximum depth to search
 * @returns The package descriptor or null if not found
 * @throws InvalidPathError if the path does not exist
 * @throws NotPackageError if the path is not a package
*/
export async function find_package(cd: string, depth: number): Promise<object | null> {
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

export async function find_package_from_cd(config: Config): Promise<object | null> {
    // Get the current directory
    const current_dir = Deno.cwd();
    // Find the package
    const package_info = await find_package(current_dir, config.packageSearchDepth);
    return package_info;
}