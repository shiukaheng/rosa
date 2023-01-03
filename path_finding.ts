// Functions for finding ROS2 workspaces and packages

import { resolve, join } from "https://deno.land/std/path/mod.ts";
import * as parser from "npm:@rgrove/parse-xml"; // Need to use alternative way to import this
import { Package } from "./package.ts";
import { brightRed } from "https://deno.land/std@0.167.0/fmt/colors.ts";
import { InvalidPathError, NotPackageError, NotWorkspaceError } from "./errors.ts";

/**
 * Determines if the given directory is a ROS2 workspace using the criteria:
 * - The directory contains "src", "log", "build", "install" directories
 * @param path The path to check
 */

export async function assert_workspace(proposed_path: string): Promise<void> {
  // Fix the path
  proposed_path = resolve(proposed_path);
  var info;
  try {
    info = await Deno.lstat(proposed_path);
  } catch(e) {
    throw new InvalidPathError(`The path '${proposed_path}' does not exist`);
  }
  if(!info.isDirectory) {
    throw new InvalidPathError(`The path '${proposed_path}' is not a directory`);
  }
  // Check if path contains the required directories
  const required_dirs = ["src", "log", "build", "install"];
  for(const dir of required_dirs) {
    try {
      info = await Deno.lstat(join(proposed_path, dir));
    } catch(e) {
      throw new NotWorkspaceError(`The path '${proposed_path}' does not contain the '${dir}' directory`);
    }
    if(!info.isDirectory) {
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

export async function find_workspace(cd: string, depth: number): Promise<string | null> {
  let current_depth = 0;
  let current_dir = cd;
  while(current_depth < depth) {
    try {
      await assert_workspace(current_dir);
      return current_dir;
    } catch(e) {
      if(e instanceof NotWorkspaceError) {
        // Terminate if we've reached the root directory
        if(current_dir === "/") {
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
/**
 * Convenience function to find the workspace from the current directory
 * @param depth The maximum depth to search
 * @returns The path to the workspace or null if not found
 */

export async function find_workspace_from_cd(depth: number = Infinity): Promise<string | null> {
  // Get the current directory
  const current_dir = Deno.cwd();
  // Find the workspace
  const ws_dir = await find_workspace(current_dir, depth);
  return ws_dir;
}
/**
 * Checks if the given directory is a ROS2 package using the criteria:
 * - The directory contains a "package.xml" file
 * @param proposed_path The path to check
 */

export async function assert_package(proposed_path: string): Promise<Package> {
  // Fix the path
  proposed_path = resolve(proposed_path);
  var info;
  try {
    info = await Deno.lstat(proposed_path);
  } catch(e) {
    throw new InvalidPathError(`The path '${proposed_path}' does not exist`);
  }
  if(!info.isDirectory) {
    throw new InvalidPathError(`The path '${proposed_path}' is not a directory`);
  }
  // Check if path contains the required files
  const required_files = ["package.xml"];
  for(const file of required_files) {
    try {
      info = await Deno.lstat(join(proposed_path, file));
    } catch(e) {
      throw new NotPackageError(`The path '${proposed_path}' does not contain the '${file}' file`);
    }
    if(!info.isFile) {
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
  while(current_depth < depth) {
    try {
      const package_info = await assert_package(current_dir);
      return package_info;
    } catch(e) {
      if(e instanceof NotPackageError) {
        // Terminate if we've reached the root directory
        if(current_dir === "/") {
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
/**
 * Convenience function to find the package from the current directory
 * @param depth The maximum depth to search
 * @returns The package descriptor or null if not found
 */

export async function find_package_from_cd(depth: number=Infinity): Promise<Package | null> {
  // Get the current directory
  const current_dir = Deno.cwd();
  // Find the package
  const package_info = await find_package(current_dir, depth);
  return package_info;
}
/**
 * Gets the configuation file path
 * @param configName The name of the configuration file to use
 * @returns The path to the configuration file
 */

export async function getConfigPath(configName: string = "rosa.config"): Promise<string> {
  // Try to get workspace
  const cwd = Deno.cwd();
  const ws_dir = await find_workspace(cwd, Infinity);
  let config_path: string;
  if(ws_dir === null) {
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
