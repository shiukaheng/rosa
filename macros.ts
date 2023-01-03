// High level functions meant to be called by the CLI

import { requireWorkspace } from "./rosa.ts";
import { Package } from "./package.ts";
import { Watcher } from "./watcher.ts";

/**
 * Builds all packages in the workspace
 * @param cwd The directory to start searching from
 * @param args Additional arguments to pass to colcon
 * @returns The status of the build process
 */
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

/**
 * Builds a single package in the workspace
 * @param cwd The directory to start searching from
 * @param package_name The name of the package to build
 * @param args Additional arguments to pass to colcon
 * @returns The status of the build process
 */
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

/**
 * Creates a watcher for all the packages of the current workspace
 * @returns The watcher instance
 */
export async function createWatcherBuilder(): Promise<Watcher> {
    // TODO: require path
    // Start watcher
    const ws_dir = await requireWorkspace();
    const watcher = new Watcher(ws_dir);
    watcher.addEventListener("package_modified_debounced", (e)=>{
        // @ts-ignore - events.detail is not typed
        const pkg = e.detail as Package;
        console.log(`Package modified: ${pkg.toStringColor()}, rebuilding...`);
        build_package(ws_dir, pkg.name).then((status)=>{
            if (status.success) {
                console.log(`✅ Build successful for ${pkg.toStringColor()}`);
            } else {
                console.log(`❌ Build failed for ${pkg.toStringColor()}`);
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
export async function createPackageWatcherBuilder(package_name: string): Promise<Watcher> {
    // Start watcher
    const ws_dir = await requireWorkspace();
    const watcher = new Watcher(ws_dir);
    watcher.addEventListener("package_modified_debounced", (e)=>{
        // @ts-ignore - events.detail is not typed
        const pkg = e.detail as Package;
        if (pkg.name === package_name) {
            console.log(`Package modified: ${pkg.toStringColor()}, rebuilding...`);
            build_package(ws_dir, pkg.name).then((status)=>{
                if (status.success) {
                    console.log(`✅ Build successful for ${pkg.toStringColor()}`);
                } else {
                    console.log(`❌ Build failed for ${pkg.toStringColor()}`);
                }
            });
        }
    })
    return watcher;
}