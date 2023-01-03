// Workspace watcher tools

import { join, relative, isAbsolute } from "https://deno.land/std@0.170.0/path/mod.ts";
import { brightRed, brightGreen, brightYellow, gray } from "https://deno.land/std@0.167.0/fmt/colors.ts";
import { build_packages } from "./macros.ts";
import { assert_package, assert_workspace, find_package } from "./path_finding.ts";
import { Package } from "./package.ts";
import { requireWorkspace } from "./rosa.ts";
import { debounce } from "https://deno.land/std@0.168.0/async/debounce.ts";

// - Monitor packages being added, removed, or modified (timeout specifiable)

/**
 * @class Watcher
 * @description A class for watching a ROS2 workspace for changes
 */
export class Watcher extends EventTarget {
    constructor(root: string, verbose: boolean = false, debounceTime = 200) {
        super();
        // Check if root is valid
        assert_workspace(root);
        this.root = root;
        this.src = join(root, "src");
        this.verbose = verbose;
        this.packageMap = new Map();
        this.debouncerMap = new Map();
        this.debounceTime = debounceTime;
        // Map out all packages
        this.watch();
    }
    root: string;
    src: string;
    fsWatcher: Deno.FsWatcher | null = null;
    verbose: boolean;
    packageMap: Map<string, Package> = new Map();
    debouncerMap: Map<string, Function> = new Map();
    debounceTime: number;

    async parse_package_xml(current_dir: any) {
        const package_xml = await assert_package(current_dir);``
        return package_xml;
    }

    private async watch() {
        await this.initialBuild();
        
        this.verbose || console.log(`🧐 Watching ${this.src} for changes...` );
        this.fsWatcher = Deno.watchFs(this.src);
        
        for await (const event of this.fsWatcher) {
            // If a package.xml file is created, emit a "package_added" event
            // Parse package.xml file and add to map
            if (event.kind === "create" && event.paths[0].endsWith("package.xml")) {
                try {
                    const package_path = join(event.paths[0], "..")
                    const package_info = await assert_package(package_path);
                    // If succesful
                    this.verbose || console.log(brightGreen("[+] ")+`Package added: ${package_info.toStringColor()}`);
                    // Add package to map
                    this.registerPackage(package_info)
                    this.dispatchEvent(new CustomEvent("package_added", { detail: package_info }));
                    continue;
                } catch (e) {
                    throw new Error(`Error parsing package.xml file: ${e}`);
                }
                
            }
            // If a package path is directly modified, and afterwards no longer exists, emit a "package_removed" event
            if ((event.kind === "modify") && this.packageMap.has(event.paths[0])) {
                // Check if package path still exists
                try {
                    Deno.statSync(event
                        .paths[0]);
                } catch (e) {
                    // Package path no longer exists
                    const pp = event.paths[0];
                    this.onPackageRemoved(pp);
                }
                continue;
            }
            // An alternative way to detect package removal is to check if a package.xml file is removed
            if (event.kind === "remove" && event.paths[0].endsWith("package.xml")) { // Gets triggered when a package.xml file is removed
                let p: Package | null = null;
                try {
                    // Let's see if we can find a valid package
                    p = await find_package(join(event.paths[0], ".."), 10);
                } catch (e) {
                    // If we can't find the path, and the path is in the package map, remove it
                    // console.log(event.paths[0], this.packageMap)
                    const packageRoot = join(event.paths[0], "..");
                    if (this.packageMap.has(packageRoot)) {
                        this.onPackageRemoved(packageRoot);
                    }
                }
                // If no package is found, emit a "package_removed" event
                if (p !== null) {
                    this.onPackageRemoved(p.rootPath);
                    continue;
                }
            }
                    
            // If any file is modified, created, or removed, check if it is inside a package. If it is, emit a "package_modified" event
            if (event.kind !== "access") {
                let matched = false;
                for (const path of event.paths) {
                    // Check if path is inside a package
                    for (const [package_path, package_info] of this.packageMap) {
                        if (
                            isSubpath(package_path, path)
                        ) {
                            this.verbose || console.log(brightYellow("[*] ")+`Package modified: ${package_info.toStringColor()} - ${event.kind} ${event.paths}`);
                            this.dispatchEvent(new CustomEvent("package_modified", { detail: package_info }));
                            this.onPackageModifiedDebounced(package_path);
                            matched = true;
                            break;
                        }
                    }
                    if (matched) {
                        break;
                    }
                }
                if (matched) {
                    continue;
                }
            }
            // Unhandled event
            // console.log(gray(`Unhandled event: ${event.kind} ${event.paths}`));
        }
    }

    private onPackageRemoved(package_path: string) {
        // Find package
        const p = this.packageFromPath(package_path);
        this.verbose || console.log(`${brightRed("[-]")} Package removed: ${p.toStringColor()}`);
        // Update package map and debouncer map
        this.deregisterPackage(p);
        this.dispatchEvent(new CustomEvent("package_removed", { detail: p }));
    }

    private onPackageModifiedDebounced(package_path: string) {
        const debouncedEmit = this.debouncerMap.get(package_path);
        if(debouncedEmit === undefined)
        throw new Error(`Debouncer not found in map for path: ${package_path}`);
        debouncedEmit();
    }

    private async initialBuild() {
        this.verbose || console.log("🔎 Searching for packages...");
        // Recursively find "package.xml" files
        // Return a list of package descriptors
        // BFS search for package.xml files
        const queue = [];
        queue.push(this.src);
        while(queue.length > 0) {
            const current_dir = queue.shift() as string;
            const entries = Deno.readDirSync(current_dir);
            for(const entry of entries) {
                if(entry.name === "package.xml") {
                // Add package descriptor to records
                const p = await this.parse_package_xml(current_dir);
                this.verbose || console.log(gray(`Found package: ${p.toStringColor()}`));
                this.registerPackage(p);
                } else if(entry.isDirectory) {
                queue.push(join(current_dir, entry.name));
                }
            }
        }
        this.verbose || console.log(brightGreen(`Found ${this.packageMap.size} package(s) in total.`));

        // Do first build
        const ws = await requireWorkspace();
        let status;
        try {
        status = await build_packages(ws);
        }
        catch(e) {
        throw new Error(`Colcon not found. Please install colcon and try again.`);
        }
        if(status.success) {
        this.verbose || console.log(brightGreen("[+] ") + `Built ${this.packageMap.size} package(s) successfully.`);
        } else {
        this.verbose || console.log(brightRed(`Colcon build failed. Please check your packages and try again.`));
        Deno.exit(1);
        }
    }

    private registerPackage(p: Package) {
        this.packageMap.set(p.rootPath, p);
        this.debouncerMap.set(p.rootPath, debounce(() => {
            // Check if package still exists since it could have been removed before the debounce time
            if(this.packageMap.has(p.rootPath)) {
                this.dispatchEvent(new CustomEvent("package_modified_debounced", { detail: p }));
            }
        }, this.debounceTime));
    }

    private deregisterPackage(p: Package) {
        this.packageMap.delete(p.rootPath);
        this.debouncerMap.delete(p.rootPath);
    }

    private packageFromPath(p: string): Package {
        const rp = this.packageMap.get(p)
        if (rp === undefined) {
            throw new Error("No package found.")
        }
        return rp
    }

    stop() {
        if (this.fsWatcher) {
            this.fsWatcher.close();
        }
    }
}

function isSubpath(parent: string, child: string) {
    // Get relative path from parent to child
    const rpath = relative(parent, child);
    // If not starting with "..", and not absolute path, then it is a subpath
    return !rpath.startsWith("..") && !isAbsolute(rpath);
}