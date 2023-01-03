import { Confirm, prompt, Select } from "https://deno.land/x/cliffy@v0.25.6/prompt/mod.ts";
import { z } from "https://deno.land/x/zod@v3.20.2/mod.ts";
import { join } from "https://deno.land/std@0.170.0/path/mod.ts";
import { getConfigPath } from "./path_finding.ts";

const configSchema = z.object({
    ros2Path: z.string()
});

export type Config = z.infer<typeof configSchema>;

export const defaultConfig = { // Everything should be present, if no default can be provided, it should be null
    ros2Path: null as string | null,
}

export default class RosaConfig implements Config {
    configPath: string;
    private _config = defaultConfig; // Does not guarantee that config is valid, but the getters will
    private isLoaded = false;
    asyncInit: Promise<void> | null = null;
    
    constructor(configPath: string) {
        this.configPath = configPath;
        // If config file exists, load it
        try {
            this.loadConfig();
            // console.log(`✅ Loaded config file from ${this.configPath}`);
        } catch (e) {
            // If config file does not exist, create it
            try {
                this.asyncInit = this.createConfig();
            } catch (e) {
                console.log(`❌ Failed to create config file: ${e}`);
                Deno.exit(1);
            }
        }
    }

    /**
     * Loads the config file from the config path
     */
    loadConfig() {
        const config = JSON.parse(Deno.readTextFileSync(this.configPath));
        // Validate config
        const result = configSchema.safeParse(config);
        if (!result.success) {
            console.log(`❌ Your config file is invalid: ${result.error}`);
            this.promptConfig();
            return;
        }
        // Check if ROS2 path is valid
        try {
            const info = Deno.lstatSync(config.ros2Path);
            if (!info.isDirectory) {
                console.log(`❌ The ROS2 path '${config.ros2Path}' is not a directory`);
                Deno.exit(1);
            }
        } catch (e) {
            console.log(`Running first-time setup...`);
            Deno.exit(1);
        }
        this._config = result.data as Config;
    }

    /**
     * Called when config file is invalid
     */
    async promptConfig() {
        console.log(`Your config file is invalid. Would you like to create a new one? (y/n)`);
        const result = await prompt({
            // @ts-ignore - cliffy types are wrong?
            type: Confirm,
            name: "confirm",
            message: "Create new config file?",
            initial: true,
        });
        if (result.confirm) {
            this.createConfig();
            return;
        } else {
            Deno.exit(1);
        }
    }

    /**
     * Runs wizard to create a new config file
    */
    async createConfig() {
        console.log(`Creating new config file at ${this.configPath}`);
        // Search for possible ROS2 paths
        const ros2Paths = await this.findRos2Paths();
        // Prompt user for ROS2 path
        const ros2Path = await Select.prompt({
            message: "Select ROS2 path",
            options: ros2Paths,
        });
        // Hidden settings (not prompted)
        // NA
        // Create config object
        this._config = {
            ros2Path: ros2Path,
        };
        // Write config to file
        this.writeConfig();
    }

    /**
     * Reconfgures the config file
     */
    async reconfigure() {
        // Delete config file
        Deno.removeSync(this.configPath);
        // Create new config file
        await this.createConfig();
    }

    /**
     * Writes the config object to the config file
     */
    writeConfig() {
        Deno.writeTextFileSync(this.configPath, JSON.stringify(this._config, null, 4));
    }

    /**
     * Finds possible ROS2 paths
     * @returns Array of possible ROS2 paths
     * @throws Error if no ROS2 paths are found
     */
    async findRos2Paths(): Promise<string[]> {
        // Assume all ROS2 paths are in /opt/ros. List all directories in /opt/ros
        const ros2Paths: string[] = [];
        const rootDir = "/opt/ros"
        for await (const dirEntry of Deno.readDir(rootDir)) {
            if (dirEntry.isDirectory) {
                ros2Paths.push(join(rootDir, dirEntry.name));
            }
        }
        if (ros2Paths.length === 0) {
            throw new Error("No ROS2 paths found, please install ROS2");
        }
        return ros2Paths;
    }

    get ros2Path(): string {
        if (this._config.ros2Path === null) {
            throw new Error("ROS2 path is null, make sure to call promptConfig() first");
        }
        return this._config.ros2Path;
    }
}

// Get config (stateful)
export let config: RosaConfig | null = null;
export async function getConfig(): Promise<Config> {
    // TODO: Make it take in a workspace path
    if (!config) {
        // Load config or prompt creation if required
        const configPath = await getConfigPath();
        config = new RosaConfig(configPath);
        await config.asyncInit;

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
    }
    return config;
}