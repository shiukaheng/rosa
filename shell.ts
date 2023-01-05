// Tools for creating subshells

import { copy } from "https://deno.land/std@0.170.0/streams/copy.ts";

export type InteractiveShellArgs = {
    dir?: string;
    initCommands?: string[];
    ps1?: string;
}

/**
 * @class InteractiveShell
 * @description A class for users to interact with shell subprocesses
 */
export class InteractiveShell {
    constructor(args: InteractiveShellArgs = {}) {
        // Get current working directory
        const cwd = args.dir || Deno.cwd();
        // // Change PS1 if specified
        const initCommands: string[] = [];
        if (args.ps1) {
            initCommands.push(`PS1="${args.ps1}"`);
        }
        if (args.initCommands) {
            initCommands.push(...args.initCommands);
        }
        // Run the process
        this.process = Deno.run({
            cmd: ["bash", `-c`, `bash --rcfile <(cat ~/.bashrc; echo '${
                initCommands.join(`;`)
            }')`,
                // // Run init command if specified
                // ...(args.initCommand ? ["-c", args.initCommand] : [])
            ],
            cwd,
        });
        this.running = true;
        this.process.status().then((status) => {
            this.running = false;
            this._onClose.forEach((cb) => cb());
        });
    }
    process: Deno.Process;
    running = false;
    _onClose: (()=>void)[] = [];

    /**
     * @method close
     * @description Closes the shell process
     * @returns Promise<void>
     */
    close(): void {
        return this.process.close();
    }

    /**
     * Add a callback to be called when the shell is closed
     * @param cb the callback to be called when the shell is closed
     */
    onClose(cb: ()=>void): void {
        this._onClose.push(cb);
    }

    /**
     * @method status
     * @description Returns the status of the shell process
     * @returns Promise<Deno.ProcessStatus>
    */
    get status(): Promise<Deno.ProcessStatus> {
        return this.process.status();
    }
}

export type InteractiveShellArgs2 = {
    dir?: string;
    initCommands?: string[][]; // Array of commands, with spaces seperated as seperate strings, so actual spaces can be escaped
    ps1?: string;
}

export function joinCommandStrings(cmd: string[]): string {
    const escaped = cmd.map((str) => str.replace(/ /g, "\\ "));
    return escaped.join(" ");
}

export class InteractiveShell2 {
    constructor(args: InteractiveShellArgs2 = {}) {
        const cwd = args.dir || Deno.cwd();
        const initCommands: string[][] = [];
        if (args.ps1) {
            initCommands.push(["PS1", args.ps1]);
        }
        if (args.initCommands) {
            initCommands.push(...args.initCommands);
        }
        this.process = Deno.run({
            cmd: ["bash", "-c", `bash --rcfile <(cat ~/.bashrc; echo '${
                initCommands.map(joinCommandStrings).join(";")
            }')`],
            cwd,
        });
        this.running = true;
        this.process.status().then((status) => {
            this.running = false;
            this._onClose.forEach((cb) => cb());
        }
        );
        this.running = true;
        this.process.status().then((status) => {
            this.running = false;
            this._onClose.forEach((cb) => cb());
        });
    }
    process: Deno.Process;
    running = false;
    _onClose: (()=>void)[] = [];

    /**
     * @method close
     * @description Closes the shell process
     * @returns Promise<void>
     */
    close(): void {
        return this.process.close();
    }

    /**
     * Add a callback to be called when the shell is closed
     * @param cb the callback to be called when the shell is closed
     */
    onClose(cb: ()=>void): void {
        this._onClose.push(cb);
    }

    /**
     * @method status
     * @description Returns the status of the shell process
     * @returns Promise<Deno.ProcessStatus>
    */
    get status(): Promise<Deno.ProcessStatus> {
        return this.process.status();
    }
}

/**
 * Run commands in a subshell
 * @param commands the commands to run
 * @param dir the directory to run the commands in
 * @param exit whether to automatically inject an exit command at the end
 */
export async function runCommands(commands: string[][], dir?: string, exit=true): Promise<void> {
    const cwd = dir || Deno.cwd();
    if (exit) {
        commands.push(["exit"]);
    }
    const process = Deno.run({
        cmd: ["bash", "-c", `bash --rcfile <(cat ~/.bashrc; echo '${
            commands.map(joinCommandStrings).join(";")
        }')`],
        cwd,
    });
    await process.status();
}

/**
 * Preprocesses a path for use in a bash script
 * @param path the path to preprocess
 */
export function bashPreprocessPath(path: string): string {
    return path.replace(/ /g, "\\ ");
}