import { copy } from "https://deno.land/std@0.170.0/streams/copy.ts";

export type InteractiveShellArgs = {
    dir?: string;
    initCommands?: string[];
    ps1: string;
}

/**
 * @class Shell
 * @description A class for interacting with shell subprocesses
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