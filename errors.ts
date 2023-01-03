// Error classes for the package

export class InvalidPathError extends Error {
    constructor(message: string) {
        super(message);
        this.name = "InvalidPathError";
    }
}

export class NotWorkspaceError extends Error {
    constructor(message: string) {
        super(message);
        this.name = "NotWorkspaceError";
    }
}

export class NotPackageError extends Error {
    constructor(message: string) {
        super(message);
        this.name = "NotPackageError";
    }
}