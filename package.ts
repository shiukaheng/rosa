import { brightRed, brightGreen, gray, brightBlue, white } from "https://deno.land/std@0.167.0/fmt/colors.ts";
import * as parser from "npm:@rgrove/parse-xml"

export class Package {
    // name: string;
    rootPath: string;
    rawXml: parser.XmlDocument;
    name: string;
    version: string;
    description: string;
    constructor(xml: parser.XmlDocument, rootPath: string) {
        this.rootPath = rootPath;
        try {
            // @ts-ignore Bypass type checking for XML parsing
            const package_node = xml.children.find((node) => node.name === "package");
            if (package_node === undefined) {
                throw new Error("Package node not found");
            }
            this.rawXml = xml;
            if (!(package_node instanceof parser.XmlElement)) {
                throw new Error("Package node is not an XmlElement");
            }
            // @ts-ignore Bypass type checking for XML parsing
            this.name = package_node.children.find((node) => node.name === "name").text
            // @ts-ignore same as above
            this.version = package_node.children.find((node) => node.name === "version").text 
            // @ts-ignore and again
            this.description = package_node.children.find((node) => node.name === "description").text
        }
        catch (e) {
            throw new Error(`Failed to parse package.xml: ${e}`);
        }
    }
    toString() {
        // Return <name>@<version>
        // But, if folder name is different from package name, return <name>@<version> (<folder name>)
        const folder_name = this.rootPath.split("/").pop();
        if (folder_name === this.name) {
            return `${this.name}@${this.version}`;
        } else {
            return `${this.name}@${this.version} (${folder_name})`;
        }
    }
    toStringColor(): string {
        // return `${brightGreen(this.name as string)}@${white(this.version as string)}`
        const folder_name = this.rootPath.split("/").pop();
        if (folder_name === this.name) {
            return `${brightGreen(this.name as string)}@${white(this.version as string)}`;
        } else {
            return `${brightGreen(this.name as string)}@${white(this.version as string)} (${brightBlue(folder_name as string)})`;
        }
    }
}