import os
import ast

def get_definitions_and_imports(file_path):
    """Parses a Python file to find imports (interactions) and definitions."""
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            tree = ast.parse(f.read())
    except Exception:
        return [], []

    definitions = []
    imports = []

    for node in tree.body:
        # Capture Classes and Functions
        if isinstance(node, ast.FunctionDef):
            args = [a.arg for a in node.args.args if a.arg != 'self']
            definitions.append(f"  └── def {node.name}({', '.join(args)})")
        elif isinstance(node, ast.ClassDef):
            definitions.append(f"  └── class {node.name}")
            for item in node.body:
                if isinstance(item, ast.FunctionDef):
                    args = [a.arg for a in item.args.args if a.arg != 'self']
                    definitions.append(f"      └── def {item.name}({', '.join(args)})")

        # Capture Interactions (Imports)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                imports.append(alias.name)
        elif isinstance(node, ast.ImportFrom):
            module = node.module if node.module else ""
            for alias in node.names:
                imports.append(f"{module}.{alias.name}")

    return definitions, imports

def generate_map(root_dir):
    ignore_dirs = {'.git', '__pycache__', 'venv', 'env', 'node_modules', '.next', 'dist', 'build'}
    
    print(f"# CODEBASE SKELETON: {os.path.basename(os.path.abspath(root_dir))}")
    print("# This section lists files, their contents, and who they talk to (Imports).")
    
    for dirpath, dirnames, filenames in os.walk(root_dir):
        # Filter directories
        dirnames[:] = [d for d in dirnames if d not in ignore_dirs and not d.startswith('.')]
        
        level = dirpath.replace(root_dir, '').count(os.sep)
        indent = ' ' * 4 * level
        
        # Don't print root dot
        if level > 0:
            print(f"{indent}📂 {os.path.basename(dirpath)}/")
        
        subindent = ' ' * 4 * (level + 1)
        for f in sorted(filenames):
            if f.endswith(".py"):
                file_path = os.path.join(dirpath, f)
                defs, interacts = get_definitions_and_imports(file_path)
                
                print(f"{subindent}🐍 {f}")
                
                # Print Interactions (Imports) first so LLM sees dependencies
                if interacts:
                    print(f"{subindent}  [Interacts with: {', '.join(interacts[:5])}{'...' if len(interacts)>5 else ''}]")
                
                # Print Definitions
                for d in defs:
                    print(f"{subindent}{d}")

if __name__ == "__main__":
    generate_map(".")



