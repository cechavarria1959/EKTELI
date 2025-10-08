import subprocess
from pathlib import Path

CLANG_PATH = r"C:\Program Files\LLVM\bin\clang-format.exe"
BASE_DIRS = [Path("Core")]
CONFIG_FILE = Path("Clang format/.clang-format")

for base in BASE_DIRS:
    for file_path in base.rglob("*.[ch]"):
        subprocess.run([
            CLANG_PATH, 
            "-i", 
            f"-style=file:{CONFIG_FILE.absolute()}", 
            str(file_path)
        ], check=True)