import subprocess
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py as _build_py


def merge_proto_files(
    proto_path: Path, dest_file: Path, package: str, syntax: str
) -> None:
    SKIP_HEADERS = {"import", "syntax", "package"}

    def create_header() -> str:
        return (
            f'syntax = "{syntax}";\n'
            f"package {package};\n\n"
            'import "google/protobuf/any.proto";\n\n'
        )

    def filter_content(content: str) -> list[str]:
        return [
            line
            for line in content.splitlines()
            if not any(line.strip().startswith(header) for header in SKIP_HEADERS)
        ]

    # Find all proto files
    proto_files = list(proto_path.rglob("*.proto"))
    if not proto_files:
        raise FileNotFoundError(f"No .proto files found in directory: {proto_path}")

    # Write merged content
    with dest_file.open("w") as output_file:
        output_file.write(create_header())

        for proto_file in proto_files:
            relative_path = proto_file.relative_to(proto_path)
            print(f"Merging: {relative_path} -> {dest_file.name}")

            content = proto_file.read_text()
            filtered_lines = filter_content(content)
            output_file.write("\n".join(filtered_lines) + "\n\n")


class BuildPyCommand(_build_py):
    def run(self):
        print("=== Starting custom build_py command ===")
        try:
            self._process_proto_files()
            super().run()
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                f"Build failed: {e.stderr.decode() if e.stderr else str(e)}"
            )

    def _process_proto_files(self) -> None:
        cwd = Path.cwd()
        proto_src = cwd
        protocol_file = cwd / "protocol.proto"

        # Merge proto files
        merge_proto_files(proto_src, protocol_file, "aimdk.protocol", "proto3")

        # Compile proto files
        subprocess.run(
            [
                "protoc",
                "--python_out=.",
                "--experimental_allow_proto3_optional",
                "protocol.proto",
            ],
            check=True,
        )


def get_version() -> str:
    version_file = Path("VERSION")
    return version_file.read_text().strip()


setup(
    name="aimdk-a-3",
    version=get_version(),
    packages=["aimdk"],
    package_dir={
        "aimdk": ".",
    },
    cmdclass={
        "build_py": BuildPyCommand,
    },
)
