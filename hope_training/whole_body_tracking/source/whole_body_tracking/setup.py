"""Installation script for the ``whole_body_tracking`` Isaac Lab extension."""

import os

import toml
from setuptools import setup

# Read the extension metadata (single source of truth for version / author / description).
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

# Minimum runtime dependencies (Isaac Lab itself is provided by the base install).
INSTALL_REQUIRES = [
    "psutil",
    "onnx",
    "onnxscript",
    "pyyaml",
    # HOPEOnPolicyRunner overrides OnPolicyRunner._prepare_logging_writer, which exists in the
    # rsl_rl 3.x line only — earlier releases would silently keep their default W&B/TB wiring.
    "rsl-rl-lib>=3.0.0,<4",
]

setup(
    name="whole_body_tracking",
    packages=["whole_body_tracking"],
    author=EXTENSION_TOML_DATA["package"]["author"],
    maintainer=EXTENSION_TOML_DATA["package"]["maintainer"],
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    install_requires=INSTALL_REQUIRES,
    license="Apache-2.0",
    include_package_data=True,
    python_requires=">=3.10",
    classifiers=[
        "License :: OSI Approved :: Apache Software License",
        "Natural Language :: English",
        "Programming Language :: Python :: 3.10",
        "Isaac Sim :: 4.0.0",
    ],
    zip_safe=False,
)
